#include <vector>

#include "base/base.h"
#include "dram/dram.h"
#include "addr_mapper/addr_mapper.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

class LinearMapperBase : public IAddrMapper {
  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1;

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;


  protected:
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;
    }

};


class ChRaBaRoCo final : public LinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, ChRaBaRoCo, "ChRaBaRoCo", "Applies a trival mapping to the address.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      for (int i = m_addr_bits.size() - 1; i >= 0; i--) {
        req.addr_vec[i] = slice_lower_bits(addr, m_addr_bits[i]);
      }
    }
};


class RoBaRaCoCh final : public LinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoBaRaCoCh, "RoBaRaCoCh", "Applies a RoBaRaCoCh mapping to the address.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);
      req.addr_vec[m_addr_bits.size() - 1] = slice_lower_bits(addr, m_addr_bits[m_addr_bits.size() - 1]);
      for (int i = 1; i <= m_row_bits_idx; i++) {
        req.addr_vec[i] = slice_lower_bits(addr, m_addr_bits[i]);
      }
    }
};

/*
 * RoBaRaCoChXOR -- RoBaRaCoCh with the row index XORed into the bank/bankgroup/
 * pseudochannel fields.
 *
 * Motivation (measured): PagedAttention gives the KV cache a power-of-two block
 * stride. The bank field ends below the row field, so a stride that is a multiple
 * of 2^16 leaves the bank UNCHANGED and only changes the row -- the stream keeps
 * returning to a bank it just used, needing a different row each time. Measured
 * on LLaMA-2 16k decode with 16-token blocks: same-bank-different-row pairs go
 * from 0.0% (contiguous) to 10.7% (paged), and while-busy bandwidth falls from
 * 96.4% to 65.9%.
 *
 * XORing row bits into the bank bits makes the bank index advance whenever the
 * row changes, so a power-of-two stride can no longer alias onto one bank. This
 * costs nothing: the mapping stays bijective, so every address still has exactly
 * one location and decoding is unchanged.
 */
class RoBaRaCoChXOR final : public LinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoBaRaCoChXOR, "RoBaRaCoChXOR", "RoBaRaCoCh with row bits XORed into the bank fields.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);
      req.addr_vec[m_addr_bits.size() - 1] = slice_lower_bits(addr, m_addr_bits[m_addr_bits.size() - 1]);
      for (int i = 1; i <= m_row_bits_idx; i++) {
        req.addr_vec[i] = slice_lower_bits(addr, m_addr_bits[i]);
      }
      /* `addr` now holds the row. Fold it down into every level between the
         column and the row, lowest row bits first. */
      Addr_t row = req.addr_vec[m_row_bits_idx];
      int shift = 0;
      for (int lvl = m_row_bits_idx - 1; lvl >= 1; lvl--) {
        int bits = m_addr_bits[lvl];
        if (bits <= 0) continue;
        req.addr_vec[lvl] ^= (int)((row >> shift) & ((1 << bits) - 1));
        shift += bits;
      }
    }
};

class RoCoBaCh final : public LinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoCoBaCh, "RoCoBaCh",
      "Row-Column-Bank-Channel: bank bits sit BELOW the column field.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase::setup(frontend, memory_system);
    }

    /*
     * RoBaRaCoCh consumes column bits first, so a whole row is walked in one
     * bank before the bank index advances -- long runs per bank, and a walk
     * that oscillates over more than a row keeps re-entering it.
     *
     * Here the bank levels are consumed FIRST, so consecutive transactions
     * cycle through every bank before the column advances. A contiguous
     * stretch of (banks x row_bytes) then shares ONE row index, so an access
     * stream that wanders within that stretch finds its row still open.
     * Row length is unchanged; only the bit positions move.
     */
    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);          // channel
      for (int i = 1; i < m_row_bits_idx; i++)                           // pch, bg, bank
        req.addr_vec[i] = slice_lower_bits(addr, m_addr_bits[i]);
      req.addr_vec[m_col_bits_idx] =
          slice_lower_bits(addr, m_addr_bits[m_col_bits_idx]);           // column
      req.addr_vec[m_row_bits_idx] =
          slice_lower_bits(addr, m_addr_bits[m_row_bits_idx]);           // row
    }
};

class M2NDP final : public LinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IAddrMapper, M2NDP, "M2NDP",
      "Applies a RoBaRaCoCh mapping to the address.");

 public:
  void init() override{};

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    LinearMapperBase::setup(frontend, memory_system);
  }

  void apply(Request& req) override {
    req.addr_vec.resize(m_num_levels, -1);
    Addr_t addr = req.addr >> m_tx_offset;
    int c_index = m_addr_bits.size() - 1;
    int c_lower = 8 - m_tx_offset;
    int c_higher = m_addr_bits[c_index] - c_lower;
    req.addr_vec[0] = slice_lower_bits(addr, m_addr_bits[0]);
    int c_lower_addr = slice_lower_bits(addr, c_lower);
    req.addr_vec[1] = slice_lower_bits(addr, m_addr_bits[1]);
    req.addr_vec[c_index] = (slice_lower_bits(addr, c_higher) << c_lower) + c_lower_addr;
    for (int i = 2; i <= m_row_bits_idx; i++) {
      req.addr_vec[i] = slice_lower_bits(addr, m_addr_bits[i]);
    }
  }
};

class MOP4CLXOR final : public LinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, MOP4CLXOR, "MOP4CLXOR", "Applies a MOP4CLXOR mapping to the address.");

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      LinearMapperBase::setup(frontend, memory_system);
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;
      req.addr_vec[m_col_bits_idx] = slice_lower_bits(addr, 2);
      for (int lvl = 0 ; lvl < m_row_bits_idx ; lvl++)
          req.addr_vec[lvl] = slice_lower_bits(addr, m_addr_bits[lvl]);
      req.addr_vec[m_col_bits_idx] += slice_lower_bits(addr, m_addr_bits[m_col_bits_idx]-2) << 2;
      req.addr_vec[m_row_bits_idx] = (int) addr;

      int row_xor_index = 0; 
      for (int lvl = 0 ; lvl < m_col_bits_idx ; lvl++){
        if (m_addr_bits[lvl] > 0){
          int mask = (req.addr_vec[m_col_bits_idx] >> row_xor_index) & ((1<<m_addr_bits[lvl])-1);
          req.addr_vec[lvl] = req.addr_vec[lvl] xor mask;
          row_xor_index += m_addr_bits[lvl];
        }
      }
    }
};

}   // namespace Ramulator