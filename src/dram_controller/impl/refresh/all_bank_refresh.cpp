#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/refresh.h"

namespace Ramulator {

class AllBankRefresh : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, AllBankRefresh, "AllBank", "All-Bank Refresh scheme.")
  private:
    Clk_t m_clk = 0;
    IDRAM* m_dram;
    IDRAMController* m_ctrl;

    int m_dram_org_levels = -1;
    int m_num_ranks = -1;
    bool m_has_rank = false;

    int m_nrefi = -1;
    int m_ref_req_id = -1;
    Clk_t m_next_refresh_cycle = -1;

  public:
    void init() override { 
      m_ctrl = cast_parent<IDRAMController>();
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = m_ctrl->m_dram;

      m_dram_org_levels = m_dram->m_levels.size();
      /* BUGFIX: not every DRAM model has a "rank" level -- HBM3 is
         channel/pseudochannel/bankgroup/bank. get_level_size() returns -1 for a
         missing level, so m_num_ranks was -1 and the loop in tick() never
         executed: refresh was silently disabled for the entire simulation
         (visible as "other 0" in the controller's command mix). REFab is
         channel-scoped (see HBM3 m_command_scopes), so when there is no rank
         level a single channel-wide refresh request is the correct form. */
      m_num_ranks = m_dram->get_level_size("rank");
      m_has_rank  = (m_num_ranks >= 1);
      if (!m_has_rank) m_num_ranks = 1;

      m_nrefi = m_dram->m_timing_vals("nREFI");
      m_ref_req_id = m_dram->m_requests("all-bank-refresh");

      m_next_refresh_cycle = m_nrefi;

      if (m_ctrl->m_channel_id == 0) {
        spdlog::info("[REFRESH] AllBank enabled: nREFI={} nRFC={} rank_level={} reqs_per_interval={}",
                     m_nrefi, m_dram->m_timing_vals("nRFC"),
                     m_has_rank ? "yes" : "no (channel-scoped)", m_num_ranks);
      }
    };

    void tick() {
      m_clk++;

      if (m_clk == m_next_refresh_cycle) {
        m_next_refresh_cycle += m_nrefi;
        for (int r = 0; r < m_num_ranks; r++) {
          std::vector<int> addr_vec(m_dram_org_levels, -1);
          addr_vec[0] = m_ctrl->m_channel_id;
          if (m_has_rank) addr_vec[1] = r;
          Request req(addr_vec, m_ref_req_id);

          bool is_success = m_ctrl->priority_send(req);
          if (!is_success) {
            throw std::runtime_error("Failed to send refresh!");
          }
        }
      }
    };

};

}       // namespace Ramulator
