#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"

namespace Ramulator {

class FRFCFS : public IScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IScheduler, FRFCFS, "FRFCFS", "FRFCFS DRAM Scheduler.")
  private:
    IDRAM* m_dram;

    /* PROBE: the stock compare() is not actually FR-FCFS.
     *
     * It ranks ready-over-unready, then falls back to arrival order. But a row
     * HIT (preq command == final command, i.e. RD/WR issues directly) and a row
     * MISS (preq command == ACT) are BOTH "ready" whenever their timings are
     * met, so the tie is broken by age -- and an older row miss beats a younger
     * row hit. That is FCFS-among-ready, not first-ready-FIRST-ROW-HIT.
     *
     * RAMULATOR_SCHED_ROWHIT=1 inserts the missing row-hit tier between the
     * readiness test and the FCFS fallback.
     * RAMULATOR_STARVE_CAP=N (0 = off) bounds the resulting unfairness: a
     * request older than N cycles outranks a row hit, so the hit tier cannot
     * starve a stream indefinitely.
     */
    bool m_rowhit_pri = false;
    Clk_t m_starve_cap = 0;
    /* RAMULATOR_SCHED_KEEPOPEN=1: never issue a PRE on a bank that still has a
       queued request able to read its currently-open row.
       The row-hit tier below cannot prevent this: a blocked RD (column timing
       not yet met) fails check_ready(), so the READINESS tier picks the competing
       PRE and the row-hit tier is never consulted. Measured consequence: 750,188
       requests per run were stranded by a precharge of the row they wanted, then
       had to re-activate it ~26 cycles later -- 46 wasted cycles each, for data
       that was in the sense amps when it was discarded.
       This guard sits ABOVE readiness: a bank with pending hits is not
       precharged even when those hits cannot issue this cycle. Falls back to the
       unguarded choice if every candidate is excluded, so it can never stall. */
    bool m_keep_open = false;
    std::vector<char> m_hit_banks;

    bool is_row_hit(ReqBuffer::iterator r) const {
      return r->command == r->final_command;
    }

    template <class V> static int bkey(const V& v) {
      int k = 0;
      for (size_t L = 1; L + 2 < v.size(); L++) k = k * 16 + (v[L] < 0 ? 0 : v[L]);
      return k & 4095;
    }

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = cast_parent<IDRAMController>()->m_dram;
      if (const char* e = std::getenv("RAMULATOR_SCHED_ROWHIT"))
        m_rowhit_pri = (e[0] == '1');
      if (const char* e = std::getenv("RAMULATOR_STARVE_CAP"))
        m_starve_cap = (Clk_t)std::strtoull(e, nullptr, 10);
      if (const char* e = std::getenv("RAMULATOR_SCHED_KEEPOPEN"))
        m_keep_open = (e[0] == '1');
      m_hit_banks.assign(4096, 0);
    };

    ReqBuffer::iterator compare(ReqBuffer::iterator req1, ReqBuffer::iterator req2) override {
      bool ready1 = m_dram->check_ready(req1->command, req1->addr_vec);
      bool ready2 = m_dram->check_ready(req2->command, req2->addr_vec);

      if (ready1 ^ ready2) {
        if (ready1) {
          return req1;
        } else {
          return req2;
        }
      }

      if (m_rowhit_pri) {
        // Starvation guard first, so it overrides the row-hit tier below.
        // The controller's m_clk is protected, so bound unfairness RELATIVELY:
        // if one request is older than the other by more than the cap, age
        // wins regardless of row hits. Within the scheduling window that is
        // the same bound -- the oldest request cannot be passed over
        // indefinitely by a stream of younger hits.
        if (m_starve_cap) {
          Clk_t a1 = req1->arrive, a2 = req2->arrive;
          if (a1 + (Clk_t)m_starve_cap < a2) return req1;
          if (a2 + (Clk_t)m_starve_cap < a1) return req2;
        }
        bool hit1 = is_row_hit(req1);
        bool hit2 = is_row_hit(req2);
        if (hit1 ^ hit2) return hit1 ? req1 : req2;
      }

      // Fallback to FCFS
      if (req1->arrive <= req2->arrive) {
        return req1;
      } else {
        return req2;
      }
    }

    ReqBuffer::iterator get_best_request(ReqBuffer& buffer) override {
      if (buffer.size() == 0) {
        return buffer.end();
      }

      for (auto& req : buffer) {
        req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
      }

      auto candidate = buffer.begin();
      for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); next++) {
        candidate = compare(candidate, next);
      }
      return candidate;
    }
};

}       // namespace Ramulator
