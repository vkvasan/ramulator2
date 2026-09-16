#include <map>

#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

class GenericDRAMController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, GenericDRAMController, "Generic", "A generic DRAM controller.");
  private:
    std::deque<Request> pending;          // A queue for read requests that are about to finish (callback after RL)

    ReqBuffer m_active_buffer;            // Buffer for requests being served. This has the highest priority 
    ReqBuffer m_priority_buffer;          // Buffer for high-priority requests (e.g., maintenance like refresh).
    ReqBuffer m_read_buffer;              // Read request buffer
    ReqBuffer m_write_buffer;             // Write request buffer

    int m_row_addr_idx = -1;

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    bool  m_is_write_mode = false;

    std::vector<IControllerPlugin*> m_plugins;

    size_t s_num_row_hits = 0;
    size_t s_num_row_misses = 0;
    size_t s_num_row_conflicts = 0;

    /* Why is no command issued this tick? Buckets partition every tick. */
    /*
     * Split hit/miss/conflict by address region. The controller sees ONNXim's
     * stripped address: ram_addr = (dram_address >> 9) << 5, i.e. orig/16.
     * RAMULATOR_WEIGHT_LIMIT is given in ORIGINAL bytes; anything below it is
     * weights, anything above is KV cache / activations.
     */
    /*
     * Precharge audit. A PRE is legitimate if the request that triggered it
     * then activates the same bank for its own row. If some OTHER request
     * activates it, or the bank sits idle a long time, the row was closed for
     * a request that never got served -- and a later request to that bank
     * records a MISS instead of a hit.
     */
    struct PreRec { Addr_t addr; Clk_t clk; };
    std::map<int, PreRec> s_last_pre;      // bank key -> the PRE that closed it
    size_t s_pre_total = 0, s_pre_act_same = 0, s_pre_act_other = 0, s_owner_hits = 0;
    uint64_t s_pre_to_act_cycles = 0;

    /*
     * Precharge-owner priority (RAMULATOR_PRE_OWNER=1).
     * Standard FR-FCFS picks whichever request is ready, so a bank that A just
     * precharged is frequently activated by some other request B -- measured at
     * 85.6% of precharges. A then finds the bank holding B's row and is charged
     * a fresh conflict/miss. This rule reserves a just-precharged bank for the
     * request that closed it, until that request has issued its ACT.
     */
    /* Periodic dump of the row counters so they can be diffed per operation. */
    FILE* s_rowtrace = nullptr;
    uint64_t s_rowtrace_every = 2000;

    bool s_pre_owner_policy = false;
    std::map<int, Addr_t> s_pre_owner;      // bank key -> addr that precharged it

    Addr_t s_weight_limit = 0;
    size_t s_hit_w = 0, s_miss_w = 0, s_conf_w = 0;
    size_t s_hit_k = 0, s_miss_k = 0, s_conf_k = 0;
    size_t s_hit_a = 0, s_miss_a = 0, s_conf_a = 0;   /* activations (operand-tagged split) */
    uint64_t s_lat_sum = 0, s_lat_n = 0, s_lat_min = 0;  /* read latency in controller cycles */
    /* Conflict attribution: which stream OWNED the open row that a conflicting
       request had to close. Owner = class of the request that issued the last
       ACT on that bank. s_conf_<requester>_by_<owner>. */
    std::map<int, bool> s_row_owner_w;   // bank key -> open row belongs to weights?
    size_t s_conf_k_by_w = 0, s_conf_k_by_k = 0, s_conf_w_by_w = 0, s_conf_w_by_k = 0;
    /* CLOSE AUDIT: a "miss" means the bank was found with NO row open, so either
       it was never activated or something closed it. Attribute every KV miss:
         first touch   - bank never activated before
         demand PRE    - a request precharged it (the conflict path)
         refresh PREA  - the refresh manager closed all banks
       and flag the pathological case where the row we now want is the very row
       that was thrown away, which is a row we should never have closed. */
    struct CloseRec { Clk_t clk = 0; int row = -1; int operand = 0; };
    std::map<int, CloseRec> s_closed;       // bank -> how it was last closed
    std::map<int, int>      s_open_row;     // bank -> row currently open
    std::map<int, bool>     s_ever_opened;  // bank -> has ever been ACTed
    Clk_t  s_prea_clk = 0;                  // last channel-wide PREA (refresh)
    size_t s_miss_first = 0, s_miss_by_pre = 0, s_miss_by_refresh = 0;
    size_t s_miss_samerow = 0, s_miss_self = 0, s_miss_other = 0;
    uint64_t s_miss_gap_sum = 0;

    /* Bank-prep priority tier (RAMULATOR_SCHED_BANKPREP=1), see 2.2.0c. */
    bool   s_bankprep = false;
    bool   s_keep_open = false;
    size_t s_keepopen_skips = 0;
    size_t s_bankprep_act = 0, s_bankprep_pre = 0;
    int    m_cmd_act = -1, m_cmd_pre = -1, m_cmd_rd = -1, m_cmd_wr = -1;
    template <class V> static int bank_key(const V& v) {
      int k = 0;
      for (size_t L = 1; L + 2 < v.size(); L++) k = k * 16 + (v[L] < 0 ? 0 : v[L]);
      return k;
    }

    size_t s_ticks = 0;
    size_t s_ticks_empty = 0;      // nothing queued at all
    size_t s_ticks_issued = 0;     // a command went out
    size_t s_ticks_blocked = 0;    // work queued, but nothing was ready
    size_t s_cmd_rd = 0;           // issued command mix
    size_t s_cmd_wr = 0;
    size_t s_cmd_act = 0;
    size_t s_cmd_pre = 0;
    size_t s_cmd_other = 0;
    size_t s_blk_read_buf = 0;     // depth of read buffer while blocked
    size_t s_reqs_served = 0;

    /* PROBE: candidate retry.
     *
     * schedule_request() asks the scheduler for ONE candidate. If that
     * candidate then fails check_ready(), or trips the 2.3 test (it would
     * precharge a row another request in m_active_buffer is still using), the
     * whole cycle stalls -- no runner-up is tried. So a large read buffer does
     * not help when the head candidate is repeatedly rejected: the extra depth
     * is never consulted.
     *
     * RAMULATOR_RETRY=N re-runs the selection up to N times, excluding
     * already-rejected addresses, reusing the scheduler's own compare() so the
     * retry ranks by the SAME policy rather than a duplicated one.
     */
    int m_retry = 0;
    std::vector<Addr_t> s_skip;
    Addr_t s_last_rejected = 0;
    size_t s_stall_nocand = 0;     // buffers empty of eligible candidates
    size_t s_stall_notready = 0;   // candidate chosen, timing not met
    size_t s_stall_actconf = 0;    // candidate chosen, would interrupt an open row
    size_t s_retry_saves = 0;      // cycles rescued by a runner-up
    size_t s_actbuf_occ = 0;       // running sum -> mean occupancy
    size_t s_actbuf_full = 0;      // cycles at capacity

    /* Rank the buffer with the scheduler's own policy, skipping rejects. */
    ReqBuffer::iterator best_skipping(ReqBuffer& buffer) {
      for (auto& req : buffer)
        req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
      /* KEEPOPEN (RAMULATOR_SCHED_KEEPOPEN=1): never select a request that would
         precharge a bank still holding a queued request for its OPEN row.
         The scheduler's row-hit tier cannot prevent this, because it sits below
         the readiness test: a hit blocked by column timing fails check_ready(),
         so the competing PRE wins on readiness and the row-hit tier is never
         consulted. Measured effect of that gap: 750,188 precharges per run
         discard a row wanted back ~25 cycles later. This pass sits above
         readiness and needs the whole buffer, which compare() cannot see. */
      static std::vector<char> hit_bank(4096);
      bool guard = false;
      if (s_keep_open) {
        std::fill(hit_bank.begin(), hit_bank.end(), 0);
        for (auto& req : buffer)
          if (req.command == req.final_command) {
            hit_bank[bank_key(req.addr_vec) & 4095] = 1; guard = true;
          }
      }
      for (int pass = 0; pass < 2; pass++) {
        auto cand = buffer.end();
        for (auto it = buffer.begin(); it != buffer.end(); it++) {
          if (!s_skip.empty() &&
              std::find(s_skip.begin(), s_skip.end(), it->addr) != s_skip.end())
            continue;
          if (pass == 0 && guard && it->command != it->final_command &&
              m_dram->check_rowbuffer_open(it->final_command, it->addr_vec) &&
              hit_bank[bank_key(it->addr_vec) & 4095]) {
            s_keepopen_skips++;
            continue;                       /* would strand a queued hit */
          }
          cand = (cand == buffer.end()) ? it : m_scheduler->compare(cand, it);
        }
        if (cand != buffer.end() || !guard) return cand;   /* pass 1 = unguarded fallback */
      }
      return buffer.end();
    }


  public:
    void init() override {
      m_wr_low_watermark =  param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
      m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching to write mode.").default_val(0.8f);

      m_scheduler = create_child_ifce<IScheduler>();
      m_refresh = create_child_ifce<IRefreshManager>();    

      if (m_config["plugins"]) {
        YAML::Node plugin_configs = m_config["plugins"];
        for (YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end(); ++it) {
          m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
        }
      }
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = memory_system->get_ifce<IDRAM>();
      m_row_addr_idx = m_dram->m_levels("row");
      m_priority_buffer.max_size = 512*3 + 32;
      {
        static int ctrl_seq = 0;
        int myid = ctrl_seq++;
        const char* rt = std::getenv("RAMULATOR_ROWTRACE");
        if (rt && myid == 0) {          // sample one controller; ipoly balances channels
          s_rowtrace = fopen(rt, "w");
          if (s_rowtrace) fprintf(s_rowtrace, "clk,hits,misses,conflicts,wh,wm,wc\n");
        }
      }
      if (const char* po = std::getenv("RAMULATOR_PRE_OWNER"))
        s_pre_owner_policy = (po[0] == '1');
      if (const char* wl = std::getenv("RAMULATOR_WEIGHT_LIMIT"))
        s_weight_limit = (Addr_t)std::strtoull(wl, nullptr, 10) / 16;
      if (const char* bp = std::getenv("RAMULATOR_SCHED_BANKPREP"))
        s_bankprep = (bp[0] == '1');
      /* PROBE: widen the scheduling window (default ReqBuffer::max_size = 64). */
      if (const char* w = std::getenv("RAMULATOR_REQBUF")) {
        size_t sz = std::strtoul(w, nullptr, 10);
        if (sz) { m_read_buffer.max_size = sz; m_write_buffer.max_size = sz; }
      }
      /* PROBE: m_active_buffer holds requests mid-activation (ACT issued,
       * column command pending). Its default 64 caps how many rows can be
       * opening at once, and 2.3 refuses any request that would precharge a
       * row still listed here -- so its size couples to the stall rate. */
      if (const char* w = std::getenv("RAMULATOR_ACTBUF")) {
        size_t sz = std::strtoul(w, nullptr, 10);
        if (sz) m_active_buffer.max_size = sz;
      }
      if (const char* r = std::getenv("RAMULATOR_RETRY"))
        m_retry = (int)std::strtol(r, nullptr, 10);
    };

    bool send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      // Forward existing write requests to incoming read requests
      if (req.type_id == Request::Type::Read) {
        auto compare_addr = [req](const Request& wreq) {
          return wreq.addr == req.addr;
        };
        if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(), compare_addr) != m_write_buffer.end()) {
          // The request will depart at the next cycle
          req.depart = m_clk + 1;
          pending.push_back(req);
          return true;
        }
      }

      // Else, enqueue them to corresponding buffer based on request type id
      bool is_success = false;
      req.arrive = m_clk;
      if        (req.type_id == Request::Type::Read) {
        is_success = m_read_buffer.enqueue(req);
      } else if (req.type_id == Request::Type::Write) {
        is_success = m_write_buffer.enqueue(req);
      } else {
        throw std::runtime_error("Invalid request type!");
      }
      if (!is_success) {
        // We could not enqueue the request
        req.arrive = -1;
        return false;
      }

      return true;
    };

    bool priority_send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      bool is_success = false;
      is_success = m_priority_buffer.enqueue(req);
      return is_success;
    }

    void tick() override {
      m_clk++;

      // 1. Serve completed reads
      serve_completed_reads();

      m_refresh->tick();

      if (s_rowtrace && (m_clk % s_rowtrace_every) == 0)
        fprintf(s_rowtrace, "%ld,%zu,%zu,%zu,%zu,%zu,%zu\n", (long)m_clk,
                s_num_row_hits, s_num_row_misses, s_num_row_conflicts,
                s_hit_w, s_miss_w, s_conf_w);

      // 2. Try to find a request to serve.
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      bool request_found = schedule_request(req_it, buffer);

      // 3. Update all plugins
      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }

      // Tick accounting
      s_ticks++;
      if (request_found) {
        s_ticks_issued++;
      } else if (m_read_buffer.size() || m_write_buffer.size() ||
                 m_active_buffer.size() || m_priority_buffer.size()) {
        s_ticks_blocked++;
        s_blk_read_buf += m_read_buffer.size();
        s_actbuf_occ += m_active_buffer.size();
        if (m_active_buffer.size() >= m_active_buffer.max_size) s_actbuf_full++;
      } else {
        s_ticks_empty++;
      }

      // 4. Finally, issue the commands to serve the request
      if (request_found) {
        {
          /* bank identity = pseudochannel/bankgroup/bank levels of addr_vec */
          int bkey = 0;
          for (size_t L = 1; L + 2 < req_it->addr_vec.size(); L++)
            bkey = bkey * 16 + (req_it->addr_vec[L] < 0 ? 0 : req_it->addr_vec[L]);
          std::string cnm = std::string(m_dram->m_commands(req_it->command));
          if (cnm.rfind("PRE", 0) == 0) {
            s_pre_total++;
            s_last_pre[bkey] = PreRec{req_it->addr, m_clk};
            if (s_pre_owner_policy) s_pre_owner[bkey] = req_it->addr;
            /* CLOSE AUDIT: PREA is channel-wide (refresh) and carries no bank in
               addr_vec, so record it as a timestamp and compare against per-bank
               demand closes. A plain PRE closes exactly this bank. */
            if (cnm == "PREA") s_prea_clk = m_clk;
            else {
              auto orit = s_open_row.find(bkey);
              s_closed[bkey] = CloseRec{m_clk,
                                        orit == s_open_row.end() ? -1 : orit->second,
                                        (int)req_it->source_id};
            }
          } else if (cnm == "ACT" || cnm == "ACT-2") {
            if (s_weight_limit) s_row_owner_w[bkey] = ((Addr_t)req_it->addr < s_weight_limit);
            s_ever_opened[bkey] = true;
            if (req_it->addr_vec.size() >= 2)
              s_open_row[bkey] = req_it->addr_vec[req_it->addr_vec.size() - 2];
            if (s_pre_owner_policy) s_pre_owner.erase(bkey);
            auto it = s_last_pre.find(bkey);
            if (it != s_last_pre.end()) {
              s_pre_to_act_cycles += (m_clk - it->second.clk);
              if (it->second.addr == req_it->addr) s_pre_act_same++;
              else s_pre_act_other++;
              s_last_pre.erase(it);
            }
          }
        }
        {
          /* Prefix match so LPDDR's RD32/WR32/ACT-1/ACT-2 are counted too
             (ACT-2 is the row-opening half of the two-step activate). */
          std::string cn = std::string(m_dram->m_commands(req_it->command));
          if (cn.rfind("RD", 0) == 0) s_cmd_rd++;
          else if (cn.rfind("WR", 0) == 0) s_cmd_wr++;
          else if (cn == "ACT" || cn == "ACT-2") s_cmd_act++;
          else if (cn.rfind("PRE", 0) == 0) s_cmd_pre++;
          else s_cmd_other++;
        }
        if(req_it->is_first) {
          req_it->is_first = false;
          bool row_hit = m_dram->check_rowbuffer_hit(req_it->final_command, req_it->addr_vec);
          bool row_open = m_dram->check_rowbuffer_open(req_it->final_command, req_it->addr_vec);
          if (row_hit) {
            s_num_row_hits++;
          } else if (row_open) {
            s_num_row_conflicts++;
          } else {
            s_num_row_misses++;
          }
          if (s_weight_limit) {
            bool w = (Addr_t)req_it->addr < s_weight_limit;
            /* Operand-tagged split: KV = attention K/V MOVINs (operand 101/102
               above the weight limit); everything else above the limit is
               activation traffic (Q, GEMM inputs, outputs). Untagged requests
               (operand 0, e.g. writes) fall into the activation bucket. The
               address-only split lumped GEMM-phase activation re-reads into
               "kv" and understated KV row hit badly. */
            bool kv = !w && (req_it->source_id == 101 || req_it->source_id == 102);   /* operand rides in source_id */
            bool a  = !w && !kv;
            if (row_hit)        { w ? s_hit_w++  : (kv ? s_hit_k++  : s_hit_a++); }
            else if (row_open)  {
              w ? s_conf_w++ : (kv ? s_conf_k++ : s_conf_a++);
              int bk = 0;
              for (size_t L = 1; L + 2 < req_it->addr_vec.size(); L++)
                bk = bk * 16 + (req_it->addr_vec[L] < 0 ? 0 : req_it->addr_vec[L]);
              auto ow = s_row_owner_w.find(bk);
              bool owner_w = (ow != s_row_owner_w.end()) && ow->second;
              if (w) { owner_w ? s_conf_w_by_w++ : s_conf_w_by_k++; }
              else   { owner_w ? s_conf_k_by_w++ : s_conf_k_by_k++; }
            }
            else {
              w ? s_miss_w++ : (kv ? s_miss_k++ : s_miss_a++);
              if (kv) {   /* CLOSE AUDIT on the KV stream only */
                int bk = bank_key(req_it->addr_vec);
                int want = req_it->addr_vec.size() >= 2
                             ? req_it->addr_vec[req_it->addr_vec.size() - 2] : -1;
                auto ci = s_closed.find(bk);
                if (!s_ever_opened.count(bk))                         s_miss_first++;
                else if (ci == s_closed.end() || s_prea_clk > ci->second.clk)
                                                                      s_miss_by_refresh++;
                else {
                  s_miss_by_pre++;
                  s_miss_gap_sum += (uint64_t)(m_clk - ci->second.clk);
                  if (ci->second.row == want) s_miss_samerow++;
                  if (ci->second.operand == (int)req_it->source_id) s_miss_self++;
                  else                                             s_miss_other++;
                }
              }
            }
          }
        }
        // If we find a real request to serve
        m_dram->issue_command(req_it->command, req_it->addr_vec);

        // If we are issuing the last command, set depart clock cycle and move the request to the pending queue
        if (req_it->command == req_it->final_command) {
          if (req_it->type_id == Request::Type::Read) {
            s_reqs_served++;
            req_it->depart = m_clk + m_dram->m_read_latency;
            pending.push_back(*req_it);
          } else if (req_it->type_id == Request::Type::Write) {
            // TODO: Add code to update statistics
            if(req_it->callback)
              req_it->callback(*req_it);
          }
          buffer->remove(req_it);
        } else {
          if (m_dram->m_command_meta(req_it->command).is_opening) {
            m_active_buffer.enqueue(*req_it);
            buffer->remove(req_it);
          }
        }

      }

    };

    void finalize() override {
      spdlog::info("Row hits: {}, Row misses: {}, Row conflicts: {}", s_num_row_hits, s_num_row_misses, s_num_row_conflicts);
      spdlog::info("READ LATENCY: n {} mean {:.1f} min {} controller cycles | ticks {} issued {} rd {} act {}",
                   s_lat_n, s_lat_n ? (double)s_lat_sum / s_lat_n : 0.0, s_lat_min, s_ticks, s_ticks_issued, s_cmd_rd, s_cmd_act);
      if (s_weight_limit) {
        size_t tw = s_hit_w + s_miss_w + s_conf_w;
        size_t tk = s_hit_k + s_miss_k + s_conf_k;
        spdlog::info("ROWSPLIT weights  acc {} hit {:.1f}% miss {:.1f}% confl {:.1f}%",
                     tw, tw ? 100.0*s_hit_w/tw : 0.0, tw ? 100.0*s_miss_w/tw : 0.0,
                     tw ? 100.0*s_conf_w/tw : 0.0);
        spdlog::info("ROWSPLIT kv+act   acc {} hit {:.1f}% miss {:.1f}% confl {:.1f}%",
                     tk, tk ? 100.0*s_hit_k/tk : 0.0, tk ? 100.0*s_miss_k/tk : 0.0,
                     tk ? 100.0*s_conf_k/tk : 0.0);
        {
          size_t ta = s_hit_a + s_miss_a + s_conf_a;
          spdlog::info("ROWSPLIT act      acc {} hit {:.1f}% miss {:.1f}% confl {:.1f}%   (kv+act line above now = KV only: operand 101/102)",
                       ta, ta ? 100.0*s_hit_a/ta : 0.0, ta ? 100.0*s_miss_a/ta : 0.0, ta ? 100.0*s_conf_a/ta : 0.0);
        }
        {
          size_t tot = s_miss_first + s_miss_by_pre + s_miss_by_refresh;
          if (tot) spdlog::info(
            "CLOSEAUDIT KV misses {} | first-touch {} ({:.1f}%) | closed by demand PRE {} ({:.1f}%) "
            "| closed by refresh PREA {} ({:.1f}%) || of the demand closes: same-row-came-back {} "
            "({:.1f}%) self-operand {} other-operand {} mean close->miss gap {:.0f} cyc",
            tot, s_miss_first, 100.0*s_miss_first/tot,
            s_miss_by_pre, 100.0*s_miss_by_pre/tot,
            s_miss_by_refresh, 100.0*s_miss_by_refresh/tot,
            s_miss_samerow, s_miss_by_pre ? 100.0*s_miss_samerow/s_miss_by_pre : 0.0,
            s_miss_self, s_miss_other,
            s_miss_by_pre ? (double)s_miss_gap_sum/s_miss_by_pre : 0.0);
        }
        spdlog::info("CONFLICT EVICTOR: kv-by-weight {} kv-by-kv {} | weight-by-weight {} weight-by-kv {}",
                     s_conf_k_by_w, s_conf_k_by_k, s_conf_w_by_w, s_conf_w_by_k);
        if (s_bankprep)
          spdlog::info("BANKPREP: early ACT {} early PRE {}", s_bankprep_act, s_bankprep_pre);
        if (s_keep_open)
          spdlog::info("KEEPOPEN: precharges suppressed {}", s_keepopen_skips);
      }
      if (s_pre_total) {
        size_t matched = s_pre_act_same + s_pre_act_other;
        spdlog::info(
            "PREAUDIT PRE {} | followed by ACT from SAME request {} ({:.1f}%) | "
            "from OTHER request {} ({:.1f}%) | never followed {} ({:.1f}%) | "
            "mean PRE->ACT gap {:.1f} cycles",
            s_pre_total, s_pre_act_same, 100.0 * s_pre_act_same / s_pre_total,
            s_pre_act_other, 100.0 * s_pre_act_other / s_pre_total,
            s_pre_total - matched, 100.0 * (s_pre_total - matched) / s_pre_total,
            matched ? (double)s_pre_to_act_cycles / matched : 0.0);
        if (s_pre_owner_policy)
          spdlog::info("PREAUDIT owner-priority fired {} times ({:.1f}% of PREs)",
                       s_owner_hits, 100.0 * s_owner_hits / s_pre_total);
      }
      auto pc = [&](size_t v) { return s_ticks ? (double)v * 100.0 / s_ticks : 0.0; };
      if (s_ticks_blocked) {
        spdlog::info(
            "STALLWHY blocked {} | no candidate {} ({:.1f}%) | not ready {} ({:.1f}%) "
            "| act-row conflict {} ({:.1f}%) | retry saves {} | actbuf mean {:.1f}/{} "
            "full {:.2f}% of blocked",
            s_ticks_blocked,
            s_stall_nocand, 100.0 * s_stall_nocand / s_ticks_blocked,
            s_stall_notready, 100.0 * s_stall_notready / s_ticks_blocked,
            s_stall_actconf, 100.0 * s_stall_actconf / s_ticks_blocked,
            s_retry_saves,
            (double)s_actbuf_occ / s_ticks_blocked, m_active_buffer.max_size,
            100.0 * s_actbuf_full / s_ticks_blocked);
      }
      spdlog::info(
          "CTRL ticks {} | issued {} ({:.1f}%) | BLOCKED(work queued, none ready) {} "
          "({:.1f}%) | empty {} ({:.1f}%)",
          s_ticks, s_ticks_issued, pc(s_ticks_issued), s_ticks_blocked,
          pc(s_ticks_blocked), s_ticks_empty, pc(s_ticks_empty));
      spdlog::info(
          "CTRL cmd mix: RD {} WR {} ACT {} PRE {} other {} | reads served {} | "
          "commands per read {:.2f} | mean read-buf depth while blocked {:.1f}",
          s_cmd_rd, s_cmd_wr, s_cmd_act, s_cmd_pre, s_cmd_other, s_reqs_served,
          s_reqs_served ? (double)(s_cmd_rd + s_cmd_wr + s_cmd_act + s_cmd_pre +
                                   s_cmd_other) / s_reqs_served : 0.0,
          s_ticks_blocked ? (double)s_blk_read_buf / s_ticks_blocked : 0.0);
    }
  private:
    /**
     * @brief    Helper function to serve the completed read requests
     * @details
     * This function is called at the beginning of the tick() function.
     * It checks the pending queue to see if the top request has received data from DRAM.
     * If so, it finishes this request by calling its callback and poping it from the pending queue.
     */
    void serve_completed_reads() {
      if (pending.size()) {
        // Check the first pending request
        auto& req = pending[0];
        if (req.depart <= m_clk) {
          // Request received data from dram
          if (req.depart - req.arrive > 1) {
            // Check if this requests accesses the DRAM or is being forwarded.
            /* Read latency (controller arrive -> data return), for the
               unloaded-latency validation gate and the HBM3/LPDDR6 comparison. */
            s_lat_sum += (req.depart - req.arrive); s_lat_n++;
            if (s_lat_n == 1 || (req.depart - req.arrive) < s_lat_min) s_lat_min = req.depart - req.arrive;
          }

          if (req.callback) {
            // If the request comes from outside (e.g., processor), call its callback
            req.callback(req);
          }
          // Finally, remove this request from the pending queue
          pending.pop_front();
        }
      };
    };


    /**
     * @brief    Checks if we need to switch to write mode
     * 
     */
    void set_write_mode() {
      if (!m_is_write_mode) {
        if ((m_write_buffer.size() > m_wr_high_watermark * m_write_buffer.max_size) || m_read_buffer.size() == 0) {
          m_is_write_mode = true;
        }
      } else {
        if ((m_write_buffer.size() < m_wr_low_watermark * m_write_buffer.max_size) && m_read_buffer.size() != 0) {
          m_is_write_mode = false;
        }
      }
    };


    /**
     * @brief    Helper function to find a request to schedule from the buffers.
     * 
     */
    bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
      s_skip.clear();
      for (int attempt = 0; ; attempt++) {
        s_last_rejected = 0;
        if (schedule_request_inner(req_it, req_buffer)) {
          if (attempt) s_retry_saves++;
          return true;
        }
        // Stop when the retry budget is spent, or when the failure was not a
        // rejected candidate (nothing to exclude -> the retry would be identical).
        if (attempt >= m_retry || s_last_rejected == 0) return false;
        s_skip.push_back(s_last_rejected);
      }
    }

    bool schedule_request_inner(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
      bool request_found = false;
      // 2.1    First, check the act buffer to serve requests that are already activating (avoid useless ACTs)
      if (req_it= best_skipping(m_active_buffer); req_it != m_active_buffer.end()) {
        if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
          request_found = true;
          req_buffer = &m_active_buffer;
        } else {
          s_last_rejected = req_it->addr;
        }
      }

      // 2.2    If no requests can be scheduled from the act buffer, check the rest of the buffers
      if (!request_found) {
        // 2.2.1    We first check the priority buffer to prioritize e.g., maintenance requests
        if (m_priority_buffer.size() != 0) {
          req_buffer = &m_priority_buffer;
          req_it = m_priority_buffer.begin();
          req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);
          
          request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
          if ((!request_found) & (m_priority_buffer.size() != 0)) {
            return false;
          }
        }

        // 2.2.0b   Precharge-owner priority: hand a just-precharged bank back
        //          to the request that closed it, if that request can go now.
        if (!request_found && s_pre_owner_policy && !s_pre_owner.empty()) {
          set_write_mode();
          auto& buf = m_is_write_mode ? m_write_buffer : m_read_buffer;
          for (auto it = buf.begin(); it != buf.end(); it++) {
            int bk = 0;
            for (size_t L = 1; L + 2 < it->addr_vec.size(); L++)
              bk = bk * 16 + (it->addr_vec[L] < 0 ? 0 : it->addr_vec[L]);
            auto own = s_pre_owner.find(bk);
            if (own == s_pre_owner.end() || own->second != it->addr) continue;
            it->command = m_dram->get_preq_command(it->final_command, it->addr_vec);
            if (m_dram->check_ready(it->command, it->addr_vec)) {
              req_it = it; req_buffer = &buf; request_found = true;
              s_owner_hits++;
              break;
            }
          }
        }
        // 2.2.0c   Bank-prep priority (RAMULATOR_SCHED_BANKPREP=1). ACT and PRE
        //          occupy the command bus only, never the data bus, so serving
        //          them ahead of OLDER row hits in other banks costs no
        //          bandwidth and shortens every row switch (measured PRE->ACT
        //          gap 62 cycles vs tRP 23: the ACT was waiting its FCFS turn
        //          behind hits). Under a bank-isolated KV layout every row
        //          switch is a same-bank switch, so this is the whole cost.
        //            ACT  bank closed and a request waits: open it now.
        //            PRE  bank open on a row with no ready hit left in the
        //                 window (drained) and another row is waiting.
        //          Guards: never PRE a row that still has a ready hit queued;
        //          never touch a bank with a request mid-activation.
        if (!request_found && s_bankprep) {
          if (m_cmd_act < 0) {
            m_cmd_act = m_dram->m_commands("ACT"); m_cmd_pre = m_dram->m_commands("PRE");
            m_cmd_rd  = m_dram->m_commands("RD");  m_cmd_wr  = m_dram->m_commands("WR");
          }
          set_write_mode();
          auto& buf = m_is_write_mode ? m_write_buffer : m_read_buffer;
          /* A preq of RD/WR already means "bank open on this row" (otherwise
             it would be ACT or PRE), so no row-buffer probe is needed.
             Bank keys are < 4096 for a 3-level bank hierarchy. */
          static std::vector<char> hit_banks(4096), act_banks(4096);
          std::fill(hit_banks.begin(), hit_banks.end(), 0);
          std::fill(act_banks.begin(), act_banks.end(), 0);
          bool any_prep = false;
          for (auto& r : buf) {
            r.command = m_dram->get_preq_command(r.final_command, r.addr_vec);
            if (r.command == m_cmd_rd || r.command == m_cmd_wr) hit_banks[bank_key(r.addr_vec) & 4095] = 1;
            else if (r.command == m_cmd_act || r.command == m_cmd_pre) any_prep = true;
          }
          auto best = buf.end();
          if (any_prep) {
            for (auto& r : m_active_buffer) act_banks[bank_key(r.addr_vec) & 4095] = 1;
            for (auto it = buf.begin(); it != buf.end(); it++) {
              const bool is_act = (it->command == m_cmd_act), is_pre = (it->command == m_cmd_pre);
              if (!is_act && !is_pre) continue;
              const int bk = bank_key(it->addr_vec) & 4095;
              if (act_banks[bk]) continue;
              if (is_pre && hit_banks[bk]) continue;
              if (best != buf.end() && it->arrive >= best->arrive) continue;
              if (!s_skip.empty() && std::find(s_skip.begin(), s_skip.end(), it->addr) != s_skip.end()) continue;
              if (!m_dram->check_ready(it->command, it->addr_vec)) continue;
              best = it;
            }
          }
          if (best != buf.end()) {
            req_it = best; req_buffer = &buf; request_found = true;
            (best->command == m_cmd_act) ? s_bankprep_act++ : s_bankprep_pre++;
          }
        }
        // 2.2.1    If no request to be scheduled in the priority buffer, check the read and write buffers.
        if (!request_found) {
          // Query the write policy to decide which buffer to serve
          set_write_mode();
          auto& buffer = m_is_write_mode ? m_write_buffer : m_read_buffer;
          if (req_it = best_skipping(buffer); req_it != buffer.end()) {
            request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
            req_buffer = &buffer;
            if (!request_found) { s_last_rejected = req_it->addr; s_stall_notready++; }
          } else {
            s_stall_nocand++;
          }
        }
      }

      // 2.3 If we find a request to schedule, we need to check if it will close an opened row in the active buffer.
      if (request_found) {
        if (m_dram->m_command_meta(req_it->command).is_closing) {
          bool has_addr_wildcard = false;
          int row_group_end = m_row_addr_idx;
          int last_valid_level = 0;
          for (int i : req_it->addr_vec) {
            if (i == -1) {
              has_addr_wildcard = true;
              row_group_end = last_valid_level;
              break;
            }
            last_valid_level++;
          }

          std::vector<Addr_t> rowgroup((req_it->addr_vec).begin(), (req_it->addr_vec).begin() + row_group_end);
          for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end(); _it++) {
            std::vector<Addr_t> _it_rowgroup(_it->addr_vec.begin(), _it->addr_vec.begin() + row_group_end);
            if (rowgroup == _it_rowgroup) {
              // Invalidate this scheduling outcome if we are to interrupt a request in the active buffer
              request_found = false;
              s_last_rejected = req_it->addr;
              s_stall_actconf++;
            }
          }
        }
      }

      return request_found;
    }

};
  
}   // namespace Ramulator