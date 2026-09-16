#include "dram/dram.h"
#include "dram/lambdas.h"

namespace Ramulator {

class LPDDR6 : public IDRAM, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAM, LPDDR6, "LPDDR6", "LPDDR6 (JESD209-6) sub-channel device model, derived from LPDDR5X")

  public:
    inline static const std::map<std::string, Organization> org_presets = {
      /* One Ramulator channel = one LPDDR6 SUB-channel (12 DQ). JESD209-6: a
         24-bit channel is two independent 12-bit sub-channels, each with its
         own CA. Bank organisation per sub-channel is ASSUMED equal to LPDDR5X
         (4 bank groups x 4 banks); JEDEC has not published it. Density is
         per sub-channel and must satisfy bg*bank*row*col*dq. */
      //   name              density   DQ   Ch Ra Bg Ba   Ro     Co
      {"LPDDR6_6Gb_x12",   {6<<10,   12, {1, 1, 4, 4, 1<<15, 1<<10}}},
      {"LPDDR6_12Gb_x12",  {12<<10,  12, {1, 1, 4, 4, 1<<15, 1<<11}}},
    };

    inline static const std::map<std::string, std::vector<int>> timing_presets = {
      /* No JEDEC-published LPDDR6 core timings exist in public sources; the
         yaml must supply every timing (ns via tXXX or cycles via nXXX) at a
         given `rate`. See configs/ramulator2_configs/_lpddr6_*.yaml. */
    };


  /************************************************
   *                Organization
   ***********************************************/   
    const int m_internal_prefetch_size = 24;   /* BL24 x 12 DQ = 36 B burst: 32 B data + 4 B meta */

    inline static constexpr ImplDef m_levels = {
      "channel", "rank", "bankgroup", "bank", "row", "column",    
    };


  /************************************************
   *             Requests & Commands
   ***********************************************/
    inline static constexpr ImplDef m_commands = {
      "ACT-1",  "ACT-2",
      "PRE",    "PREA",
      "CASRD",  "CASWR",   // WCK2CK Sync
      "RD32",   "WR32",   "RD32A",   "WR32A",
      "REFab",  "REFpb",
      "RFMab",  "RFMpb",
    };

    inline static const ImplLUT m_command_scopes = LUT (
      m_commands, m_levels, {
        {"ACT-1", "row"},    {"ACT-2",  "row"},
        {"PRE",   "bank"},   {"PREA",   "rank"},
        {"CASRD", "rank"},   {"CASWR",  "rank"},
        {"RD32",  "column"}, {"WR32",   "column"}, {"RD32A", "column"}, {"WR32A", "column"},
        {"REFab", "rank"},   {"REFpb",  "rank"},
        {"RFMab", "rank"},   {"RFMpb",  "rank"},
      }
    );

    inline static const ImplLUT m_command_meta = LUT<DRAMCommandMeta> (
      m_commands, {
                // open?   close?   access?  refresh?
        {"ACT-1",  {false,  false,   false,   false}},
        {"ACT-2",  {true,   false,   false,   false}},
        {"PRE",    {false,  true,    false,   false}},
        {"PREA",   {false,  true,    false,   false}},
        {"CASRD",  {false,  false,   false,   false}},
        {"CASWR",  {false,  false,   false,   false}},
        {"RD32",   {false,  false,   true,    false}},
        {"WR32",   {false,  false,   true,    false}},
        {"RD32A",  {false,  true,    true,    false}},
        {"WR32A",  {false,  true,    true,    false}},
        {"REFab",  {false,  false,   false,   true }},
        {"REFpb",  {false,  false,   false,   true }},
        {"RFMab",  {false,  false,   false,   true }},
        {"RFMpb",  {false,  false,   false,   true }},
      }
    );

    inline static constexpr ImplDef m_requests = {
      "read16", "write16",
      "all-bank-refresh", "per-bank-refresh"
    };

    inline static const ImplLUT m_request_translations = LUT (
      m_requests, m_commands, {
        {"read16", "RD32"}, {"write16", "WR32"}, 
        {"all-bank-refresh", "REFab"}, {"per-bank-refresh", "REFpb"},
      }
    );

   
  /************************************************
   *                   Timing
   ***********************************************/
    inline static constexpr ImplDef m_timings = {
      "rate", 
      "nBL32",
      "nCL", "nRCDW", "nRCDR",
      "nRPab", "nRPpb", "nRAS", "nRC", "nWR", "nRTP", "nCWL",
      "nCCD",
      "nRRD",
      "nWTRS", "nWTRL",
      "nFAW",
      "nPPD",
      "nRFCab", "nRFCpb","nREFI",
      "nPBR2PBR", "nPBR2ACT",
      "nCS",
      "tCK_ps"
    };


  /************************************************
   *                 Node States
   ***********************************************/
    inline static constexpr ImplDef m_states = {
    //    ACT-1       ACT-2
       "Pre-Opened", "Opened", "Closed", "PowerUp", "N/A"
    };

    inline static const ImplLUT m_init_states = LUT (
      m_levels, m_states, {
        {"channel",   "N/A"}, 
        {"rank",      "PowerUp"},
        {"bankgroup", "N/A"},
        {"bank",      "Closed"},
        {"row",       "Closed"},
        {"column",    "N/A"},
      }
    );

  public:
    struct Node : public DRAMNodeBase<LPDDR6> {
      Clk_t m_final_synced_cycle = -1; // Extra CAS Sync command needed for RD/WR after this cycle

      Node(LPDDR6* dram, Node* parent, int level, int id) : DRAMNodeBase<LPDDR6>(dram, parent, level, id) {};
    };
    std::vector<Node*> m_channels;
    
    FuncMatrix<ActionFunc_t<Node>>  m_actions;
    FuncMatrix<PreqFunc_t<Node>>    m_preqs;
    FuncMatrix<RowhitFunc_t<Node>>  m_rowhits;
    FuncMatrix<RowopenFunc_t<Node>> m_rowopens;


  public:
    void tick() override {
      m_clk++;
    };

    void init() override {
      RAMULATOR_DECLARE_SPECS();
      set_organization();
      set_timing_vals();

      set_actions();
      set_preqs();
      set_rowhits();
      set_rowopens();
      
      create_nodes();
    };

    void issue_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      m_channels[channel_id]->update_timing(command, addr_vec, m_clk);
      m_channels[channel_id]->update_states(command, addr_vec, m_clk);
    };

    int get_preq_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->get_preq_command(command, addr_vec, m_clk);
    };

    bool check_ready(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_ready(command, addr_vec, m_clk);
    };

    bool check_rowbuffer_hit(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_rowbuffer_hit(command, addr_vec, m_clk);
    };

    bool check_rowbuffer_open(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_rowbuffer_open(command, addr_vec, m_clk);
    };

  private:
    void set_organization() {
      // Channel width
      m_channel_width = param_group("org").param<int>("channel_width").default_val(12);

      // Organization
      m_organization.count.resize(m_levels.size(), -1);

      // Load organization preset if provided
      if (auto preset_name = param_group("org").param<std::string>("preset").optional()) {
        if (org_presets.count(*preset_name) > 0) {
          m_organization = org_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized organization preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      // Override the preset with any provided settings
      if (auto dq = param_group("org").param<int>("dq").optional()) {
        m_organization.dq = *dq;
      }

      for (int i = 0; i < m_levels.size(); i++){
        auto level_name = m_levels(i);
        if (auto sz = param_group("org").param<int>(level_name).optional()) {
          m_organization.count[i] = *sz;
        }
      }

      if (auto density = param_group("org").param<int>("density").optional()) {
        m_organization.density = *density;
      }

      // Sanity check: is the calculated chip density the same as the provided one?
      size_t _density = size_t(m_organization.count[m_levels["bankgroup"]]) *
                        size_t(m_organization.count[m_levels["bank"]]) *
                        size_t(m_organization.count[m_levels["row"]]) *
                        size_t(m_organization.count[m_levels["column"]]) *
                        size_t(m_organization.dq);
      _density >>= 20;
      if (m_organization.density != _density) {
        throw ConfigurationError(
            "Calculated {} chip density {} Mb does not equal the provided density {} Mb!", 
            get_name(),
            _density, 
            m_organization.density
        );
      }

    };

    void set_timing_vals() {
      m_timing_vals.resize(m_timings.size(), -1);

      // Load timing preset if provided
      bool preset_provided = false;
      if (auto preset_name = param_group("timing").param<std::string>("preset").optional()) {
        if (timing_presets.count(*preset_name) > 0) {
          m_timing_vals = timing_presets.at(*preset_name);
          preset_provided = true;
        } else {
          throw ConfigurationError("Unrecognized timing preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      // Check for rate (in MT/s), and if provided, calculate and set tCK (in picosecond)
      if (auto dq = param_group("timing").param<int>("rate").optional()) {
        if (preset_provided) {
          throw ConfigurationError("Cannot change the transfer rate of {} when using a speed preset !", get_name());
        }
        m_timing_vals("rate") = *dq;
      }
      int tCK_ps = 1E6 / (m_timing_vals("rate") / 2);
      m_timing_vals("tCK_ps") = tCK_ps;

      // Load the organization specific timings
      int dq_id = [](int dq) -> int {
        switch (dq) {
          case 12: return 0;
          case 16: return 0;
          default: return -1;
        }
      }(m_organization.dq);

      int rate_id = [](int rate) -> int {
        switch (rate) {
          case 6400:  return 0;
          default:    return -1;
        }
      }(m_timing_vals("rate"));


      // Refresh timings
      // tRFC table (unit is nanosecond!)
      constexpr int tRFCab_TABLE[4] = {
      //  2Gb   4Gb   8Gb  16Gb
          130,  180,  210,  280, 
      };

      constexpr int tRFCpb_TABLE[4] = {
      //  2Gb   4Gb   8Gb  16Gb
          60,   90,   120,  140, 
      };

      constexpr int tPBR2PBR_TABLE[4] = {
      //  2Gb   4Gb   8Gb  16Gb
          60,   90,   90,  90, 
      };

      constexpr int tPBR2ACT_TABLE[4] = {
      //  2Gb   4Gb   8Gb  16Gb
          8,    8,    8,   8, 
      };

      // tREFI(base) table (unit is nanosecond!)
      constexpr int tREFI_BASE = 3906;
      int density_id = [](int density_Mb) -> int { 
        switch (density_Mb) {
          case 2048:  return 0;
          case 4096:  return 1;
          case 8192:  return 2;
          case 16384: return 3;
          case 6144:  return 2;   /* 6 Gb sub-channel: use the 8 Gb refresh row */
          case 12288: return 3;   /* 12 Gb sub-channel: use the 16 Gb refresh row */
          default:    return -1;
        }
      }(m_organization.density);

      m_timing_vals("nRFCab")    = JEDEC_rounding(tRFCab_TABLE[density_id], tCK_ps);
      m_timing_vals("nRFCpb")    = JEDEC_rounding(tRFCpb_TABLE[density_id], tCK_ps);
      m_timing_vals("nPBR2PBR")  = JEDEC_rounding(tPBR2PBR_TABLE[density_id], tCK_ps);
      m_timing_vals("nPBR2ACT")  = JEDEC_rounding(tPBR2ACT_TABLE[density_id], tCK_ps);
      m_timing_vals("nREFI") = JEDEC_rounding(tREFI_BASE, tCK_ps);

      // Overwrite timing parameters with any user-provided value
      // Rate and tCK should not be overwritten
      for (int i = 1; i < m_timings.size() - 1; i++) {
        auto timing_name = std::string(m_timings(i));

        if (auto provided_timing = param_group("timing").param<int>(timing_name).optional()) {
          // Check if the user specifies in the number of cycles (e.g., nRCD)
          m_timing_vals(i) = *provided_timing;
        } else if (auto provided_timing = param_group("timing").param<float>(timing_name.replace(0, 1, "t")).optional()) {
          // Check if the user specifies in nanoseconds (e.g., tRCD)
          m_timing_vals(i) = JEDEC_rounding(*provided_timing, tCK_ps);
        }
      }

      // Check if there is any uninitialized timings
      for (int i = 0; i < m_timing_vals.size(); i++) {
        if (m_timing_vals(i) == -1) {
          throw ConfigurationError("In \"{}\", timing {} is not specified!", get_name(), m_timings(i));
        }
      }      

      // Set read latency
      m_read_latency = m_timing_vals("nCL") + m_timing_vals("nBL32") * 3;

      // Populate the timing constraints
      #define V(timing) (m_timing_vals(timing))
      populate_timingcons(this, {
          /*** Channel ***/ 
          // CAS <-> CAS
          /// Data bus occupancy
          {.level = "channel", .preceding = {"RD32", "RD32A"}, .following = {"RD32", "RD32A"}, .latency = V("nBL32")},
          {.level = "channel", .preceding = {"WR32", "WR32A"}, .following = {"WR32", "WR32A"}, .latency = V("nBL32")},
          // Interleaving burst
          {.level = "channel", .preceding = {"RD32", "RD32A"}, .following = {"RD32", "RD32A"}, .latency = V("nBL32") * 3, .blocked_offset=V("nBL32") * 2},
          {.level = "channel", .preceding = {"WR32", "WR32A"}, .following = {"WR32", "WR32A"}, .latency = V("nBL32") * 3, .blocked_offset=V("nBL32") * 2},
          {.level = "channel", .preceding = {"RD32", "RD32A"}, .following = {"RD32", "RD32A"}, .latency = V("nBL32") * 4, .window=2},
          {.level = "channel", .preceding = {"WR32", "WR32A"}, .following = {"WR32", "WR32A"}, .latency = V("nBL32") * 4, .window=2},
          /*** Rank (or different BankGroup) ***/ 
          // CAS <-> CAS
          {.level = "rank", .preceding = {"RD32", "RD32A"}, .following = {"RD32", "RD32A"}, .latency = V("nCCD")},
          {.level = "rank", .preceding = {"WR32", "WR32A"}, .following = {"WR32", "WR32A"}, .latency = V("nCCD")},
          /// RD <-> WR, Minimum Read to Write, Assuming tWPRE = 1 tCK                          
          {.level = "rank", .preceding = {"RD32", "RD32A"}, .following = {"WR32", "WR32A"}, .latency = V("nCL") + V("nBL32") + 2 - V("nCWL")},
          /// WR <-> RD, Minimum Read after Write
          {.level = "rank", .preceding = {"WR32", "WR32A"}, .following = {"RD32", "RD32A"}, .latency = V("nCWL") + V("nBL32") + V("nWTRS")},
          /// CAS <-> CAS between sibling ranks, nCS (rank switching) is needed for new DQS
          {.level = "rank", .preceding = {"RD32", "RD32A"}, .following = {"RD32", "RD32A", "WR32", "WR32A"}, .latency = V("nBL32") + V("nCS"), .is_sibling = true},
          {.level = "rank", .preceding = {"WR32", "WR32A"}, .following = {"RD32", "RD32A"}, .latency = V("nCL")  + V("nBL32") + V("nCS") - V("nCWL"), .is_sibling = true},
          /// CAS <-> PREab
          {.level = "rank", .preceding = {"RD32"}, .following = {"PREA"}, .latency = V("nRTP")},
          {.level = "rank", .preceding = {"WR32"}, .following = {"PREA"}, .latency = V("nCWL") + V("nBL32") + V("nWR")},          
          /// RAS <-> RAS
          {.level = "rank", .preceding = {"ACT-1"}, .following = {"ACT-1", "REFpb"}, .latency = V("nRRD")},          
          {.level = "rank", .preceding = {"ACT-1"}, .following = {"ACT-1"}, .latency = V("nFAW"), .window = 4},          
          {.level = "rank", .preceding = {"ACT-1"}, .following = {"PREA"}, .latency = V("nRAS")},          
          {.level = "rank", .preceding = {"PREA"}, .following = {"ACT-1"}, .latency = V("nRPab")},          
          /// RAS <-> REF
          {.level = "rank", .preceding = {"ACT-1"}, .following = {"REFab"}, .latency = V("nRC")},          
          {.level = "rank", .preceding = {"PRE"}, .following = {"REFab"}, .latency = V("nRPpb")},          
          {.level = "rank", .preceding = {"PREA"}, .following = {"REFab"}, .latency = V("nRPab")},          
          {.level = "rank", .preceding = {"RD32A"}, .following = {"REFab"}, .latency = V("nRPpb") + V("nRTP")},          
          {.level = "rank", .preceding = {"WR32A"}, .following = {"REFab"}, .latency = V("nCWL") + V("nBL32") + V("nWR") + V("nRPpb")},          
          {.level = "rank", .preceding = {"REFab"}, .following = {"REFab", "ACT-1", "REFpb"}, .latency = V("nRFCab")},          
          {.level = "rank", .preceding = {"REFpb"},   .following = {"ACT-1"}, .latency = V("nPBR2ACT")},  
          {.level = "rank", .preceding = {"REFpb"}, .following = {"REFpb"}, .latency = V("nPBR2PBR")},  

          /*** Same Bank Group ***/ 
          /// CAS <-> CAS
          {.level = "bankgroup", .preceding = {"RD32", "RD32A"}, .following = {"RD32", "RD32A"}, .latency = 4 * V("nCCD")},          
          {.level = "bankgroup", .preceding = {"WR32", "WR32A"}, .following = {"WR32", "WR32A"}, .latency = 4 * V("nCCD")},          
          {.level = "bankgroup", .preceding = {"WR32", "WR32A"}, .following = {"RD32", "RD32A"}, .latency = V("nCWL") + V("nBL32") + V("nWTRL")},
          /// RAS <-> RAS
          {.level = "bankgroup", .preceding = {"ACT-1"}, .following = {"ACT-1"}, .latency = V("nRRD")},  

          /*** Bank ***/ 
          {.level = "bank", .preceding = {"ACT-1"}, .following = {"ACT-1"}, .latency = V("nRC")},  
          {.level = "bank", .preceding = {"ACT-1"}, .following = {"RD32", "RD32A"}, .latency = V("nRCDR")},  
          {.level = "bank", .preceding = {"ACT-1"}, .following = {"WR32", "WR32A"}, .latency = V("nRCDW")},
          {.level = "bank", .preceding = {"ACT-1"}, .following = {"PRE"}, .latency = V("nRAS")},  
          {.level = "bank", .preceding = {"PRE"}, .following = {"ACT-1"}, .latency = V("nRPpb")},  
          {.level = "bank", .preceding = {"RD32"},  .following = {"PRE"}, .latency = V("nRTP")},  
          {.level = "bank", .preceding = {"WR32"},  .following = {"PRE"}, .latency = V("nCWL") + V("nBL32") + V("nWR")},  
          {.level = "bank", .preceding = {"RD32A"}, .following = {"ACT-1"}, .latency = V("nRTP") + V("nRPpb")},  
          {.level = "bank", .preceding = {"WR32A"}, .following = {"ACT-1"}, .latency = V("nCWL") + V("nBL32") + V("nWR") + V("nRPpb")},  
        }
      );
      #undef V

    };

    void set_actions() {
      m_actions.resize(m_levels.size(), std::vector<ActionFunc_t<Node>>(m_commands.size()));

      // Rank Actions
      m_actions[m_levels["rank"]][m_commands["PREA"]] = Lambdas::Action::Rank::PREab<LPDDR6>;
      m_actions[m_levels["rank"]][m_commands["CASRD"]] = [this] (Node* node, int cmd, int target_id, Clk_t clk) {
        node->m_final_synced_cycle = clk + m_timing_vals("nCL") + m_timing_vals("nBL32") + 1; 
      };
      m_actions[m_levels["rank"]][m_commands["CASWR"]] = [this] (Node* node, int cmd, int target_id, Clk_t clk) {
        node->m_final_synced_cycle = clk + m_timing_vals("nCWL") + m_timing_vals("nBL32") + 1; 
      };
      m_actions[m_levels["rank"]][m_commands["RD32"]] = [this] (Node* node, int cmd, int target_id, Clk_t clk) {
        node->m_final_synced_cycle = clk + m_timing_vals("nCL") + m_timing_vals("nBL32"); 
      };
      m_actions[m_levels["rank"]][m_commands["WR32"]] = [this] (Node* node, int cmd, int target_id, Clk_t clk) {
        node->m_final_synced_cycle = clk + m_timing_vals("nCWL") + m_timing_vals("nBL32"); 
      };
      // Bank actions
      m_actions[m_levels["bank"]][m_commands["ACT-1"]] = [] (Node* node, int cmd, int target_id, Clk_t clk) {
        node->m_state = m_states["Pre-Opened"];
        node->m_row_state[target_id] = m_states["Pre-Opened"];
      };
      m_actions[m_levels["bank"]][m_commands["ACT-2"]] = Lambdas::Action::Bank::ACT<LPDDR6>;
      m_actions[m_levels["bank"]][m_commands["PRE"]]   = Lambdas::Action::Bank::PRE<LPDDR6>;
      m_actions[m_levels["bank"]][m_commands["RD32A"]] = Lambdas::Action::Bank::PRE<LPDDR6>;
      m_actions[m_levels["bank"]][m_commands["WR32A"]] = Lambdas::Action::Bank::PRE<LPDDR6>;
    };

    void set_preqs() {
      m_preqs.resize(m_levels.size(), std::vector<PreqFunc_t<Node>>(m_commands.size()));

      // Rank Preqs
      m_preqs[m_levels["rank"]][m_commands["REFab"]] = Lambdas::Preq::Rank::RequireAllBanksClosed<LPDDR6>;
      m_preqs[m_levels["rank"]][m_commands["RFMab"]] = Lambdas::Preq::Rank::RequireAllBanksClosed<LPDDR6>;

      m_preqs[m_levels["rank"]][m_commands["REFpb"]] = [this] (Node* node, int cmd, int target_id, Clk_t clk) {
        int target_bank_id = target_id;
        int another_target_bank_id = target_id + 8;

        for (auto bg : node->m_child_nodes) {
          for (auto bank : bg->m_child_nodes) {
            int num_banks_per_bg = m_organization.count[m_levels["bank"]];
            int flat_bankid = bank->m_node_id + bg->m_node_id * num_banks_per_bg;
            if (flat_bankid == target_id || flat_bankid == another_target_bank_id) {
              switch (node->m_state) {
                case m_states["Pre-Opened"]: return m_commands["PRE"];
                case m_states["Opened"]: return m_commands["PRE"];
              }
            }
          }
        }

        return cmd;
      };
      
      m_preqs[m_levels["rank"]][m_commands["RFMpb"]] = m_preqs[m_levels["rank"]][m_commands["REFpb"]];

      // Bank Preqs
      m_preqs[m_levels["bank"]][m_commands["RD32"]] = [] (Node* node, int cmd, int target_id, Clk_t clk) {
        switch (node->m_state) {
          case m_states["Closed"]: return m_commands["ACT-1"];
          case m_states["Pre-Opened"]: return m_commands["ACT-2"];
          case m_states["Opened"]: {
            if (node->m_row_state.find(target_id) != node->m_row_state.end()) {
              Node* rank = node->m_parent_node->m_parent_node;
              if (rank->m_final_synced_cycle < clk) {
                return m_commands["CASRD"];
              } else {
                return cmd;
              }
            } else {
              return m_commands["PRE"];
            }
          }    
          default: {
            spdlog::error("[Preq::Bank] Invalid bank state for an RD/WR command!");
            std::exit(-1);      
          } 
        }
      };
      m_preqs[m_levels["bank"]][m_commands["WR32"]] = [] (Node* node, int cmd, int target_id, Clk_t clk) {
        switch (node->m_state) {
          case m_states["Closed"]: return m_commands["ACT-1"];
          case m_states["Pre-Opened"]: return m_commands["ACT-2"];
          case m_states["Opened"]: {
            if (node->m_row_state.find(target_id) != node->m_row_state.end()) {
              Node* rank = node->m_parent_node->m_parent_node;
              if (rank->m_final_synced_cycle < clk) {
                return m_commands["CASWR"];
              } else {
                return cmd;
              }
            } else {
              return m_commands["PRE"];
            }
          }    
          default: {
            spdlog::error("[Preq::Bank] Invalid bank state for an RD/WR command!");
            std::exit(-1);      
          } 
        }
      };
    };

    void set_rowhits() {
      m_rowhits.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));
      auto rowhit_func = [] (Node* node, int cmd, int target_id, Clk_t clk) {
        switch (node->m_state)  {
          case m_states["Closed"]: return false;
          case m_states["Pre-Opened"]: return false;
          case m_states["Opened"]:
            if (node->m_row_state.find(target_id) != node->m_row_state.end()) {
              return true;
            }
            else {
              return false;
            }
          default: {
            spdlog::error("[RowHit::Bank] Invalid bank state for an RD/WR command!");
            std::exit(-1);      
          }
        }
      };
      m_rowhits[m_levels["bank"]][m_commands["RD32"]] = rowhit_func;
      m_rowhits[m_levels["bank"]][m_commands["WR32"]] = rowhit_func;
    }


    void set_rowopens() {
      m_rowopens.resize(m_levels.size(), std::vector<RowhitFunc_t<Node>>(m_commands.size()));
      auto rowopen_func = [](Node* node, int cmd, int target_id, Clk_t clk) {
        switch (node->m_state)  {
          case m_states["Closed"]: return false;
          case m_states["Pre-Opened"]: return false;
          case m_states["Opened"]: return true;
          default: {
            spdlog::error("[RowHit::Bank] Invalid bank state for an RD/WR command!");
            std::exit(-1);      
          }
        }
      };
      m_rowopens[m_levels["bank"]][m_commands["RD32"]] = rowopen_func;
      m_rowopens[m_levels["bank"]][m_commands["WR32"]] = rowopen_func;
    }


    void create_nodes() {
      int num_channels = m_organization.count[m_levels["channel"]];
      for (int i = 0; i < num_channels; i++) {
        Node* channel = new Node(this, nullptr, 0, i);
        m_channels.push_back(channel);
      }
    };
};


}        // namespace Ramulator
