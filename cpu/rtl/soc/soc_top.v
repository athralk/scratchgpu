`default_nettype none
`include "memory_map.vh"
// Synapse-32 FPGA SoC: core + I/D caches + I/D TLBs + page walker + AXI3 master to the
// Zynq PS DDR (S_AXI_HP0), CLINT timer, UART, PLIC, and a control block for the ARM
// (M_AXI_GP0). One clock domain (PS FCLK_CLK0).
module soc_top #(
    parameter [31:0] DDR_BASE = 32'h20000000,
    parameter CORE_RESET_HELD = 1      // 0 in simulation: run without the ARM
) (
    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 clk CLK" *)
    (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF m_axi:s_axi, ASSOCIATED_RESET rst_n" *)
    input  wire        clk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 rst_n RST" *)
    (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        rst_n,           // PS FCLK_RESET0_N

    output wire        uart_tx,
    input  wire        uart_rx,

    // ---- AXI3 master -> S_AXI_HP0 (64-bit) ----
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME m_axi, PROTOCOL AXI3, DATA_WIDTH 64, ADDR_WIDTH 32, ID_WIDTH 6, MAX_BURST_LENGTH 4, NUM_READ_OUTSTANDING 1, NUM_WRITE_OUTSTANDING 4" *)
    output wire [5:0]  m_axi_awid,
    output wire [31:0] m_axi_awaddr,
    output wire [3:0]  m_axi_awlen,
    output wire [2:0]  m_axi_awsize,
    output wire [1:0]  m_axi_awburst,
    output wire [1:0]  m_axi_awlock,
    output wire [3:0]  m_axi_awcache,
    output wire [2:0]  m_axi_awprot,
    output wire [3:0]  m_axi_awqos,
    output wire        m_axi_awvalid,
    input  wire        m_axi_awready,
    output wire [5:0]  m_axi_wid,
    output wire [63:0] m_axi_wdata,
    output wire [7:0]  m_axi_wstrb,
    output wire        m_axi_wlast,
    output wire        m_axi_wvalid,
    input  wire        m_axi_wready,
    input  wire [5:0]  m_axi_bid,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,
    output wire [5:0]  m_axi_arid,
    output wire [31:0] m_axi_araddr,
    output wire [3:0]  m_axi_arlen,
    output wire [2:0]  m_axi_arsize,
    output wire [1:0]  m_axi_arburst,
    output wire [1:0]  m_axi_arlock,
    output wire [3:0]  m_axi_arcache,
    output wire [2:0]  m_axi_arprot,
    output wire [3:0]  m_axi_arqos,
    output wire        m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [5:0]  m_axi_rid,
    input  wire [63:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready,

    // ---- AXI3 slave <- M_AXI_GP0 (32-bit) ----
    (* X_INTERFACE_PARAMETER = "XIL_INTERFACENAME s_axi, PROTOCOL AXI3, DATA_WIDTH 32, ADDR_WIDTH 32, ID_WIDTH 12" *)
    input  wire [11:0] s_axi_awid,
    input  wire [31:0] s_axi_awaddr,
    input  wire [3:0]  s_axi_awlen,
    input  wire [2:0]  s_axi_awsize,
    input  wire [1:0]  s_axi_awburst,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [11:0] s_axi_wid,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wlast,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output wire [11:0] s_axi_bid,
    output wire [1:0]  s_axi_bresp,
    output wire        s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [11:0] s_axi_arid,
    input  wire [31:0] s_axi_araddr,
    input  wire [3:0]  s_axi_arlen,
    input  wire [2:0]  s_axi_arsize,
    input  wire [1:0]  s_axi_arburst,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output wire [11:0] s_axi_rid,
    output wire [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output wire        s_axi_rlast,
    output wire        s_axi_rvalid,
    input  wire        s_axi_rready
);

    // ---------------- Resets ----------------
    // Fabric logic (AXI, control block) runs from PS reset; the core and its caches also
    // wait for the ARM to release them through the control block.
    (* ASYNC_REG = "TRUE" *) reg [1:0] rst_sync;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rst_sync <= 2'b11;
        else        rst_sync <= {rst_sync[0], 1'b0};
    end
    wire sys_rst = rst_sync[1];
    // core_rst drives the asynchronous clear of every core and vector-unit flop (about 5k
    // endpoints). One register cannot reach them all within a clock period, so it is
    // replicated (max_fanout) after a first stage, and each copy serves a local group.
    wire ctrl_core_reset;
    reg core_rst_pre;
    (* max_fanout = 100 *) reg core_rst;
    always @(posedge clk) begin
        core_rst_pre <= sys_rst || ctrl_core_reset;
        core_rst <= core_rst_pre;
    end

    // ---------------- Core ----------------
    wire [31:0] pc, pc_next;
    wire [31:0] instr;
    wire instr_rvalid;
    wire instr_page_fault;
    wire [31:0] d_rdata;
    wire d_rvalid;
    wire d_load_pf, d_store_pf;
    // Core data port
    wire [31:0] cpu_wdata;
    wire cpu_wr_en, cpu_rd_en;
    wire [31:0] cpu_read_addr, cpu_write_addr, cpu_addr_next;
    wire [3:0] cpu_be;
    wire [2:0] cpu_load_type;
    wire cpu_write_intent;
    // Vector unit data port
    wire vpu_own, vpu_rd, vpu_wr;
    wire [31:0] vpu_addr, vpu_addr_next, vpu_wdata;
    wire [3:0] vpu_be;
    wire [2:0] vpu_load_type;
    // Shared D-side request: the vector unit owns it during a vector load/store (the core is
    // then waiting in MEM on that instruction and issues nothing of its own).
    wire d_rd_en = vpu_own ? vpu_rd : cpu_rd_en;
    wire d_wr_en = vpu_own ? vpu_wr : cpu_wr_en;
    wire [31:0] d_wdata = vpu_own ? vpu_wdata : cpu_wdata;
    wire [31:0] d_read_addr = vpu_own ? vpu_addr : cpu_read_addr;
    wire [31:0] d_write_addr = vpu_own ? vpu_addr : cpu_write_addr;
    wire [31:0] d_addr_next = vpu_own ? vpu_addr_next : cpu_addr_next;
    wire [3:0] d_be = vpu_own ? vpu_be : cpu_be;
    wire [2:0] d_load_type = vpu_own ? vpu_load_type : cpu_load_type;
    wire d_write_intent = vpu_own ? vpu_wr : cpu_write_intent;
    wire d_mmu_enable;
    wire [1:0] d_priv;
    wire [31:0] satp;
    wire d_sum, d_mxr;
    wire i_mmu_enable;
    wire [1:0] i_priv;
    wire fence_i, sfence_vma;
    wire timer_interrupt, plic_interrupt;

    riscv_cpu cpu_inst (
        .clk(clk),
        .rst(core_rst),
        .module_instr_in(instr),
        .module_read_data_in(d_rdata),
        .module_pc_out(pc),
        .module_wr_data_out(cpu_wdata),
        .module_mem_wr_en(cpu_wr_en),
        .module_mem_rd_en(cpu_rd_en),
        .module_read_addr(cpu_read_addr),
        .module_write_addr(cpu_write_addr),
        .module_write_byte_enable(cpu_be),
        .module_load_type(cpu_load_type),
        .module_load_page_fault_in(d_load_pf),
        .module_store_page_fault_in(d_store_pf),
        .module_page_fault_addr_in(d_write_addr),
        .module_instr_page_fault_in(instr_page_fault),
        .module_instr_gnt_in(instr_rvalid),
        .module_instr_rvalid_in(instr_rvalid),
        .module_data_gnt_in(d_rvalid),
        .module_data_rvalid_in(d_rvalid),
        .module_data_write_intent_out(cpu_write_intent),
        .module_data_mmu_enable_out(d_mmu_enable),
        .module_data_privilege_out(d_priv),
        .module_satp_out(satp),
        .module_data_sum_out(d_sum),
        .module_data_mxr_out(d_mxr),
        .module_instr_mmu_enable_out(i_mmu_enable),
        .module_instr_privilege_out(i_priv),
        .module_pc_next_out(pc_next),
        .module_data_addr_next_out(cpu_addr_next),
        .module_fence_i_out(fence_i),
        .module_sfence_vma_out(sfence_vma),
        .vec_issue_valid(vec_issue_valid), .vec_issue_ready(vec_issue_ready),
        .vec_issue_instr(vec_issue_instr), .vec_issue_rs1(vec_issue_rs1),
        .vec_issue_rs2(vec_issue_rs2), .vec_issue_vl(vec_issue_vl),
        .vec_issue_vtype(vec_issue_vtype), .vec_issue_vstart(vec_issue_vstart),
        .vec_issue_vxrm(vec_issue_vxrm), .vec_issue_frm(vec_issue_frm),
        .vec_resp_valid(vec_resp_valid), .vec_resp_data(vec_resp_data),
        .vec_resp_fault(vec_resp_fault), .vec_resp_fault_store(vec_resp_fault_store),
        .vec_resp_fault_addr(vec_resp_fault_addr), .vec_resp_fault_vstart(vec_resp_fault_vstart),
        .vec_resp_vl_valid(vec_resp_vl_valid), .vec_resp_vl(vec_resp_vl),
        .vec_busy(vec_busy), .vec_vxsat_set(vec_vxsat_set), .vec_fflags_set(vec_fflags_set),
        .timer_interrupt(timer_interrupt),
        .software_interrupt(1'b0),
        .external_interrupt(plic_interrupt)
    );

    // ---------------- Vector unit (TinyGPU v2) ----------------
    wire vec_issue_valid, vec_issue_ready;
    wire [31:0] vec_issue_instr, vec_issue_rs1, vec_issue_rs2, vec_issue_vl, vec_issue_vtype, vec_issue_vstart;
    wire [1:0] vec_issue_vxrm;
    wire [2:0] vec_issue_frm;
    wire vec_resp_valid, vec_resp_fault, vec_resp_fault_store, vec_resp_vl_valid, vec_busy, vec_vxsat_set;
    wire [31:0] vec_resp_data, vec_resp_fault_addr, vec_resp_fault_vstart, vec_resp_vl;
    wire [4:0] vec_fflags_set;

    vpu_top vpu (
        .clk(clk), .rst(core_rst),
        .issue_valid(vec_issue_valid), .issue_ready(vec_issue_ready),
        .issue_instr(vec_issue_instr), .issue_rs1(vec_issue_rs1), .issue_rs2(vec_issue_rs2),
        .issue_vl(vec_issue_vl), .issue_vtype(vec_issue_vtype), .issue_vstart(vec_issue_vstart),
        .issue_vxrm(vec_issue_vxrm), .issue_frm(vec_issue_frm),
        .resp_valid(vec_resp_valid), .resp_data(vec_resp_data), .resp_fault(vec_resp_fault),
        .resp_fault_store(vec_resp_fault_store), .resp_fault_addr(vec_resp_fault_addr),
        .resp_fault_vstart(vec_resp_fault_vstart), .resp_vl_valid(vec_resp_vl_valid),
        .resp_vl(vec_resp_vl), .busy(vec_busy), .vxsat_set(vec_vxsat_set), .fflags_set(vec_fflags_set),
        .mem_rd(vpu_rd), .mem_wr(vpu_wr), .mem_addr(vpu_addr), .mem_addr_next(vpu_addr_next),
        .mem_own(vpu_own), .mem_wdata(vpu_wdata), .mem_be(vpu_be), .mem_load_type(vpu_load_type),
        .mem_rvalid(d_rvalid), .mem_rdata(d_rdata),
        .mem_load_pf(d_load_pf), .mem_store_pf(d_store_pf)
    );

    // ---------------- Page walker ----------------
    wire ptw_fill_d, ptw_fill_i, ptw_fault_d, ptw_fault_i;
    wire [19:0] ptw_vpn;
    wire [31:0] ptw_pte;
    wire ptw_mega;
    wire ptw_mem_req, ptw_mem_gnt, ptw_mem_rvalid;
    wire [31:0] ptw_mem_addr, ptw_mem_rdata;
    wire ptw_d_req, ptw_i_req;

    // ---------------- FENCE.I / SFENCE.VMA ----------------
    // The core's pulses depend on the memory stage advancing (a long path), so they are
    // registered and applied one cycle later. Fetch is held during that cycle so the refetch
    // after the fence cannot use the old I-cache lines or translations.
    reg fence_i_q, sfence_q;
    always @(posedge clk) begin
        fence_i_q <= !core_rst && fence_i;
        sfence_q <= !core_rst && sfence_vma;
    end
    wire i_hold = fence_i_q || sfence_q;

    // ---------------- Instruction translation ----------------
    wire [19:0] i_vpn = pc[31:12];
    wire itlb_hit;
    wire [19:0] itlb_ppn;
    wire [7:0] itlb_flags;
    tlb #(.ENTRIES(8)) itlb (
        .clk(clk), .rst(core_rst), .flush(sfence_q),
        .lookup_vpn(i_vpn), .hit(itlb_hit), .ppn(itlb_ppn), .pte_flags(itlb_flags),
        .fill(ptw_fill_i), .fill_vpn(ptw_vpn), .fill_pte(ptw_pte), .fill_mega(ptw_mega)
    );
    wire i_perm_fault;
    sv32_instr_check i_check (
        .translate_enable(i_mmu_enable),
        .addr_valid_in(itlb_hit),
        .privilege_mode(i_priv),
        .leaf_pte({24'b0, itlb_flags}),
        .page_fault(i_perm_fault),
        .update_accessed()
    );
    wire i_walk_fault = ptw_fault_i && (ptw_vpn == i_vpn);
    wire [19:0] i_ppn = i_mmu_enable ? itlb_ppn : pc[31:12];
    wire i_ready = !i_hold && (!i_mmu_enable || (itlb_hit && !i_perm_fault));
    assign ptw_i_req = !i_hold && i_mmu_enable && !itlb_hit && !i_walk_fault;
    assign instr_page_fault = !i_hold && i_mmu_enable && ((itlb_hit && i_perm_fault) || i_walk_fault);

    // ---------------- Data translation ----------------
    // The memory stage sees a few registered translations (L0): the VPN, PPN, permission results
    // and cacheability of recent pages, so the D-cache response depends on 20-bit compares
    // instead of the TLB search, PPN mux and permission check. A miss costs one cycle:
    // the D-TLB is searched with the registered VPN and L0 is filled (or the page walker is
    // started). L0 is tagged with the translation context (MMU on, privilege, SUM, MXR), so any
    // change of those misses it; SFENCE.VMA clears it.
    wire d_active = d_rd_en || d_wr_en;
    wire [31:0] d_vaddr = d_wr_en ? d_write_addr : d_read_addr;
    wire [19:0] d_vpn = d_vaddr[31:12];
    wire d_xlate = d_mmu_enable && d_active;
    wire [4:0] d_ctx = {d_mmu_enable, d_priv, d_sum, d_mxr};

    // L0 entries (4, round-robin), each with its permission results and cacheability
    localparam L0N = 4;
    reg [L0N-1:0] l0_valid;
    reg [19:0]    l0_vpn [0:L0N-1];
    reg [19:0]    l0_ppn [0:L0N-1];
    reg [4:0]     l0_ctx [0:L0N-1];
    reg [L0N-1:0] l0_ld_pf, l0_st_pf, l0_cacheable;
    reg [1:0]     l0_victim;
    // Both candidate addresses are compared in parallel; the read/write select comes last.
    reg [L0N-1:0] l0_match;
    // Kept as separate compares so synthesis does not move them behind the read/write select
    // (the write enable arrives late).
    (* keep = "true" *) wire [L0N-1:0] l0_eq_r, l0_eq_w;
    genvar lg;
    generate for (lg = 0; lg < L0N; lg = lg + 1) begin : g_l0eq
        assign l0_eq_r[lg] = (l0_vpn[lg] == d_read_addr[31:12]);
        assign l0_eq_w[lg] = (l0_vpn[lg] == d_write_addr[31:12]);
    end endgenerate
    reg [19:0]    l0_hit_ppn;
    reg           l0_hit_ld_pf, l0_hit_st_pf, l0_hit_cacheable;
    integer li;
    always @(*) begin
        l0_hit_ppn = 20'b0;
        l0_hit_ld_pf = 1'b0;
        l0_hit_st_pf = 1'b0;
        l0_hit_cacheable = 1'b0;
        for (li = 0; li < L0N; li = li + 1) begin
            l0_match[li] = l0_valid[li] && (l0_ctx[li] == d_ctx) &&
                           (d_wr_en ? l0_eq_w[li] : l0_eq_r[li]);
            if (l0_match[li]) begin
                l0_hit_ppn = l0_hit_ppn | l0_ppn[li];
                l0_hit_ld_pf = l0_hit_ld_pf | l0_ld_pf[li];
                l0_hit_st_pf = l0_hit_st_pf | l0_st_pf[li];
                l0_hit_cacheable = l0_hit_cacheable | l0_cacheable[li];
            end
        end
    end
    wire l0_hit = |l0_match;
    wire d_walk_fault = ptw_fault_d && (ptw_vpn == d_vpn) && d_xlate;

    wire l0_fill;
    // L0 miss: search the D-TLB next cycle with the registered page
    reg        lk_valid;
    reg [19:0] lk_vpn;
    reg [4:0]  lk_ctx;
    always @(posedge clk) begin
        // A miss that is being filled this very cycle is not looked up again (no duplicate entries)
        lk_valid <= !core_rst && !sfence_q && d_active && !l0_hit && !d_walk_fault &&
                    !(l0_fill && lk_vpn == d_vpn && lk_ctx == d_ctx);
        lk_vpn <= d_vpn;
        lk_ctx <= d_ctx;
    end
    wire lk_mmu = lk_ctx[4];
    wire dtlb_hit;
    wire [19:0] dtlb_ppn;
    wire [7:0] dtlb_flags;
    tlb #(.ENTRIES(8)) dtlb (
        .clk(clk), .rst(core_rst), .flush(sfence_q),
        .lookup_vpn(lk_vpn), .hit(dtlb_hit), .ppn(dtlb_ppn), .pte_flags(dtlb_flags),
        .fill(ptw_fill_d), .fill_vpn(ptw_vpn), .fill_pte(ptw_pte), .fill_mega(ptw_mega)
    );
    // Permission results for both access kinds, computed once when L0 is filled
    wire fill_ld_pf, fill_st_pf;
    sv32_data_check d_check_ld (
        .translate_enable(1'b1), .addr_valid_in(1'b1),
        .privilege_mode(lk_ctx[3:2]), .sum(lk_ctx[1]), .mxr(lk_ctx[0]),
        .data_rd_en(1'b1), .data_wr_req(1'b0), .leaf_pte({24'b0, dtlb_flags}),
        .load_page_fault(fill_ld_pf), .store_page_fault(), .update_accessed(), .update_dirty()
    );
    sv32_data_check d_check_st (
        .translate_enable(1'b1), .addr_valid_in(1'b1),
        .privilege_mode(lk_ctx[3:2]), .sum(lk_ctx[1]), .mxr(lk_ctx[0]),
        .data_rd_en(1'b0), .data_wr_req(1'b1), .leaf_pte({24'b0, dtlb_flags}),
        .load_page_fault(), .store_page_fault(fill_st_pf), .update_accessed(), .update_dirty()
    );
    wire [19:0] fill_ppn = lk_mmu ? dtlb_ppn : lk_vpn;
    assign l0_fill = lk_valid && (!lk_mmu || dtlb_hit);
    always @(posedge clk) begin
        if (core_rst || sfence_q) begin
            l0_valid <= {L0N{1'b0}};
            l0_victim <= 2'd0;
        end else if (l0_fill) begin
            l0_valid[l0_victim] <= 1'b1;
            l0_vpn[l0_victim] <= lk_vpn;
            l0_ctx[l0_victim] <= lk_ctx;
            l0_ppn[l0_victim] <= fill_ppn;
            l0_ld_pf[l0_victim] <= lk_mmu && fill_ld_pf;
            l0_st_pf[l0_victim] <= lk_mmu && fill_st_pf;
            l0_cacheable[l0_victim] <= `IS_SOC_DRAM({fill_ppn, 12'h000});
            l0_victim <= l0_victim + 2'd1;
        end
    end
    assign ptw_d_req = lk_valid && lk_mmu && !dtlb_hit && !(ptw_fault_d && ptw_vpn == lk_vpn);

    wire d_is_load = d_rd_en && !d_write_intent;
    wire d_perm_fault = (d_is_load && l0_hit_ld_pf) || (d_write_intent && l0_hit_st_pf);
    assign d_load_pf = (l0_hit && d_is_load && l0_hit_ld_pf) || (d_walk_fault && d_is_load);
    assign d_store_pf = (l0_hit && d_write_intent && l0_hit_st_pf) || (d_walk_fault && d_write_intent);
    wire [19:0] d_ppn = l0_hit_ppn;
    wire d_ready = d_active && l0_hit && !d_perm_fault;
    wire [31:0] d_paddr = {d_ppn, d_vaddr[11:0]};
    wire [31:0] i_paddr = {i_ppn, pc[11:0]};

    ptw ptw_inst (
        .clk(clk), .rst(core_rst), .flush(sfence_q), .satp(satp),
        .d_req(ptw_d_req), .d_vpn(lk_vpn),
        .i_req(ptw_i_req), .i_vpn(i_vpn),
        .fill_d(ptw_fill_d), .fill_i(ptw_fill_i),
        .fault_d(ptw_fault_d), .fault_i(ptw_fault_i),
        .res_vpn(ptw_vpn), .res_pte(ptw_pte), .res_mega(ptw_mega),
        .mem_req(ptw_mem_req), .mem_addr(ptw_mem_addr), .mem_gnt(ptw_mem_gnt),
        .mem_rvalid(ptw_mem_rvalid), .mem_rdata(ptw_mem_rdata)
    );

    // ---------------- Caches ----------------
    wire irf_req, irf_order, irf_gnt, irf_beat, irf_last;
    wire [31:0] irf_addr;
    wire drf_req, drf_gnt, drf_beat, drf_last;
    wire [31:0] drf_addr;
    wire [63:0] rf_data;
    wire wb_valid, wb_pop, wb_empty, write_idle;
    wire [31:0] wb_addr, wb_data;
    wire [3:0] wb_be;
    wire io_rd, io_wr;
    wire [31:0] io_addr, io_wdata;
    reg  [31:0] io_rdata;

    icache icache_inst (
        .clk(clk), .rst(core_rst), .invalidate(fence_i_q),
        .pc_next(pc_next), .pc(pc),
        .lookup_valid(i_ready), .cacheable(`IS_SOC_DRAM(i_paddr)), .ppn(i_ppn),
        .hit(instr_rvalid), .instr(instr),
        .rf_req(irf_req), .rf_addr(irf_addr), .rf_order(irf_order), .rf_gnt(irf_gnt),
        .rf_beat(irf_beat), .rf_data(rf_data), .rf_last(irf_last)
    );

    dcache dcache_inst (
        .clk(clk), .rst(core_rst),
        .req_rd(d_rd_en), .req_wr(d_wr_en), .vaddr(d_vaddr), .addr_next(d_addr_next),
        .wdata(d_wdata), .be(d_be), .load_type(d_load_type),
        .lookup_valid(d_ready), .ppn(d_ppn), .cacheable(l0_hit_cacheable),
        .cand_ppn({l0_ppn[3], l0_ppn[2], l0_ppn[1], l0_ppn[0]}), .cand_match(l0_match),
        .rvalid(d_rvalid), .rdata(d_rdata),
        .inval_req(1'b0), .inval_addr(32'b0),
        .rf_req(drf_req), .rf_addr(drf_addr), .rf_gnt(drf_gnt),
        .rf_beat(drf_beat), .rf_data(rf_data), .rf_last(drf_last),
        .wb_valid(wb_valid), .wb_addr(wb_addr), .wb_data(wb_data), .wb_be(wb_be),
        .wb_pop(wb_pop), .wb_empty(wb_empty), .write_idle(write_idle),
        .io_rd(io_rd), .io_wr(io_wr), .io_addr(io_addr), .io_wdata(io_wdata),
        .io_rdata(io_rdata)
    );

    // ---------------- Memory master ----------------
    axi_mem_master #(.DDR_BASE(DDR_BASE)) mem_master (
        .clk(clk), .rst(core_rst),
        .ptw_req(ptw_mem_req), .ptw_addr(ptw_mem_addr), .ptw_gnt(ptw_mem_gnt),
        .ptw_rvalid(ptw_mem_rvalid), .ptw_rdata(ptw_mem_rdata),
        .drf_req(drf_req), .drf_addr(drf_addr), .drf_gnt(drf_gnt),
        .drf_beat(drf_beat), .drf_last(drf_last),
        .irf_req(irf_req), .irf_addr(irf_addr), .irf_order(irf_order), .irf_gnt(irf_gnt),
        .irf_beat(irf_beat), .irf_last(irf_last),
        .rf_data(rf_data),
        .wb_valid(wb_valid), .wb_addr(wb_addr), .wb_data(wb_data), .wb_be(wb_be),
        .wb_empty(wb_empty), .wb_pop(wb_pop), .write_idle(write_idle),
        .m_axi_awid(m_axi_awid), .m_axi_awaddr(m_axi_awaddr), .m_axi_awlen(m_axi_awlen),
        .m_axi_awsize(m_axi_awsize), .m_axi_awburst(m_axi_awburst), .m_axi_awlock(m_axi_awlock),
        .m_axi_awcache(m_axi_awcache), .m_axi_awprot(m_axi_awprot), .m_axi_awqos(m_axi_awqos),
        .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wid(m_axi_wid), .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb),
        .m_axi_wlast(m_axi_wlast), .m_axi_wvalid(m_axi_wvalid), .m_axi_wready(m_axi_wready),
        .m_axi_bid(m_axi_bid), .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid),
        .m_axi_bready(m_axi_bready),
        .m_axi_arid(m_axi_arid), .m_axi_araddr(m_axi_araddr), .m_axi_arlen(m_axi_arlen),
        .m_axi_arsize(m_axi_arsize), .m_axi_arburst(m_axi_arburst), .m_axi_arlock(m_axi_arlock),
        .m_axi_arcache(m_axi_arcache), .m_axi_arprot(m_axi_arprot), .m_axi_arqos(m_axi_arqos),
        .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rid(m_axi_rid), .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rlast(m_axi_rlast), .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready)
    );

    // ---------------- MMIO ----------------
    wire timer_sel = `IS_TIMER_MEM(io_addr);
    wire uart_sel  = `IS_UART_MEM(io_addr);
    wire plic_sel  = `IS_PLIC_MEM(io_addr);
    wire [31:0] timer_rdata, uart_rdata, plic_rdata;
    wire uart_interrupt;

    always @(*) begin
        io_rdata = timer_sel ? timer_rdata :
                   uart_sel  ? uart_rdata :
                   plic_sel  ? plic_rdata : 32'h0;
    end

    timer timer_inst (
        .clk(clk), .rst(core_rst),
        .addr(io_addr), .write_data(io_wdata),
        .write_enable(io_wr && timer_sel), .read_enable(io_rd && timer_sel),
        .read_data(timer_rdata), .timer_valid(), .timer_interrupt(timer_interrupt)
    );

    // 16550 timing; reset divisor 27 gives 115200 baud at 50 MHz (27 * 16 = 432 clocks per bit)
    uart #(.OVERSAMPLE(16), .DEFAULT_BAUD_DIV(16'd27)) uart_inst (
        .clk(clk), .rst(core_rst),
        .addr(io_addr), .write_data(io_wdata),
        .write_enable(io_wr && uart_sel), .read_enable(io_rd && uart_sel),
        .read_data(uart_rdata), .uart_valid(), .interrupt(uart_interrupt),
        .tx(uart_tx), .rx(uart_rx)
    );

    plic plic_inst (
        .clk(clk), .rst(core_rst),
        .addr(io_addr), .write_data(io_wdata),
        .write_enable(io_wr && plic_sel), .read_enable(io_rd && plic_sel),
        .read_data(plic_rdata), .plic_valid(),
        .source_irq(uart_interrupt), .external_interrupt(plic_interrupt)
    );

    // ---------------- ARM control block ----------------
    ps_ctrl #(.DDR_BASE(DDR_BASE), .RESET_HELD(CORE_RESET_HELD)) ctrl_inst (
        .clk(clk), .rst(sys_rst),
        .core_reset(ctrl_core_reset), .pc_debug(pc),
        .s_axi_awid(s_axi_awid), .s_axi_awaddr(s_axi_awaddr), .s_axi_awlen(s_axi_awlen),
        .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
        .s_axi_wid(s_axi_wid), .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb),
        .s_axi_wlast(s_axi_wlast), .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
        .s_axi_bid(s_axi_bid), .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid),
        .s_axi_bready(s_axi_bready),
        .s_axi_arid(s_axi_arid), .s_axi_araddr(s_axi_araddr), .s_axi_arlen(s_axi_arlen),
        .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
        .s_axi_rid(s_axi_rid), .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
        .s_axi_rlast(s_axi_rlast), .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready)
    );

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
