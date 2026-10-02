`default_nettype none
`include "memory_map.vh"
module riscv_cpu (
    input wire clk,
    input wire rst,
    input wire [31:0] module_instr_in,
    input wire [31:0] module_read_data_in,
    output wire [31:0] module_pc_out,
    output wire [31:0] module_wr_data_out,
    output wire module_mem_wr_en,
    output wire module_mem_rd_en,
    output wire [31:0] module_read_addr,
    output wire [31:0] module_write_addr,
    output wire [3:0] module_write_byte_enable,  // Write byte enables
    output wire [2:0] module_load_type,          // Load type
    input wire module_load_page_fault_in,
    input wire module_store_page_fault_in,
    input wire [31:0] module_page_fault_addr_in,
    input wire module_instr_page_fault_in,
    // Memory interface: a request is outstanding until its response arrives.
    input wire module_instr_gnt_in,
    input wire module_instr_rvalid_in,
    input wire module_data_gnt_in,
    input wire module_data_rvalid_in,
    // The MMU treats a whole AMO as a write, its read transaction included.
    output wire module_data_write_intent_out,
    output wire module_data_mmu_enable_out,
    output wire [1:0] module_data_privilege_out,
    output wire [31:0] module_satp_out,
    output wire module_data_sum_out,
    output wire module_data_mxr_out,
    output wire module_instr_mmu_enable_out,
    output wire [1:0] module_instr_privilege_out,
    // Early addresses for synchronous cache arrays: the PC and the MEM data address of next cycle.
    output wire [31:0] module_pc_next_out,
    output wire [31:0] module_data_addr_next_out,
    // One-cycle pulses when FENCE.I / SFENCE.VMA execute: invalidate I$ / flush TLBs.
    output wire module_fence_i_out,
    output wire module_sfence_vma_out,

    // Vector unit (TinyGPU v2). A vector instruction is issued from MEM, where nothing older
    // can still trap. Arithmetic is fire-and-forget; scalar results and vector loads/stores
    // hold MEM until the unit responds (a fault there traps precisely, setting vstart).
    output wire        vec_issue_valid,
    input  wire        vec_issue_ready,
    output wire [31:0] vec_issue_instr,
    output wire [31:0] vec_issue_rs1,
    output wire [31:0] vec_issue_rs2,
    output wire [31:0] vec_issue_vl,
    output wire [31:0] vec_issue_vtype,
    output wire [31:0] vec_issue_vstart,
    output wire [1:0]  vec_issue_vxrm,
    output wire [2:0]  vec_issue_frm,
    input  wire        vec_resp_valid,
    input  wire [31:0] vec_resp_data,
    input  wire        vec_resp_fault,
    input  wire        vec_resp_fault_store,
    input  wire [31:0] vec_resp_fault_addr,
    input  wire [31:0] vec_resp_fault_vstart,
    input  wire        vec_resp_vl_valid,
    input  wire [31:0] vec_resp_vl,
    input  wire        vec_busy,
    input  wire        vec_vxsat_set,
    input  wire [4:0]  vec_fflags_set,

    // Interrupt inputs
    input wire timer_interrupt,
    input wire software_interrupt,
    input wire external_interrupt
);
`include "instr_defines.vh"

    // Instantiate PC
    wire [31:0] pc_inst0_out;
    wire pc_inst0_j_signal;
    wire [31:0] pc_inst0_jump;
    wire hazard_stall; // For load-use hazards
    wire pipeline_stall;
    wire fetch_wait;
    wire fetch_response_valid;
    wire mem_wait;
    wire ex_stage_active;
    wire muldiv_busy;  // an M instruction in EX still computing
    wire mem_stage_page_fault_taken;
    wire mem_stage_load_page_fault;
    wire mem_stage_store_page_fault;
    wire mem_stage_trap_to_supervisor;
    wire [31:0] mem_stage_jump_addr;
    wire instr_stage_page_fault_taken;
    wire instr_stage_trap_to_supervisor;
    wire [31:0] instr_stage_jump_addr;
    // CSR state exported by csr_file
    wire [31:0] csr_mstatus;
    wire [31:0] csr_medeleg;
    wire [31:0] csr_mideleg;
    wire [31:0] csr_mie;
    wire [31:0] csr_mip;
    wire [31:0] csr_mtvec;
    wire [31:0] csr_mepc;
    wire [31:0] csr_mcounteren;
    wire [31:0] csr_stvec;
    wire [31:0] csr_sepc;
    wire [31:0] csr_scounteren;
    wire [31:0] csr_satp;
    wire [1:0] csr_privilege_mode;
    wire [31:0] csr_vl, csr_vtype, csr_vstart;
    wire [1:0] csr_vxrm, csr_vs, csr_fs;
    wire [2:0] csr_frm;
    wire vset_we;
    wire [31:0] vset_vl, vset_vtype;
    wire vec_illegal;
    wire mem_is_fp;
    wire vec_fault;
    wire vec_resp_done;
    wire vec_retire_mem;
    wire vec_wait;
    wire vcsr_stall;
    // Priority: MEM page fault > instr page fault > EX branch/trap
    assign pc_inst0_j_signal = mem_stage_page_fault_taken || instr_stage_page_fault_taken || ex_inst0_jump_signal_out;
    assign pc_inst0_jump = mem_stage_page_fault_taken   ? mem_stage_jump_addr :
                           instr_stage_page_fault_taken ? instr_stage_jump_addr :
                           ex_inst0_jump_addr_out;
    pc pc_inst0 (
        .clk(clk),
        .rst(rst),
        .j_signal(pc_inst0_j_signal),
        .jump(pc_inst0_jump),
        .stall(pipeline_stall || fetch_wait), // Stall on hazard, WFI sleep or a waiting memory
        .out(pc_inst0_out),
        .next_out(module_pc_next_out)
    );

    // Send out the PC value
    assign module_pc_out = pc_inst0_out;


    // Instantiate IF_ID pipeline register
    wire [31:0] if_id_pc_out;
    wire [31:0] if_id_instr_out;
    wire if_id_instr_valid_out;
    wire if_id_instr_page_fault_out;
    wire execution_flush;
    wire branch_flush;
    wire if_id_flush;
    assign branch_flush = ex_inst0_jump_signal_out; // Flush IF/ID if branch taken
    assign if_id_flush = branch_flush || execution_flush || mem_stage_page_fault_taken || instr_stage_page_fault_taken;
    IF_ID if_id_inst0 (
        .clk(clk),
        .rst(rst),
        .pc_in(pc_inst0_out),
        .instruction_in(module_instr_in),
        .instr_page_fault_in(module_instr_page_fault_in),
        // A fetch still in flight delivers a bubble; while stalled, IF/ID keeps what it holds.
        .flush(if_id_flush || (fetch_wait && !pipeline_stall)),
        // Flush must win over stall, otherwise a stale IF/ID instruction can
        // survive an interrupt/branch redirect and execute one cycle later.
        .stall(pipeline_stall && !if_id_flush),
        .pc_out(if_id_pc_out),
        .instruction_out(if_id_instr_out),
        .instruction_valid_out(if_id_instr_valid_out),
        .instr_page_fault_out(if_id_instr_page_fault_out)
    );

    // Instantiate Decoder
    wire [4:0] decoder_inst0_rs1_out;
    wire [4:0] decoder_inst0_rs2_out;
    wire [4:0] decoder_inst0_rd_out;
    wire [31:0] decoder_inst0_imm_out;
    wire decoder_inst0_rs1_valid_out;
    wire decoder_inst0_rs2_valid_out;
    wire decoder_inst0_rd_valid_out;
    wire [6:0] decoder_inst0_opcode_out;
    wire [6:0] decoder_inst0_instr_id_out;

    decoder decoder_inst0 (
        .instr(if_id_instr_out),
        .rs2(decoder_inst0_rs2_out),
        .rs1(decoder_inst0_rs1_out),
        .imm(decoder_inst0_imm_out),
        .rd(decoder_inst0_rd_out),
        .rs1_valid(decoder_inst0_rs1_valid_out),
        .rs2_valid(decoder_inst0_rs2_valid_out),
        .rd_valid(decoder_inst0_rd_valid_out),
        .opcode(decoder_inst0_opcode_out),
        .instr_id(decoder_inst0_instr_id_out)
    );
    
    // Instantiate Load-Use Hazard Detector
    load_use_detector load_use_detector_inst0 (
        .rs1_id(decoder_inst0_rs1_out),
        .rs2_id(decoder_inst0_rs2_out),
        .rs1_valid_id(decoder_inst0_rs1_valid_out),
        .rs2_valid_id(decoder_inst0_rs2_valid_out),
        .instr_id_ex(id_ex_inst0_instr_id_out),
        .rd_ex(id_ex_inst0_rd_addr_out),
        .rd_valid_ex(id_ex_inst0_rd_valid_out),
        .stall_pipeline(hazard_stall)
    );

    // Instantiate Register File
    wire [31:0] rf_inst0_rs1_value_out;
    wire [31:0] rf_inst0_rs2_value_out;
    // RD control signals will be later handled by WB stage
    wire [4:0] rf_inst0_rd_in;
    wire rf_inst0_wr_en;
    wire [31:0] rf_inst0_rd_value_in;


    registerfile rf_inst0 (
        .clk(clk),
        .rst(rst),
        .rs1(decoder_inst0_rs1_out),
        .rs2(decoder_inst0_rs2_out),
        .rs1_valid(decoder_inst0_rs1_valid_out),
        .rs2_valid(decoder_inst0_rs2_valid_out),
        .rd(rf_inst0_rd_in),
        .wr_en(rf_inst0_wr_en),
        .rd_value(rf_inst0_rd_value_in),
        .rs1_value(rf_inst0_rs1_value_out),
        .rs2_value(rf_inst0_rs2_value_out)
    );

    // Instantiate ID_EX pipeline register
    wire id_ex_inst0_rs1_valid_out;
    wire id_ex_inst0_rs2_valid_out;
    wire id_ex_inst0_rd_valid_out;
    wire [31:0] id_ex_inst0_imm_out;
    wire [4:0] id_ex_inst0_rs1_addr_out;
    wire [4:0] id_ex_inst0_rs2_addr_out;
    wire [4:0] id_ex_inst0_rd_addr_out;
    wire [6:0] id_ex_inst0_opcode_out;
    wire [6:0] id_ex_inst0_instr_id_out;
    wire [31:0] id_ex_inst0_pc_out;
    wire [31:0] id_ex_inst0_instr_out;
    wire [31:0] id_ex_inst0_rs1_value_out;
    wire [31:0] id_ex_inst0_rs2_value_out;
    wire id_ex_inst0_instr_valid_out;
    wire id_ex_inst0_instr_page_fault_out;

    // Pipeline flush signals
    wire pipeline_flush;

    // Combine branch flush and execution unit flush
    assign pipeline_flush = branch_flush || execution_flush || mem_stage_page_fault_taken || instr_stage_page_fault_taken;

    ID_EX id_ex_inst0 (
        .clk(clk),
        .rst(rst),
        .rs1_valid_in(decoder_inst0_rs1_valid_out),
        .rs2_valid_in(decoder_inst0_rs2_valid_out),
        .rd_valid_in(decoder_inst0_rd_valid_out),
        .imm_in(decoder_inst0_imm_out),
        .rs1_addr_in(decoder_inst0_rs1_out),
        .rs2_addr_in(decoder_inst0_rs2_out),
        .rd_addr_in(decoder_inst0_rd_out),
        .opcode_in(decoder_inst0_opcode_out),
        .instr_id_in(decoder_inst0_instr_id_out),
        .pc_in(if_id_pc_out),
        .instr_in(if_id_instr_out),
        .rs1_value_in(rf_inst0_rs1_value_out),
        .rs2_value_in(rf_inst0_rs2_value_out),
        .rs1_value_resolved_in(ex_inst0_rs1_value_out),
        .rs2_value_resolved_in(ex_inst0_rs2_value_out),
        .instr_valid_in(if_id_instr_valid_out),
        .instr_page_fault_in(if_id_instr_page_fault_out),
        .flush(pipeline_flush),
        .hold(pipeline_hold && !pipeline_flush),
        .stall(hazard_stall && !pipeline_flush),
        .rs1_valid_out(id_ex_inst0_rs1_valid_out),
        .rs2_valid_out(id_ex_inst0_rs2_valid_out),
        .rd_valid_out(id_ex_inst0_rd_valid_out),
        .imm_out(id_ex_inst0_imm_out),
        .rs1_addr_out(id_ex_inst0_rs1_addr_out),
        .rs2_addr_out(id_ex_inst0_rs2_addr_out),
        .rd_addr_out(id_ex_inst0_rd_addr_out),
        .opcode_out(id_ex_inst0_opcode_out),
        .instr_id_out(id_ex_inst0_instr_id_out),
        .pc_out(id_ex_inst0_pc_out),
        .instr_out(id_ex_inst0_instr_out),
        .rs1_value_out(id_ex_inst0_rs1_value_out),
        .rs2_value_out(id_ex_inst0_rs2_value_out),
        .instr_valid_out(id_ex_inst0_instr_valid_out),
        .instr_page_fault_out(id_ex_inst0_instr_page_fault_out)
    );

    // Instantiate Execution Unit
    wire [31:0] ex_inst0_exec_output_out;
    wire ex_inst0_jump_signal_out;
    wire [31:0] ex_inst0_jump_addr_out;
    wire [31:0] ex_inst0_mem_addr_out;
    wire [31:0] ex_inst0_rs1_value_out;
    wire [31:0] ex_inst0_rs2_value_out;
    
    // Forwarding unit signals
    wire [1:0] forward_a;
    wire [1:0] forward_b;
    
    // Instantiate forwarding unit
    forwarding_unit forwarding_unit_inst0 (
        .rs1_addr_ex(id_ex_inst0_rs1_addr_out),
        .rs2_addr_ex(id_ex_inst0_rs2_addr_out),
        .rs1_valid_ex(id_ex_inst0_rs1_valid_out),
        .rs2_valid_ex(id_ex_inst0_rs2_valid_out),
        .rd_addr_mem(ex_mem_inst0_rd_addr_out),
        .rd_valid_mem(ex_mem_inst0_rd_valid_out),
        .instr_id_mem(ex_mem_inst0_instr_id_out),
        .rd_addr_wb(mem_wb_inst0_rd_addr_out),
        .rd_valid_wb(mem_wb_inst0_rd_valid_out),
        .wr_en_wb(wb_inst0_wr_en_out),
        .forward_a(forward_a),
        .forward_b(forward_b)
    );

    // CSR file signals
    wire [11:0] csr_addr;
    wire [31:0] csr_read_data;
    wire [31:0] csr_write_data;
    wire csr_write_enable;
    wire csr_read_enable;
    wire csr_valid;

    // Interrupt controller signals
    wire interrupt_pending;
    wire [31:0] interrupt_cause;
    wire [31:0] interrupt_pc;
    wire interrupt_taken;
    wire interrupt_taken_qualified;
    wire interrupt_to_supervisor;
    wire mret_instruction;
    wire sret_instruction;
    wire trap_to_supervisor;
    wire ecall_exception;
    wire ebreak_exception;
    wire illegal_instruction_exception;
    wire instruction_address_misaligned_exception;
    wire load_address_misaligned_exception;
    wire store_address_misaligned_exception;
    wire [31:0] exception_tval;
    wire synchronous_exception_taken;
    wire [31:0] csr_exception_pc;
    wire csr_trap_to_supervisor;
    wire [31:0] csr_exception_tval;
    wire wfi_instruction;
    wire breakpoint_trigger_exception;
    wire execute_trigger_hit;
    wire [27:0] trigger_control;
    wire [127:0] trigger_tdata2;
    wire instret_increment;
    wire [31:0] exception_pc;
    assign exception_pc = id_ex_inst0_pc_out;
    assign synchronous_exception_taken = ecall_exception || ebreak_exception ||
                                         breakpoint_trigger_exception ||
                                         illegal_instruction_exception ||
                                         instruction_address_misaligned_exception ||
                                         load_address_misaligned_exception ||
                                         store_address_misaligned_exception;
    assign instret_increment = id_ex_inst0_instr_valid_out &&
                               ex_stage_active &&
                               !muldiv_busy && !vcsr_stall &&
                               (id_ex_inst0_instr_id_out != INSTR_INVALID) &&
                               !interrupt_taken &&
                               !synchronous_exception_taken &&
                               !mem_stage_page_fault_taken;

    localparam PRIV_U = 2'b00;
    localparam PRIV_S = 2'b01;
    localparam PRIV_M = 2'b11;
    wire data_use_mprv = (csr_privilege_mode == PRIV_M) && csr_mstatus[17];
    wire [1:0] data_privilege_mode = data_use_mprv ? csr_mstatus[12:11] : csr_privilege_mode;
    wire data_mmu_enable = (data_privilege_mode != PRIV_M) && csr_satp[31];
    assign module_data_mmu_enable_out = data_mmu_enable;
    assign module_data_privilege_out = data_privilege_mode;
    assign module_satp_out = csr_satp;
    assign module_data_sum_out = csr_mstatus[18];
    assign module_data_mxr_out = csr_mstatus[19];
    assign module_instr_mmu_enable_out = (csr_privilege_mode != PRIV_M) && csr_satp[31];
    assign module_instr_privilege_out = csr_privilege_mode;
    // Sdtrig: a trigger fires only in its selected mode, and not while that mode's interrupts are
    // disabled, or a handler would retrigger on itself (Sdtrig "Native Triggers", no tcontrol).
    function trigger_enabled_in_mode;
        input [6:0] control;
        input [1:0] mode;
        input machine_interrupts_enabled;
        input supervisor_interrupts_enabled;
        begin
            trigger_enabled_in_mode =
                (mode == PRIV_M) ? (control[6] && machine_interrupts_enabled) :
                (mode == PRIV_S) ? (control[4] && supervisor_interrupts_enabled) :
                                   control[3];
        end
    endfunction

    wire breakpoint_delegated = csr_medeleg[3];
    wire [3:0] trigger_enabled = {
        trigger_enabled_in_mode(trigger_control[27:21], csr_privilege_mode,
                                csr_mstatus[3], !breakpoint_delegated || csr_mstatus[1]),
        trigger_enabled_in_mode(trigger_control[20:14], csr_privilege_mode,
                                csr_mstatus[3], !breakpoint_delegated || csr_mstatus[1]),
        trigger_enabled_in_mode(trigger_control[13:7], csr_privilege_mode,
                                csr_mstatus[3], !breakpoint_delegated || csr_mstatus[1]),
        trigger_enabled_in_mode(trigger_control[6:0], csr_privilege_mode,
                                csr_mstatus[3], !breakpoint_delegated || csr_mstatus[1])
    };

    // Trap targets (privileged spec 3.1.7): exceptions at BASE, vectored interrupts at BASE + 4 * cause.
    wire [31:0] mtvec_base = {csr_mtvec[31:2], 2'b00};
    wire [31:0] stvec_base = {csr_stvec[31:2], 2'b00};
    wire interrupt_vectored = interrupt_to_supervisor ? csr_stvec[0] : csr_mtvec[0];
    wire [31:0] interrupt_vector = (interrupt_to_supervisor ? stvec_base : mtvec_base) +
                                   (interrupt_vectored ? {interrupt_cause[29:0], 2'b00} : 32'h0);
    assign interrupt_taken_qualified = interrupt_taken &&
                                       !synchronous_exception_taken &&
                                       !mem_stage_page_fault_taken &&
                                       !instr_stage_page_fault_taken;

    // WFI sleep state: stall fetch/decode until an enabled interrupt becomes
    // pending, even if the corresponding global interrupt-enable bit is clear.
    // Trap delivery still uses interrupt_pending below; this only controls
    // whether WFI can resume.
    reg wfi_active;
    reg fetch_outstanding;
    reg fetch_discard;
    wire wfi_stall;
    wire wfi_resume_pending;
    localparam [31:0] WFI_WAKE_INTERRUPT_MASK = 32'h00000AAA;
    assign wfi_resume_pending =
        |(csr_mip & csr_mie & WFI_WAKE_INTERRUPT_MASK);
    assign wfi_stall = wfi_active && !wfi_resume_pending;

    // One-entry store buffer (for future decoupled memory interfaces).
    reg store_buf_valid;
    reg [31:0] store_buf_addr;
    reg [31:0] store_buf_data;
    reg [3:0] store_buf_be;
    wire ex_mem_std_store_raw_req;
    wire ex_mem_std_store_req;
    wire ex_mem_std_store_direct_req;
    wire [31:0] ex_mem_store_addr;
    wire [31:0] ex_mem_store_data;
    wire [3:0] ex_mem_store_be;
    wire ex_mem_read_req;
    wire [31:0] ex_mem_read_addr;
    wire [2:0] ex_mem_read_type;
    wire non_atomic_store_write_enable;
    wire [31:0] non_atomic_store_write_addr;
    reg [31:0] mem_read_data_effective;
    wire load_all_bytes_covered;
    wire read_needs_memory;
    wire store_buf_commit_fire;
    wire atomic_clobbers_store_buf;

    // An AMO is a read then a write, since the value written depends on the value read.
    reg amo_write_phase;
    reg [31:0] amo_read_data;
    wire amo_read_phase;

    // Atomic LSU signals (produced by atomic_lsu module in MEM stage).
    wire is_lr_w;
    wire is_sc_w;
    wire is_amo_w;
    wire atomic_read_enable;
    wire atomic_write_enable;
    wire sc_success;
    wire sc_wait;
    wire [31:0] sc_result;
    wire [31:0] atomic_new_word;
    // FENCE must wait for any queued store to drain.
    wire store_buf_busy_stall = 1'b0;
    wire fence_drain_stall = (id_ex_inst0_instr_id_out == INSTR_FENCE) && store_buf_valid;
    // A page-faulting access never reaches memory, so it is not waited for.
    // A fetch accepted before a redirect answers for the old PC, so its response is dropped.
    assign fetch_response_valid = module_instr_rvalid_in && !fetch_discard;
    assign fetch_wait = !fetch_response_valid && !module_instr_page_fault_in;
    // Outstanding until rvalid; an AMO's read response still leaves its write to issue.
    assign mem_wait = ((module_mem_rd_en || module_mem_wr_en) &&
                       !(module_data_rvalid_in && !amo_read_phase) &&
                       !mem_stage_page_fault_taken) ||
                      (vec_wait && !mem_stage_page_fault_taken) ||
                      sc_wait;                    // SC's reservation-check cycle
    assign ex_stage_active = !mem_wait;
    wire pipeline_hold = wfi_stall || store_buf_busy_stall || fence_drain_stall || mem_wait || muldiv_busy ||
                         vcsr_stall;
    assign pipeline_stall = hazard_stall || pipeline_hold;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            amo_write_phase <= 1'b0;
            amo_read_data <= 32'b0;
            fetch_outstanding <= 1'b0;
            fetch_discard <= 1'b0;
            wfi_active <= 1'b0;
            store_buf_valid <= 1'b0;
            store_buf_addr <= 32'b0;
            store_buf_data <= 32'b0;
            store_buf_be <= 4'b0;
        end else begin
            if (module_instr_rvalid_in) begin
                fetch_outstanding <= 1'b0;
                fetch_discard <= 1'b0;
            end else if (module_instr_gnt_in) begin
                fetch_outstanding <= 1'b1;
            end
            if (pc_inst0_j_signal && (fetch_outstanding || module_instr_gnt_in) &&
                !module_instr_rvalid_in) begin
                fetch_discard <= 1'b1;
            end

            if (wfi_resume_pending) begin
                wfi_active <= 1'b0;
            end else if (wfi_instruction) begin
                wfi_active <= 1'b1;
            end

            // AMO phase: capture the value read, then let the write go out.
            if (amo_read_phase && module_data_rvalid_in) begin
                amo_read_data <= mem_read_data_effective;
                amo_write_phase <= 1'b1;
            end else if (amo_write_phase && module_data_rvalid_in) begin
                amo_write_phase <= 1'b0;
            end

            // One-entry store buffer update. Frozen while MEM waits for memory.
            // If a store is committed and no new store is entering, clear valid.
            // If a new store enters, capture it (and replace any just-committed entry).
            if (mem_wait) begin
                store_buf_valid <= store_buf_valid;
            end else if (ex_mem_std_store_req) begin
                store_buf_valid <= 1'b1;
                store_buf_addr <= ex_mem_store_addr;
                store_buf_data <= ex_mem_store_data;
                store_buf_be <= ex_mem_store_be;
            end else if (atomic_clobbers_store_buf) begin
                // Older store to the same word is architecturally consumed by the
                // AMO/SC full-word write and must not commit afterward.
                store_buf_valid <= 1'b0;
            end else if (store_buf_commit_fire) begin
                store_buf_valid <= 1'b0;
            end
        end
    end

    // Instantiate interrupt controller
    interrupt_controller int_ctrl_inst (
        .clk(clk),
        .rst(rst),
        .timer_interrupt(timer_interrupt),
        .software_interrupt(software_interrupt),
        .external_interrupt(external_interrupt),
        .mstatus(csr_mstatus),
        .mie(csr_mie),
        .mip(csr_mip),
        .mideleg(csr_mideleg),
        .privilege_mode(csr_privilege_mode),
        .interrupt_pending(interrupt_pending),
        .interrupt_cause(interrupt_cause),
        .interrupt_to_supervisor(interrupt_to_supervisor),
        .interrupt_taken(interrupt_taken_qualified),
        // Oldest unexecuted instruction: EX, else IF/ID (EX bubble), else fetch PC.
        .current_pc(id_ex_inst0_instr_valid_out ? id_ex_inst0_pc_out :
                    if_id_instr_valid_out      ? if_id_pc_out :
                                                 pc_inst0_out),
        .interrupt_pc(interrupt_pc)
    );

    // Instantiate CSR file at CPU level
    csr_file csr_file_inst (
        .clk(clk),
        .rst(rst),
        .csr_addr(csr_addr),
        .write_data(csr_write_data),
        .write_enable(csr_write_enable && ex_stage_active && !vcsr_stall),
        .read_enable(csr_read_enable),
        .read_data(csr_read_data),
        .csr_valid(csr_valid),
        .interrupt_pending(interrupt_pending),
        .interrupt_cause_in(interrupt_cause),
        .interrupt_pc_in(interrupt_pc),
        .exception_pc_in(csr_exception_pc),
        .interrupt_taken(interrupt_taken_qualified),
        .mret_instruction(mret_instruction),
        .sret_instruction(sret_instruction),
        .interrupt_to_supervisor(interrupt_to_supervisor),
        .trap_to_supervisor(csr_trap_to_supervisor),
        .ecall_exception(ecall_exception),
        .ebreak_exception(ebreak_exception),
        .illegal_instruction_exception(illegal_instruction_exception),
        .instruction_address_misaligned_exception(instruction_address_misaligned_exception),
        .load_address_misaligned_exception(load_address_misaligned_exception),
        .store_address_misaligned_exception(store_address_misaligned_exception),
        .instr_page_fault_exception(instr_stage_page_fault_taken),
        .load_page_fault_exception(mem_stage_load_page_fault),
        .store_page_fault_exception(mem_stage_store_page_fault),
        .breakpoint_trigger_exception(breakpoint_trigger_exception),
        .exception_tval_in(csr_exception_tval),
        .trigger_control(trigger_control),
        .trigger_tdata2(trigger_tdata2),
        .state_mstatus(csr_mstatus),
        .state_medeleg(csr_medeleg),
        .state_mideleg(csr_mideleg),
        .state_mie(csr_mie),
        .state_mip(csr_mip),
        .state_mtvec(csr_mtvec),
        .state_mepc(csr_mepc),
        .state_mcounteren(csr_mcounteren),
        .state_stvec(csr_stvec),
        .state_sepc(csr_sepc),
        .state_scounteren(csr_scounteren),
        .state_satp(csr_satp),
        .state_privilege_mode(csr_privilege_mode),
        .vset_we(vset_we),
        .vset_vl(vset_vl),
        .vset_vtype(vset_vtype),
        .vec_retire(vec_retire_mem),
        .vec_fault_we(vec_fault),
        .vec_fault_vstart(vec_resp_fault_vstart),
        .vec_vl_we(vec_resp_done && vec_resp_vl_valid && !vec_resp_fault),
        .vec_vl_new(vec_resp_vl),
        .vxsat_set(vec_vxsat_set),
        .fflags_set(vec_fflags_set),
        .fp_retire(vec_retire_mem && mem_is_fp),
        .state_frm(csr_frm),
        .state_fs(csr_fs),
        .state_vl(csr_vl),
        .state_vtype(csr_vtype),
        .state_vstart(csr_vstart),
        .state_vxrm(csr_vxrm),
        .state_vs(csr_vs),
        .instret_increment(instret_increment),
        .timer_interrupt(timer_interrupt),
        .software_interrupt(software_interrupt),
        .external_interrupt(external_interrupt)
    );

    // Value available for EX-stage forwarding from MEM stage.
    // SC computes its architectural result in MEM, so forward that instead of
    // the raw EX result for dependent instructions.
    wire [31:0] ex_mem_forward_result = is_sc_w ? sc_result : ex_mem_inst0_exec_output_out;

    execution_unit ex_unit_inst0 (
        .rs1(id_ex_inst0_rs1_value_out),
        .rs2(id_ex_inst0_rs2_value_out),
        .imm(id_ex_inst0_imm_out),
        .rs1_addr(id_ex_inst0_rs1_addr_out),
        .rs2_addr(id_ex_inst0_rs2_addr_out),
        .opcode(id_ex_inst0_opcode_out),
        .instr_id(id_ex_inst0_instr_id_out),
        .rs1_valid(id_ex_inst0_rs1_valid_out),
        .rs2_valid(id_ex_inst0_rs2_valid_out),
        .instr_valid(id_ex_inst0_instr_valid_out),
        // Held while a vector/FP CSR read waits for the coprocessor: no flush/redirect yet.
        .stage_enable(ex_stage_active && !vcsr_stall),
        .pc_input(id_ex_inst0_pc_out),
        .instr(id_ex_inst0_instr_out),
        .vec_illegal(vec_illegal),
        .vset_vl_in(vset_vl),
        .forward_a(forward_a),
        .forward_b(forward_b),
        .ex_mem_result(ex_mem_forward_result),
        .mem_wb_result(wb_inst0_rd_value_out),
        
        // CSR interface connections
        .csr_read_data(csr_read_data),
        .csr_valid(csr_valid),
        .csr_addr(csr_addr),
        .csr_read_enable(csr_read_enable),
        .csr_write_data(csr_write_data),
        .csr_write_enable(csr_write_enable),
        
        .exec_output(ex_inst0_exec_output_out),
        .jump_signal(ex_inst0_jump_signal_out),
        .jump_addr(ex_inst0_jump_addr_out),
        .mem_addr(ex_inst0_mem_addr_out),
        .rs1_value_out(ex_inst0_rs1_value_out),
        .rs2_value_out(ex_inst0_rs2_value_out),
        .flush_pipeline(execution_flush),

        // Interrupt connections
        .interrupt_pending(interrupt_pending &&
                           !mem_stage_page_fault_taken &&
                           !instr_stage_page_fault_taken),
        .interrupt_cause(interrupt_cause),
        .interrupt_to_supervisor(interrupt_to_supervisor),
        .interrupt_vector(interrupt_vector),
        .mtvec(mtvec_base),
        .mepc(csr_mepc),
        .stvec(stvec_base),
        .sepc(csr_sepc),
        .medeleg(csr_medeleg),
        .privilege_mode(csr_privilege_mode),
        .mstatus(csr_mstatus),
        .mcounteren(csr_mcounteren),
        .scounteren(csr_scounteren),
        .trigger_enabled(trigger_enabled),
        .trigger_control(trigger_control),
        .trigger_tdata2(trigger_tdata2),
        .interrupt_taken(interrupt_taken),
        .mret_instruction(mret_instruction),
        .sret_instruction(sret_instruction),
        .trap_to_supervisor(trap_to_supervisor),
        .ecall_exception(ecall_exception),
        .ebreak_exception(ebreak_exception),
        .illegal_instruction_exception(illegal_instruction_exception),
        .instruction_address_misaligned_exception(instruction_address_misaligned_exception),
        .load_address_misaligned_exception(load_address_misaligned_exception),
        .store_address_misaligned_exception(store_address_misaligned_exception),
        .exception_tval(exception_tval),
        .wfi_instruction(wfi_instruction),
        .breakpoint_trigger_exception(breakpoint_trigger_exception),
        .execute_trigger_hit(execute_trigger_hit)
    );

    // RV32M: multi-cycle unit beside the ALU. EX holds the instruction until the result is ready.
    wire muldiv_op = id_ex_inst0_instr_valid_out &&
                     (id_ex_inst0_instr_id_out >= INSTR_MUL) && (id_ex_inst0_instr_id_out <= INSTR_REMU);
    wire muldiv_ready;
    wire [31:0] muldiv_result;
    assign muldiv_busy = muldiv_op && !muldiv_ready;

    muldiv muldiv_inst0 (
        .clk(clk),
        .rst(rst),
        .req(muldiv_op),
        .kill(pipeline_flush),
        .advance(!mem_wait),
        .instr_id(id_ex_inst0_instr_id_out),
        .a(ex_inst0_rs1_value_out),
        .b(ex_inst0_rs2_value_out),
        .ready(muldiv_ready),
        .result(muldiv_result)
    );

    // Next cycle's MEM address: EX/MEM holds while memory waits, otherwise it takes EX's address.
    assign module_data_addr_next_out = mem_wait ? ex_mem_inst0_mem_addr_out : ex_inst0_mem_addr_out;

    // A fence retires when it leaves EX with no trap taken in its place.
    wire ex_retires = id_ex_inst0_instr_valid_out && ex_stage_active && !muldiv_busy &&
                      !interrupt_taken && !synchronous_exception_taken &&
                      !mem_stage_page_fault_taken && !instr_stage_page_fault_taken;
    assign module_fence_i_out = ex_retires && (id_ex_inst0_instr_id_out == INSTR_FENCE_I);
    assign module_sfence_vma_out = ex_retires && (id_ex_inst0_instr_id_out == INSTR_SFENCE_VMA);

    // ---------------- Vector: vsetvl and legality in EX ----------------
    wire [31:0] ex_instr = id_ex_inst0_instr_out;
    wire ex_is_vsetvl = id_ex_inst0_instr_valid_out && (id_ex_inst0_instr_id_out == INSTR_VSETVL);
    wire ex_is_vop = id_ex_inst0_instr_valid_out &&
                     ((id_ex_inst0_instr_id_out == INSTR_VARITH) || (id_ex_inst0_instr_id_out == INSTR_VSCALAR) ||
                      (id_ex_inst0_instr_id_out == INSTR_VLOAD) || (id_ex_inst0_instr_id_out == INSTR_VSTORE));
    wire vs_off = (csr_vs == 2'b00);
    wire fs_off = (csr_fs == 2'b00);
    // Scalar FP (and vector FP) needs FS on; vector instructions need VS on.
    wire ex_is_fp = (ex_instr[6:0] == 7'b1010011) || (ex_instr[6:0] == 7'b1000011) ||
                    (ex_instr[6:0] == 7'b1000111) || (ex_instr[6:0] == 7'b1001011) ||
                    (ex_instr[6:0] == 7'b1001111) ||
                    (((ex_instr[6:0] == 7'b0000111) || (ex_instr[6:0] == 7'b0100111)) &&
                     ((ex_instr[14:12] == 3'b010) || (ex_instr[14:12] == 3'b011)));
    wire vdec_legal;
    vec_decode ex_vec_decode (
        .instr(ex_instr), .vtype(csr_vtype), .fp_enabled(!fs_off), .frm(csr_frm),
        .is_vector(), .legal(vdec_legal), .kind(), .is_load(), .is_store(), .scalar_dest(),
        .uses_rs1(), .uses_rs2(), .uses_frs1(), .eew_d(), .eew_s2(), .eew_s1()
    );
    assign vec_illegal = (ex_is_vop && ((vs_off && !ex_is_fp) || !vdec_legal)) || (ex_is_vsetvl && vs_off);

    vsetvl_unit vsetvl_inst0 (
        .instr(ex_instr),
        .rs1_value(ex_inst0_rs1_value_out),
        .rs2_value(ex_inst0_rs2_value_out),
        .cur_vl(csr_vl),
        .new_vl(vset_vl),
        .new_vtype(vset_vtype)
    );
    assign vset_we = ex_retires && ex_is_vsetvl && !vec_illegal;

    // vxsat / vcsr reads see every flag the unit has raised: wait for it to drain.
    wire ex_is_csr = id_ex_inst0_instr_valid_out && (ex_instr[6:0] == 7'b1110011) && (ex_instr[13:12] != 2'b00);
    assign vcsr_stall = ex_is_csr && vec_busy &&
                        ((ex_instr[31:20] == 12'h009) || (ex_instr[31:20] == 12'h00F) ||
                         (ex_instr[31:20] == 12'h001) || (ex_instr[31:20] == 12'h003));

    // Memory Stage

    // Instantiate EX_MEM pipeline register
    wire [4:0] ex_mem_inst0_rs1_addr_out;
    wire [4:0] ex_mem_inst0_rs2_addr_out;
    wire [4:0] ex_mem_inst0_rd_addr_out;
    wire [31:0] ex_mem_inst0_rs1_value_out;
    wire [31:0] ex_mem_inst0_rs2_value_out;
    wire [31:0] ex_mem_inst0_pc_out;
    wire [31:0] ex_mem_inst0_mem_addr_out;
    wire [31:0] ex_mem_inst0_exec_output_out;
    wire ex_mem_inst0_jump_signal_out;
    wire [31:0] ex_mem_inst0_jump_addr_out;
    wire [6:0] ex_mem_inst0_instr_id_out;
    wire ex_mem_inst0_rd_valid_out;
    wire [31:0] ex_mem_inst0_instr_out;

    EX_MEM ex_mem_inst0 (
        .clk(clk),
        .rst(rst),
        .hold(mem_wait),
        .flush(mem_stage_page_fault_taken ||
               instr_stage_page_fault_taken ||
               synchronous_exception_taken ||
               interrupt_taken_qualified ||
               ((muldiv_busy || vcsr_stall) && !mem_wait)),
        .rs1_addr_in(id_ex_inst0_rs1_addr_out),
        .rs2_addr_in(id_ex_inst0_rs2_addr_out),
        .rd_addr_in(id_ex_inst0_rd_addr_out),
        .rs1_value_in(ex_inst0_rs1_value_out),
        .rs2_value_in(ex_inst0_rs2_value_out),
        .pc_in(id_ex_inst0_pc_out),
        .mem_addr_in(ex_inst0_mem_addr_out),
        .exec_output_in(muldiv_op ? muldiv_result : ex_inst0_exec_output_out),
        .jump_signal_in(ex_inst0_jump_signal_out),
        .jump_addr_in(ex_inst0_jump_addr_out),
        .instr_id_in(id_ex_inst0_instr_id_out),
        .rd_valid_in(id_ex_inst0_rd_valid_out),
        .instr_in(id_ex_inst0_instr_out),
        .instr_out(ex_mem_inst0_instr_out),
        .rs1_addr_out(ex_mem_inst0_rs1_addr_out),
        .rs2_addr_out(ex_mem_inst0_rs2_addr_out),
        .rd_addr_out(ex_mem_inst0_rd_addr_out),
        .rs1_value_out(ex_mem_inst0_rs1_value_out),
        .rs2_value_out(ex_mem_inst0_rs2_value_out),
        .pc_out(ex_mem_inst0_pc_out),
        .mem_addr_out(ex_mem_inst0_mem_addr_out),
        .exec_output_out(ex_mem_inst0_exec_output_out),
        .jump_signal_out(ex_mem_inst0_jump_signal_out),
        .jump_addr_out(ex_mem_inst0_jump_addr_out),
        .instr_id_out(ex_mem_inst0_instr_id_out),
        .rd_valid_out(ex_mem_inst0_rd_valid_out)
    );

    // Instantiate Memory Unit
    wire mem_unit_inst0_wr_enable_out;
    wire mem_unit_inst0_read_enable_out;
    wire [31:0] mem_unit_inst0_wr_data_out;
    wire [31:0] mem_unit_inst0_read_addr_out;
    wire [31:0] mem_unit_inst0_wr_addr_out;
    wire [3:0] mem_unit_inst0_write_byte_enable_out;  // Write byte enables
    wire [2:0] mem_unit_inst0_load_type_out;          // Load type

    memory_unit mem_unit_inst0 (
        .instr_id(ex_mem_inst0_instr_id_out),
        .rs2_value(ex_mem_inst0_rs2_value_out),
        .mem_addr(ex_mem_inst0_mem_addr_out),
        .wr_enable(mem_unit_inst0_wr_enable_out),
        .read_enable(mem_unit_inst0_read_enable_out),
        .wr_data(mem_unit_inst0_wr_data_out),
        .read_addr(mem_unit_inst0_read_addr_out),
        .wr_addr(mem_unit_inst0_wr_addr_out),
        .write_byte_enable(mem_unit_inst0_write_byte_enable_out),
        .load_type(mem_unit_inst0_load_type_out)
    );

    atomic_lsu atomic_lsu_inst0 (
        .clk(clk),
        .rst(rst),
        .instr_id_mem(ex_mem_inst0_instr_id_out),
        .mem_addr_mem(ex_mem_inst0_mem_addr_out),
        .rs2_value_mem(ex_mem_inst0_rs2_value_out),
        // The new word is only written in the write phase, from the value the read returned
        // (registered). Feeding the live load data in the read phase too would leave a
        // never-used path from the D-cache response through the AMO ALU to the store data.
        .mem_read_data(amo_read_data),
        .mem_hold(mem_wait),
        .non_atomic_store_write_enable(non_atomic_store_write_enable),
        .non_atomic_store_write_addr(non_atomic_store_write_addr),
        .is_lr_w(is_lr_w),
        .is_sc_w(is_sc_w),
        .is_amo_w(is_amo_w),
        .atomic_read_enable(atomic_read_enable),
        .atomic_write_enable(atomic_write_enable),
        .sc_success(sc_success),
        .sc_wait(sc_wait),
        .sc_result(sc_result),
        .atomic_new_word(atomic_new_word)
    );

    // Store request generated in EX/MEM for SB/SH/SW.
    // For now, retire standard stores directly. The previous one-entry queue
    // needs a stronger hold/retire protocol before it is safe under Linux.
    assign ex_mem_std_store_raw_req = mem_unit_inst0_wr_enable_out &&
                                      (mem_unit_inst0_write_byte_enable_out != 4'b0000);
    assign ex_mem_store_addr = mem_unit_inst0_wr_addr_out;
    assign ex_mem_store_data = mem_unit_inst0_wr_data_out;
    assign ex_mem_store_be = mem_unit_inst0_write_byte_enable_out;
    assign ex_mem_std_store_req = 1'b0;
    assign ex_mem_std_store_direct_req = ex_mem_std_store_raw_req;

    // Read request for loads/LR/AMO.
    assign amo_read_phase = is_amo_w && !amo_write_phase;
    assign ex_mem_read_req = mem_unit_inst0_read_enable_out ||
                             (atomic_read_enable && !(is_amo_w && amo_write_phase));
    assign ex_mem_read_addr = atomic_read_enable ? ex_mem_inst0_mem_addr_out : mem_unit_inst0_read_addr_out;
    assign ex_mem_read_type = (is_lr_w || is_amo_w) ? 3'b010 : mem_unit_inst0_load_type_out;

    // Lookup a pending store-buffer byte by absolute byte address.
    function [8:0] store_buf_lookup_byte;
        input [31:0] addr;
        begin
            store_buf_lookup_byte = 9'h000;
            if (store_buf_valid && store_buf_be[0] && (store_buf_addr == addr)) begin
                store_buf_lookup_byte = {1'b1, store_buf_data[7:0]};
            end else if (store_buf_valid && store_buf_be[1] && ((store_buf_addr + 32'd1) == addr)) begin
                store_buf_lookup_byte = {1'b1, store_buf_data[15:8]};
            end else if (store_buf_valid && store_buf_be[2] && ((store_buf_addr + 32'd2) == addr)) begin
                store_buf_lookup_byte = {1'b1, store_buf_data[23:16]};
            end else if (store_buf_valid && store_buf_be[3] && ((store_buf_addr + 32'd3) == addr)) begin
                store_buf_lookup_byte = {1'b1, store_buf_data[31:24]};
            end
        end
    endfunction

    reg [8:0] load_byte0_lookup;
    reg [8:0] load_byte1_lookup;
    reg [8:0] load_byte2_lookup;
    reg [8:0] load_byte3_lookup;
    reg [7:0] load_byte0;
    reg [7:0] load_byte1;
    reg [7:0] load_byte2;
    reg [7:0] load_byte3;

    // Coverage check used to decide whether memory read is required.
    wire [8:0] cover_byte0_lookup = store_buf_lookup_byte(ex_mem_read_addr);
    wire [8:0] cover_byte1_lookup = store_buf_lookup_byte(ex_mem_read_addr + 32'd1);
    wire [8:0] cover_byte2_lookup = store_buf_lookup_byte(ex_mem_read_addr + 32'd2);
    wire [8:0] cover_byte3_lookup = store_buf_lookup_byte(ex_mem_read_addr + 32'd3);
    wire cover_byte0 = cover_byte0_lookup[8];
    wire cover_byte1 = cover_byte1_lookup[8];
    wire cover_byte2 = cover_byte2_lookup[8];
    wire cover_byte3 = cover_byte3_lookup[8];

    assign load_all_bytes_covered = !ex_mem_read_req ? 1'b0 :
                                    ((ex_mem_read_type == 3'b000) || (ex_mem_read_type == 3'b100)) ? cover_byte0 :
                                    ((ex_mem_read_type == 3'b001) || (ex_mem_read_type == 3'b101)) ? (cover_byte0 && cover_byte1) :
                                    (ex_mem_read_type == 3'b010) ? (cover_byte0 && cover_byte1 && cover_byte2 && cover_byte3) :
                                    1'b0;

    // Merge pending store-buffer bytes onto memory read data.
    always @(*) begin
        load_byte0_lookup = 9'h000;
        load_byte1_lookup = 9'h000;
        load_byte2_lookup = 9'h000;
        load_byte3_lookup = 9'h000;
        load_byte0 = module_read_data_in[7:0];
        load_byte1 = module_read_data_in[15:8];
        load_byte2 = module_read_data_in[23:16];
        load_byte3 = module_read_data_in[31:24];
        mem_read_data_effective = module_read_data_in;

        if (ex_mem_read_req) begin
            load_byte0_lookup = store_buf_lookup_byte(ex_mem_read_addr);
            if (load_byte0_lookup[8]) begin
                load_byte0 = load_byte0_lookup[7:0];
            end

            case (ex_mem_read_type)
                3'b000: begin // LB
                    mem_read_data_effective = {{24{load_byte0[7]}}, load_byte0};
                end
                3'b100: begin // LBU
                    mem_read_data_effective = {24'h0, load_byte0};
                end
                3'b001: begin // LH
                    load_byte1_lookup = store_buf_lookup_byte(ex_mem_read_addr + 32'd1);
                    if (load_byte1_lookup[8]) begin
                        load_byte1 = load_byte1_lookup[7:0];
                    end
                    mem_read_data_effective = {{16{load_byte1[7]}}, load_byte1, load_byte0};
                end
                3'b101: begin // LHU
                    load_byte1_lookup = store_buf_lookup_byte(ex_mem_read_addr + 32'd1);
                    if (load_byte1_lookup[8]) begin
                        load_byte1 = load_byte1_lookup[7:0];
                    end
                    mem_read_data_effective = {16'h0, load_byte1, load_byte0};
                end
                3'b010: begin // LW (also LR/AMO read path)
                    load_byte1_lookup = store_buf_lookup_byte(ex_mem_read_addr + 32'd1);
                    load_byte2_lookup = store_buf_lookup_byte(ex_mem_read_addr + 32'd2);
                    load_byte3_lookup = store_buf_lookup_byte(ex_mem_read_addr + 32'd3);
                    if (load_byte1_lookup[8]) begin
                        load_byte1 = load_byte1_lookup[7:0];
                    end
                    if (load_byte2_lookup[8]) begin
                        load_byte2 = load_byte2_lookup[7:0];
                    end
                    if (load_byte3_lookup[8]) begin
                        load_byte3 = load_byte3_lookup[7:0];
                    end
                    mem_read_data_effective = {load_byte3, load_byte2, load_byte1, load_byte0};
                end
                default: begin
                    mem_read_data_effective = module_read_data_in;
                end
            endcase
        end
    end

    // Use memory when the load/atomic read is not fully covered by the store buffer.
    assign read_needs_memory = ex_mem_read_req && !load_all_bytes_covered;

    // Commit buffered store only when memory read/write port is free this cycle.
    assign store_buf_commit_fire = store_buf_valid && !read_needs_memory &&
                                   !atomic_write_enable && !ex_mem_std_store_direct_req;
    assign non_atomic_store_write_enable = ex_mem_std_store_direct_req || store_buf_commit_fire;
    assign non_atomic_store_write_addr = ex_mem_std_store_direct_req ? ex_mem_store_addr : store_buf_addr;
    assign atomic_clobbers_store_buf = store_buf_valid && atomic_write_enable &&
                                       (store_buf_addr[31:2] == ex_mem_inst0_mem_addr_out[31:2]);

    // External memory interface arbitration:
    // - Standard stores are buffered then committed from store_buf.
    // - AMO/SC writes are driven directly.
    // - Loads/LR/AMO reads use memory only when needed; otherwise bypass from store_buf.
    assign module_mem_wr_en = (atomic_write_enable && (!is_amo_w || amo_write_phase)) ||
                              ex_mem_std_store_direct_req || store_buf_commit_fire;
    assign module_mem_rd_en = read_needs_memory;
    // AMO/SC and direct stores all write at the MEM address (ex_mem_store_addr is the same
    // value); only a store-buffer commit uses another. Selecting on the commit keeps
    // atomic_write_enable out of the address that the data TLB compares.
    assign module_write_addr = store_buf_commit_fire ? store_buf_addr : ex_mem_inst0_mem_addr_out;
    assign module_read_addr = ex_mem_read_addr;
    assign module_wr_data_out = atomic_write_enable ?
                                (is_amo_w ? atomic_new_word : ex_mem_inst0_rs2_value_out) :
                                (ex_mem_std_store_direct_req ? ex_mem_store_data : store_buf_data);
    assign module_write_byte_enable = atomic_write_enable ? 4'b1111 :
                                      (ex_mem_std_store_direct_req ? ex_mem_store_be : store_buf_be);
    assign module_load_type = ex_mem_read_type;
    assign module_data_write_intent_out = module_mem_wr_en || is_amo_w;
    assign mem_stage_load_page_fault = (module_load_page_fault_in && ex_mem_read_req) ||
                                       (vec_fault && !vec_resp_fault_store);
    // An AMO's read half takes its store/AMO page fault, or MEM would wait on a blocked write.
    assign mem_stage_store_page_fault = (module_store_page_fault_in && module_data_write_intent_out) ||
                                        (vec_fault && vec_resp_fault_store);
    assign mem_stage_page_fault_taken = mem_stage_load_page_fault || mem_stage_store_page_fault;
    assign mem_stage_trap_to_supervisor =
        (csr_privilege_mode != PRIV_M) &&
        ((mem_stage_load_page_fault && csr_medeleg[13]) ||
         (mem_stage_store_page_fault && csr_medeleg[15]));
    assign mem_stage_jump_addr = mem_stage_trap_to_supervisor ? stvec_base : mtvec_base;

    // An execute trigger is higher priority than the fetch page fault of the same instruction.
    assign instr_stage_page_fault_taken = id_ex_inst0_instr_page_fault_out &&
                                          id_ex_inst0_instr_valid_out &&
                                          !execute_trigger_hit;
    assign instr_stage_trap_to_supervisor =
        (csr_privilege_mode != PRIV_M) && csr_medeleg[12];
    assign instr_stage_jump_addr = instr_stage_trap_to_supervisor ? stvec_base : mtvec_base;

    assign csr_exception_pc = mem_stage_page_fault_taken   ? ex_mem_inst0_pc_out :
                              instr_stage_page_fault_taken ? id_ex_inst0_pc_out :
                              exception_pc;
    assign csr_trap_to_supervisor = mem_stage_page_fault_taken   ? mem_stage_trap_to_supervisor :
                                    instr_stage_page_fault_taken ? instr_stage_trap_to_supervisor :
                                    trap_to_supervisor;
    assign csr_exception_tval = mem_stage_page_fault_taken   ? (vec_fault ? vec_resp_fault_addr :
                                                                ex_mem_inst0_mem_addr_out) :
                                instr_stage_page_fault_taken ? id_ex_inst0_pc_out :
                                exception_tval;

    // ---------------- Vector: issue from MEM ----------------
    wire vec_in_mem = (ex_mem_inst0_instr_id_out == INSTR_VARITH) ||
                      (ex_mem_inst0_instr_id_out == INSTR_VSCALAR) ||
                      (ex_mem_inst0_instr_id_out == INSTR_VLOAD) ||
                      (ex_mem_inst0_instr_id_out == INSTR_VSTORE);
    wire vec_sync = vec_in_mem && (ex_mem_inst0_instr_id_out != INSTR_VARITH);
    reg vec_issued;
    wire vec_issue_fire = vec_issue_valid && vec_issue_ready;
    assign vec_issue_valid = vec_in_mem && !vec_issued;
    assign vec_resp_done = vec_sync && vec_issued && vec_resp_valid;
    assign vec_wait = vec_in_mem && (vec_sync ? !vec_resp_done : !(vec_issued || vec_issue_fire));
    assign vec_fault = vec_resp_done && vec_resp_fault;
    assign vec_retire_mem = vec_in_mem && !vec_wait && !vec_fault;

    always @(posedge clk or posedge rst) begin
        if (rst) vec_issued <= 1'b0;
        else if (!vec_in_mem || !vec_wait) vec_issued <= 1'b0;   // the instruction leaves MEM
        else if (vec_issue_fire) vec_issued <= 1'b1;
    end

    assign vec_issue_instr = ex_mem_inst0_instr_out;
    // Memory ops carry their effective address (base + imm for flw/fsw/fld/fsd)
    assign mem_is_fp = (ex_mem_inst0_instr_out[6:0] == 7'b1010011) || (ex_mem_inst0_instr_out[6:0] == 7'b1000011) ||
                     (ex_mem_inst0_instr_out[6:0] == 7'b1000111) || (ex_mem_inst0_instr_out[6:0] == 7'b1001011) ||
                     (ex_mem_inst0_instr_out[6:0] == 7'b1001111) ||
                     (((ex_mem_inst0_instr_out[6:0] == 7'b0000111) || (ex_mem_inst0_instr_out[6:0] == 7'b0100111)) &&
                      ((ex_mem_inst0_instr_out[14:12] == 3'b010) || (ex_mem_inst0_instr_out[14:12] == 3'b011)));
    assign vec_issue_rs1 = ((ex_mem_inst0_instr_id_out == INSTR_VLOAD) || (ex_mem_inst0_instr_id_out == INSTR_VSTORE)) ?
                           ex_mem_inst0_mem_addr_out : ex_mem_inst0_rs1_value_out;
    assign vec_issue_rs2 = ex_mem_inst0_rs2_value_out;
    assign vec_issue_vl = csr_vl;
    assign vec_issue_vtype = csr_vtype;
    assign vec_issue_vstart = csr_vstart;
    assign vec_issue_vxrm = csr_vxrm;
    assign vec_issue_frm = csr_frm;

    // Instantiate MEM_WB pipeline register
    wire [4:0] mem_wb_inst0_rs1_addr_out;
    wire [4:0] mem_wb_inst0_rs2_addr_out;
    wire [4:0] mem_wb_inst0_rd_addr_out;
    wire [31:0] mem_wb_inst0_rs1_value_out;
    wire [31:0] mem_wb_inst0_rs2_value_out;
    wire [31:0] mem_wb_inst0_pc_out;
    wire [31:0] mem_wb_inst0_mem_addr_out;
    wire [31:0] mem_wb_inst0_exec_output_out;
    wire mem_wb_inst0_jump_signal_out;
    wire [31:0] mem_wb_inst0_jump_addr_out;
    wire [6:0] mem_wb_inst0_instr_id_out;
    wire mem_wb_inst0_rd_valid_out;
    wire [31:0] mem_wb_inst0_mem_data_out;

    wire [31:0] ex_mem_exec_output_to_mem_wb = is_sc_w ? sc_result : ex_mem_inst0_exec_output_out;

    MEM_WB mem_wb_inst0 (
        .clk(clk),
        .rst(rst),
        // While MEM waits, WB takes a bubble: the access has not completed.
        .flush(mem_wait),
        .rs1_addr_in(ex_mem_inst0_rs1_addr_out),
        .rs2_addr_in(ex_mem_inst0_rs2_addr_out),
        .rd_addr_in(ex_mem_inst0_rd_addr_out),
        .rs1_value_in(ex_mem_inst0_rs1_value_out),
        .rs2_value_in(ex_mem_inst0_rs2_value_out),
        .pc_in(ex_mem_inst0_pc_out),
        .mem_addr_in(ex_mem_inst0_mem_addr_out),
        .exec_output_in(ex_mem_exec_output_to_mem_wb),
        .jump_signal_in(ex_mem_inst0_jump_signal_out),
        .jump_addr_in(ex_mem_inst0_jump_addr_out),
        .instr_id_in(mem_stage_page_fault_taken ? 7'b0000000 : ex_mem_inst0_instr_id_out),
        .rd_valid_in(mem_stage_page_fault_taken ? 1'b0 : ex_mem_inst0_rd_valid_out),
        // An AMO writes back the value it read, not what memory holds after its write.
        .mem_data_in((ex_mem_inst0_instr_id_out == INSTR_VSCALAR) ? vec_resp_data :
                     amo_write_phase ? amo_read_data : mem_read_data_effective),

        // Outputs
        .rs1_addr_out(mem_wb_inst0_rs1_addr_out),
        .rs2_addr_out(mem_wb_inst0_rs2_addr_out),
        .rd_addr_out(mem_wb_inst0_rd_addr_out),
        .rs1_value_out(mem_wb_inst0_rs1_value_out),
        .rs2_value_out(mem_wb_inst0_rs2_value_out),
        .pc_out(mem_wb_inst0_pc_out),
        .mem_addr_out(mem_wb_inst0_mem_addr_out),
        .exec_output_out(mem_wb_inst0_exec_output_out),
        .jump_signal_out(mem_wb_inst0_jump_signal_out),
        .jump_addr_out(mem_wb_inst0_jump_addr_out),
        .instr_id_out(mem_wb_inst0_instr_id_out),
        .rd_valid_out(mem_wb_inst0_rd_valid_out),
        .mem_data_out(mem_wb_inst0_mem_data_out)  // Output to WB stage
    );

    // Instantiate Write Back Stage
    wire wb_inst0_wr_en_out;
    wire [4:0] wb_inst0_rd_addr_out;
    wire [31:0] wb_inst0_rd_value_out;

    assign rf_inst0_rd_in = wb_inst0_rd_addr_out;
    assign rf_inst0_wr_en = wb_inst0_wr_en_out;
    assign rf_inst0_rd_value_in = wb_inst0_rd_value_out;

    writeback wb_inst0 (
        .rd_valid_in(mem_wb_inst0_rd_valid_out),
        .rd_addr_in(mem_wb_inst0_rd_addr_out),
        .rd_value_in(mem_wb_inst0_exec_output_out),
        .mem_data_in(mem_wb_inst0_mem_data_out),  // Use pipelined data
        .instr_id_in(mem_wb_inst0_instr_id_out),
        .rd_addr_out(wb_inst0_rd_addr_out),
        .rd_value_out(wb_inst0_rd_value_out),
        .wr_en_out(wb_inst0_wr_en_out)
    );

    // Write Back Stage

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
