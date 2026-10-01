`default_nettype none
module csr_file (
    input wire clk,
    input wire rst,
    input wire [11:0] csr_addr,
    input wire [31:0] write_data,
    input wire write_enable,
    input wire read_enable,
    output reg [31:0] read_data,
    output wire csr_valid,

    // Add interrupt handling ports
    input wire interrupt_pending,
    input wire [31:0] interrupt_cause_in,
    input wire [31:0] interrupt_pc_in,
    input wire [31:0] exception_pc_in,
    input wire interrupt_taken,
    input wire mret_instruction,
    input wire sret_instruction,
    input wire interrupt_to_supervisor,
    input wire trap_to_supervisor,
    input wire ecall_exception,
    input wire ebreak_exception,
    input wire illegal_instruction_exception,
    input wire instruction_address_misaligned_exception,
    input wire load_address_misaligned_exception,
    input wire store_address_misaligned_exception,
    input wire instr_page_fault_exception,
    input wire load_page_fault_exception,
    input wire store_page_fault_exception,
    input wire breakpoint_trigger_exception,
    input wire [31:0] exception_tval_in,
    input wire instret_increment,
    
    // Trigger state for EX: 4 x {m, 0, s, u, execute, store, load}, then the four tdata2 values.
    output wire [27:0] trigger_control,
    output wire [127:0] trigger_tdata2,

    // Architectural state read directly by the core (trap routing, MMU control)
    output wire [31:0] state_mstatus,
    output wire [31:0] state_medeleg,
    output wire [31:0] state_mideleg,
    output wire [31:0] state_mie,
    output wire [31:0] state_mip,
    output wire [31:0] state_mtvec,
    output wire [31:0] state_mepc,
    output wire [31:0] state_mcounteren,
    output wire [31:0] state_stvec,
    output wire [31:0] state_sepc,
    output wire [31:0] state_scounteren,
    output wire [31:0] state_satp,
    output wire [1:0] state_privilege_mode,

    // Vector state. vsetvl retires in EX; vector instructions complete (or fault) in MEM.
    input wire vset_we,
    input wire [31:0] vset_vl,
    input wire [31:0] vset_vtype,
    input wire vec_retire,               // a vector instruction finished: vstart = 0, VS dirty
    input wire vec_fault_we,             // a vector access faulted at element vec_fault_vstart
    input wire [31:0] vec_fault_vstart,
    input wire vec_vl_we,                // fault-only-first trimmed vl
    input wire [31:0] vec_vl_new,
    input wire vxsat_set,
    input wire [4:0] fflags_set,         // accrued FP exception flags from the coprocessor
    input wire fp_retire,                // an FP instruction finished: FS dirty
    output wire [2:0] state_frm,
    output wire [1:0] state_fs,
    output wire [31:0] state_vl,
    output wire [31:0] state_vtype,
    output wire [31:0] state_vstart,
    output wire [1:0] state_vxrm,
    output wire [1:0] state_vs,

    // Timer interrupt input
    input wire timer_interrupt,
    input wire software_interrupt,
    input wire external_interrupt
);

    // Common CSR addresses
    localparam CSR_MSTATUS   = 12'h300;
    localparam CSR_MISA      = 12'h301;
    localparam CSR_MEDELEG   = 12'h302;
    localparam CSR_MIDELEG   = 12'h303;
    localparam CSR_MIE       = 12'h304;
    localparam CSR_MTVEC     = 12'h305;
    localparam CSR_MCOUNTEREN = 12'h306;
    localparam CSR_MSCRATCH  = 12'h340;
    localparam CSR_MEPC      = 12'h341;
    localparam CSR_MCAUSE    = 12'h342;
    localparam CSR_MTVAL     = 12'h343;
    localparam CSR_MIP       = 12'h344;
    localparam CSR_MCOUNTINHIBIT = 12'h320;
    localparam CSR_SSTATUS   = 12'h100;
    localparam CSR_SIE       = 12'h104;
    localparam CSR_STVEC     = 12'h105;
    localparam CSR_SCOUNTEREN = 12'h106;
    localparam CSR_SSCRATCH  = 12'h140;
    localparam CSR_SEPC      = 12'h141;
    localparam CSR_SCAUSE    = 12'h142;
    localparam CSR_STVAL     = 12'h143;
    localparam CSR_SIP       = 12'h144;
    localparam CSR_SATP      = 12'h180;
    localparam CSR_PMPCFG0   = 12'h3A0;
    localparam CSR_PMPADDR0  = 12'h3B0;
    // Machine-mode writable counter CSRs
    localparam CSR_MCYCLE    = 12'hB00;
    localparam CSR_MINSTRET  = 12'hB02;
    localparam CSR_MCYCLEH   = 12'hB80;
    localparam CSR_MINSTRETH = 12'hB82;
    // Unprivileged read-only counter mirrors
    localparam CSR_CYCLE     = 12'hC00;
    localparam CSR_TIME      = 12'hC01;
    localparam CSR_INSTRET   = 12'hC02;
    localparam CSR_CYCLEH    = 12'hC80;
    localparam CSR_TIMEH     = 12'hC81;
    localparam CSR_INSTRETH  = 12'hC82;
    // Trigger module CSRs (Sdtrig: 4 mcontrol type-2 triggers)
    localparam CSR_TSELECT   = 12'h7A0;
    localparam CSR_TDATA1    = 12'h7A1;
    localparam CSR_TDATA2    = 12'h7A2;
    localparam CSR_TDATA3    = 12'h7A3;
    // Read-only machine info CSRs (addr[11:10]=2'b11 → any write is illegal)
    localparam CSR_MVENDORID = 12'hF11;
    localparam CSR_MARCHID   = 12'hF12;
    localparam CSR_MIMPID    = 12'hF13;
    localparam CSR_MHARTID   = 12'hF14;
    // AIA CSRs — return 0 so probe sequences don't fault
    localparam CSR_MTOPI     = 12'hFB0;
    localparam CSR_SCOVTOVF  = 12'hDA0;
    // RV32 / Sme / Smstateen / Smepmp extensions — read-zero, write-ignore
    localparam CSR_MSTATUSH  = 12'h310;
    localparam CSR_MENVCFG   = 12'h30A;
    localparam CSR_MENVCFGH  = 12'h31A;
    localparam CSR_MSECCFG   = 12'h747;
    localparam CSR_MSECCFGH  = 12'h757;
    localparam CSR_MCONFIGPTR = 12'hF15;
    // Vector CSRs
    localparam CSR_FFLAGS    = 12'h001;
    localparam CSR_FRM       = 12'h002;
    localparam CSR_FCSR      = 12'h003;
    localparam CSR_VSTART    = 12'h008;
    localparam CSR_VXSAT     = 12'h009;
    localparam CSR_VXRM      = 12'h00A;
    localparam CSR_VCSR      = 12'h00F;
    localparam CSR_VL        = 12'hC20;
    localparam CSR_VTYPE     = 12'hC21;
    localparam CSR_VLENB     = 12'hC22;
    // mhpmcounter3-31: 0xB03-0xB1F; high halves mhpmcounterh3-31: 0xB83-0xB9F
    // mhpmevent3-31:   0x323-0x33F
    // (handled via range checks in csr_valid)

    localparam PRIV_U = 2'b00;
    localparam PRIV_S = 2'b01;
    localparam PRIV_M = 2'b11;
    localparam SSTATUS_MASK = 32'h000C6722;   // + VS[10:9], FS[14:13]
    localparam S_INTERRUPT_MASK = 32'h00000222;
    // Writable mstatus bits: 1(SIE),3(MIE),5(SPIE),7(MPIE),8(SPP),
    // 11:12(MPP),17(MPRV),18(SUM),19(MXR),20(TVM),21(TW),22(TSR)
    // 9:10(VS)
    localparam MSTATUS_WRITABLE_MASK = 32'h007E7FAA;   // + FS[14:13]
    localparam SUPPORTED_MISA = 32'h40141129;  // RV32IMAFDSU
    localparam MCOUNTINHIBIT_MASK = 32'h00000005;
    localparam COUNTEREN_MASK = 32'h00000007;

    // CSR registers
    reg [31:0] mstatus;
    reg [31:0] misa;
    reg [31:0] medeleg;
    reg [31:0] mideleg;
    reg [31:0] mie;
    reg [31:0] mtvec;
    reg [31:0] mcounteren;
    reg [31:0] mscratch;
    reg [31:0] mepc;
    reg [31:0] mcause;
    reg [31:0] mtval;
    reg [31:0] mip;
    reg [31:0] mcountinhibit;
    reg [63:0] cycle_counter;
    reg [63:0] instret_counter;
    reg [1:0] privilege_mode;
    reg [31:0] stvec;
    reg [31:0] scounteren;
    reg [31:0] sscratch;
    reg [31:0] sepc;
    reg [31:0] scause;
    reg [31:0] stval;
    reg [31:0] satp;
    reg [31:0] pmpcfg0;
    reg [31:0] pmpaddr0;
    // Sdtrig state: tdata1 holds only the supported fields, so a write of 0 disables the trigger.
    reg [1:0] tselect;
    reg [6:0] trigger_ctl [0:3];   // {m, 0, s, u, execute, store, load}
    reg [31:0] trigger_addr [0:3]; // tdata2
    reg ssip_software_pending;
    reg stip_software_pending;
    reg seip_software_pending;
    reg [31:0] vl;
    reg [31:0] vtype;
    reg [31:0] vstart;
    reg vxsat;
    reg [1:0] vxrm;
    reg [4:0] fflags;
    reg [2:0] frm;

    localparam [3:0] TDATA1_TYPE_MCONTROL = 4'd2;
    wire [31:0] tdata1_read = {TDATA1_TYPE_MCONTROL, 21'b0, trigger_ctl[tselect]};
    assign trigger_control = {trigger_ctl[3], trigger_ctl[2], trigger_ctl[1], trigger_ctl[0]};
    assign trigger_tdata2 = {trigger_addr[3], trigger_addr[2], trigger_addr[1], trigger_addr[0]};

    // SD summarizes a dirty extension state (VS = 3).
    wire [31:0] mstatus_read = {(mstatus[10:9] == 2'b11) || (mstatus[14:13] == 2'b11), mstatus[30:0]};
    wire [31:0] sstatus = mstatus_read & (SSTATUS_MASK | 32'h80000000);
    wire vs_off = (mstatus[10:9] == 2'b00);
    wire fs_off = (mstatus[14:13] == 2'b00);
    wire is_fp_csr = (csr_addr == CSR_FFLAGS) || (csr_addr == CSR_FRM) || (csr_addr == CSR_FCSR);
    wire is_vec_csr = (csr_addr == CSR_VSTART) || (csr_addr == CSR_VXSAT) || (csr_addr == CSR_VXRM) ||
                      (csr_addr == CSR_VCSR) || (csr_addr == CSR_VL) || (csr_addr == CSR_VTYPE) ||
                      (csr_addr == CSR_VLENB);
    wire [31:0] sie = mie & S_INTERRUPT_MASK;
    wire [31:0] sip = mip & S_INTERRUPT_MASK;
    wire cycle_enabled = !mcountinhibit[0];
    wire instret_enabled = !mcountinhibit[2];
    wire writes_mcycle = write_enable && ((csr_addr == CSR_MCYCLE) || (csr_addr == CSR_MCYCLEH));
    wire writes_minstret = write_enable && ((csr_addr == CSR_MINSTRET) || (csr_addr == CSR_MINSTRETH));
    wire synchronous_exception = ecall_exception || ebreak_exception ||
                                 illegal_instruction_exception ||
                                 instruction_address_misaligned_exception ||
                                 load_address_misaligned_exception ||
                                 store_address_misaligned_exception ||
                                 instr_page_fault_exception ||
                                 load_page_fault_exception ||
                                 store_page_fault_exception ||
                                 breakpoint_trigger_exception;
    wire [31:0] ecall_cause =
        (privilege_mode == PRIV_U) ? 32'h00000008 :
        (privilege_mode == PRIV_S) ? 32'h00000009 :
                                     32'h0000000B;
    // MEM-stage faults are older than IF/EX-stage exceptions and must win if
    // more than one pipeline stage requests a trap in the same cycle.
    wire page_fault_exception = instr_page_fault_exception ||
                                load_page_fault_exception ||
                                store_page_fault_exception;
    wire [31:0] exception_cause =
        load_page_fault_exception                 ? 32'h0000000D :
        store_page_fault_exception                ? 32'h0000000F :
        instr_page_fault_exception                ? 32'h0000000C :
        instruction_address_misaligned_exception ? 32'h00000000 :
        breakpoint_trigger_exception              ? 32'h00000003 :
        illegal_instruction_exception             ? 32'h00000002 :
        ebreak_exception                          ? 32'h00000003 :
        load_address_misaligned_exception         ? 32'h00000004 :
        store_address_misaligned_exception        ? 32'h00000006 :
        instr_page_fault_exception                ? 32'h0000000C :
        load_page_fault_exception                 ? 32'h0000000D :
        store_page_fault_exception                ? 32'h0000000F :
                                                    ecall_cause;
    wire [31:0] exception_tval = (!page_fault_exception &&
                                  (ecall_exception || ebreak_exception)) ?
                                 32'h00000000 : exception_tval_in;

    // Check if CSR address is valid
    assign csr_valid = (is_vec_csr && !vs_off) || (is_fp_csr && !fs_off) ||
                       (csr_addr == CSR_MSTATUS) || (csr_addr == CSR_MISA) ||
                       (csr_addr == CSR_MEDELEG) || (csr_addr == CSR_MIDELEG) ||
                       (csr_addr == CSR_MIE) || (csr_addr == CSR_MTVEC) ||
                       (csr_addr == CSR_MCOUNTEREN) ||
                       (csr_addr == CSR_MSCRATCH) || (csr_addr == CSR_MEPC) ||
                       (csr_addr == CSR_MCAUSE) || (csr_addr == CSR_MTVAL) ||
                       (csr_addr == CSR_MCOUNTINHIBIT) ||
                       (csr_addr == CSR_SSTATUS) || (csr_addr == CSR_SIE) ||
                       (csr_addr == CSR_STVEC) || (csr_addr == CSR_SCOUNTEREN) ||
                       (csr_addr == CSR_SSCRATCH) || (csr_addr == CSR_SEPC) ||
                       (csr_addr == CSR_SCAUSE) || (csr_addr == CSR_STVAL) ||
                       (csr_addr == CSR_SIP) || (csr_addr == CSR_SATP) ||
                       (csr_addr == CSR_MIP) ||
                       (csr_addr == CSR_PMPCFG0) || (csr_addr == CSR_PMPADDR0) ||
                       (csr_addr == CSR_MCYCLE) || (csr_addr == CSR_MINSTRET) ||
                       (csr_addr == CSR_MCYCLEH) || (csr_addr == CSR_MINSTRETH) ||
                       (csr_addr == CSR_CYCLE) ||
                       (csr_addr == CSR_TIME) || (csr_addr == CSR_INSTRET) ||
                       (csr_addr == CSR_CYCLEH) || (csr_addr == CSR_TIMEH) ||
                       (csr_addr == CSR_INSTRETH) ||
                       (csr_addr == CSR_TSELECT) || (csr_addr == CSR_TDATA1) ||
                       (csr_addr == CSR_TDATA2) || (csr_addr == CSR_TDATA3) ||
                       (csr_addr == CSR_MVENDORID) || (csr_addr == CSR_MARCHID) ||
                       (csr_addr == CSR_MIMPID) || (csr_addr == CSR_MHARTID) ||
                       (csr_addr == CSR_MSTATUSH) || (csr_addr == CSR_MENVCFG) ||
                       (csr_addr == CSR_MENVCFGH) || (csr_addr == CSR_MSECCFG) ||
                       (csr_addr == CSR_MSECCFGH) || (csr_addr == CSR_MCONFIGPTR) ||
                       (csr_addr >= 12'hB03 && csr_addr <= 12'hB1F) ||  // mhpmcounter3-31
                       (csr_addr >= 12'hB83 && csr_addr <= 12'hB9F) ||  // mhpmcounterh3-31
                       (csr_addr >= 12'h323 && csr_addr <= 12'h33F);    // mhpmevent3-31

    // Initialize CSRs
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            mstatus <= 32'h00001800;  // MPP=11 (machine mode)
            misa <= SUPPORTED_MISA;
            medeleg <= 32'h0;
            mideleg <= 32'h0;
            mie <= 32'h0;
            mtvec <= 32'h0;
            mcounteren <= 32'h0;
            mscratch <= 32'h0;
            mepc <= 32'h0;
            mcause <= 32'h0;
            mtval <= 32'h0;
            mip <= 32'h0;
            mcountinhibit <= 32'h0;
            cycle_counter <= 64'h0;
            instret_counter <= 64'h0;
            privilege_mode <= PRIV_M;
            stvec <= 32'h0;
            scounteren <= 32'h0;
            sscratch <= 32'h0;
            sepc <= 32'h0;
            scause <= 32'h0;
            stval <= 32'h0;
            satp <= 32'h0;
            pmpcfg0 <= 32'h0;
            pmpaddr0 <= 32'h0;
            tselect <= 2'b0;
            trigger_ctl[0] <= 7'b0;
            trigger_ctl[1] <= 7'b0;
            trigger_ctl[2] <= 7'b0;
            trigger_ctl[3] <= 7'b0;
            trigger_addr[0] <= 32'h0;
            trigger_addr[1] <= 32'h0;
            trigger_addr[2] <= 32'h0;
            trigger_addr[3] <= 32'h0;
            ssip_software_pending <= 1'b0;
            stip_software_pending <= 1'b0;
            seip_software_pending <= 1'b0;
            vl <= 32'h0;
            vtype <= 32'h80000000;   // vill until the first vsetvl
            vstart <= 32'h0;
            vxsat <= 1'b0;
            vxrm <= 2'b0;
            fflags <= 5'b0;
            frm <= 3'b0;
        end else begin
            // MEM-stage vector events first: the EX-stage instruction is younger and wins below.
            if (vec_retire) begin
                vstart <= 32'h0;
                mstatus[10:9] <= 2'b11;
            end
            if (vec_fault_we) vstart <= vec_fault_vstart;
            if (vec_vl_we) vl <= vec_vl_new;
            if (vxsat_set) vxsat <= 1'b1;
            if (fflags_set != 5'b0) begin
                fflags <= fflags | fflags_set;
                mstatus[14:13] <= 2'b11;
            end
            if (fp_retire) mstatus[14:13] <= 2'b11;
            if (vset_we) begin
                vl <= vset_vl;
                vtype <= vset_vtype;
                vstart <= 32'h0;
                mstatus[10:9] <= 2'b11;
            end
            if (cycle_enabled && !writes_mcycle) begin
                cycle_counter <= cycle_counter + 64'h1;
            end

            if (instret_increment && instret_enabled && !writes_minstret) begin
                instret_counter <= instret_counter + 64'h1;
            end
            
            // Update MIP/SIP view based on interrupt inputs and delegation.
            // Delegated interrupts are surfaced through the supervisor-pending
            // bits so S-mode sees the expected cause codes.
            mip[1] <= (mideleg[1] ? software_interrupt : 1'b0) |
                      ssip_software_pending;                    // SSIP
            mip[3] <= mideleg[1] ? 1'b0 : software_interrupt;   // MSIP
            mip[5] <= (mideleg[5] ? timer_interrupt : 1'b0) |
                      stip_software_pending;                    // STIP
            mip[7] <= mideleg[5] ? 1'b0 : timer_interrupt;      // MTIP
            mip[9] <= (mideleg[9] ? external_interrupt : 1'b0) |
                      seip_software_pending;                    // SEIP
            mip[11] <= mideleg[9] ? 1'b0 : external_interrupt;  // MEIP
            
            // Handle interrupt entry
            if (interrupt_taken) begin
                if (interrupt_to_supervisor) begin
                    sepc <= interrupt_pc_in;       // Save interrupted PC
                    scause <= interrupt_cause_in;  // Save interrupt cause
                    mstatus[5] <= mstatus[1];      // Save SIE to SPIE
                    mstatus[1] <= 1'b0;            // Disable supervisor interrupts
                    mstatus[8] <= (privilege_mode == PRIV_S);
                    privilege_mode <= PRIV_S;      // Trap to supervisor mode
                end else begin
                    mepc <= interrupt_pc_in;       // Save current PC
                    mcause <= interrupt_cause_in;  // Save interrupt cause
                    mstatus[7] <= mstatus[3];      // Save MIE to MPIE
                    mstatus[3] <= 1'b0;            // Disable interrupts
                    mstatus[12:11] <= privilege_mode; // Save previous privilege in MPP
                    privilege_mode <= PRIV_M;      // Trap to machine mode
                end
            end
            
            // Handle MRET
            else if (mret_instruction) begin
                mstatus[3] <= mstatus[7];        // Restore MIE from MPIE
                mstatus[7] <= 1'b1;              // Set MPIE to 1
                privilege_mode <= mstatus[12:11]; // Return to privilege encoded in MPP
                mstatus[12:11] <= PRIV_U;        // Clear MPP after return
                // MPRV is cleared when MRET returns to a mode less
                // privileged than M (privileged spec §3.1.6.5).
                if (mstatus[12:11] != PRIV_M) begin
                    mstatus[17] <= 1'b0;
                end
            end

            // Handle SRET
            else if (sret_instruction) begin
                mstatus[1] <= mstatus[5];        // Restore SIE from SPIE
                mstatus[5] <= 1'b1;              // Set SPIE to 1
                privilege_mode <= mstatus[8] ? PRIV_S : PRIV_U;
                mstatus[8] <= 1'b0;              // Clear SPP after return
            end
            
            // Handle synchronous exceptions
            else if (synchronous_exception) begin
                if (trap_to_supervisor) begin
                    sepc <= exception_pc_in;        // Save exception PC
                    scause <= exception_cause;
                    stval <= exception_tval;
                    mstatus[5] <= mstatus[1];      // Save SIE to SPIE
                    mstatus[1] <= 1'b0;            // Disable supervisor interrupts
                    mstatus[8] <= (privilege_mode == PRIV_S);
                    privilege_mode <= PRIV_S;      // Trap to supervisor mode
                end else begin
                    mepc <= exception_pc_in;        // Save exception PC
                    mcause <= exception_cause;
                    mtval <= exception_tval;
                    mstatus[7] <= mstatus[3];      // Save MIE to MPIE
                    mstatus[3] <= 1'b0;            // Disable interrupts
                    mstatus[12:11] <= privilege_mode; // Save previous privilege in MPP
                    privilege_mode <= PRIV_M;      // Trap to machine mode
                end
            end
            
            // Normal CSR writes
            else if (write_enable && csr_valid) begin
                case (csr_addr)
                    CSR_MSTATUS:  mstatus <= write_data & MSTATUS_WRITABLE_MASK;
                    CSR_SSTATUS:  mstatus <= (mstatus & ~SSTATUS_MASK) | (write_data & SSTATUS_MASK);
                    CSR_MISA:     misa <= (write_data & SUPPORTED_MISA) | 32'h40000000;
                    CSR_MEDELEG:  medeleg <= write_data;
                    CSR_MIDELEG:  mideleg <= write_data;
                    CSR_MIE:      mie <= write_data;
                    CSR_SIE:      mie <= (mie & ~S_INTERRUPT_MASK) | (write_data & S_INTERRUPT_MASK);
                    // MODE is WARL: clearing bit 1 maps reserved 2 and 3 onto 0 and 1 (privileged spec 3.1.7).
                    CSR_MTVEC:    mtvec <= {write_data[31:2], 1'b0, write_data[0]};
                    CSR_MCOUNTEREN: mcounteren <= write_data & COUNTEREN_MASK;
                    CSR_MSCRATCH: mscratch <= write_data;
                    CSR_MEPC:     mepc <= write_data;
                    CSR_MCAUSE:   mcause <= write_data;
                    CSR_MTVAL:    mtval <= write_data;
                    CSR_MCOUNTINHIBIT: mcountinhibit <= write_data & MCOUNTINHIBIT_MASK;
                    CSR_STVEC:    stvec <= {write_data[31:2], 1'b0, write_data[0]};
                    CSR_SCOUNTEREN: scounteren <= write_data & COUNTEREN_MASK;
                    CSR_SSCRATCH: sscratch <= write_data;
                    CSR_SEPC:     sepc <= write_data;
                    CSR_SCAUSE:   scause <= write_data;
                    CSR_STVAL:    stval <= write_data;
                    CSR_SATP:     satp <= write_data & 32'h803FFFFF;  // ASID reads as zero
                    CSR_SIP: begin
                        ssip_software_pending <= write_data[1];
                        stip_software_pending <= write_data[5];
                        seip_software_pending <= write_data[9];
                    end
                    // Keep hardware pending bits driven by the platform, but
                    // allow software to synthesize/clear supervisor-visible
                    // pending bits for focused trap tests and SBI flows.
                    CSR_MIP: begin
                        ssip_software_pending <= write_data[1];
                        stip_software_pending <= write_data[5];
                        seip_software_pending <= write_data[9];
                    end
                    // Only 4 triggers, so tselect is 2 bits wide.
                    CSR_TSELECT:  tselect <= write_data[1:0];
                    // Supported mcontrol fields only; type stays 2 and the rest read back 0.
                    CSR_TDATA1:   trigger_ctl[tselect] <= {write_data[6], 1'b0, write_data[4],
                                                           write_data[3], write_data[2],
                                                           write_data[1], write_data[0]};
                    CSR_TDATA2:   trigger_addr[tselect] <= write_data;
                    CSR_PMPCFG0:  pmpcfg0 <= write_data;
                    CSR_PMPADDR0: pmpaddr0 <= write_data;
                    CSR_MCYCLE:   cycle_counter[31:0] <= write_data;
                    CSR_MCYCLEH:  cycle_counter[63:32] <= write_data;
                    CSR_MINSTRET: instret_counter[31:0] <= write_data;
                    CSR_MINSTRETH: instret_counter[63:32] <= write_data;
                    CSR_FFLAGS:   begin fflags <= write_data[4:0]; mstatus[14:13] <= 2'b11; end
                    CSR_FRM:      begin frm <= write_data[2:0]; mstatus[14:13] <= 2'b11; end
                    CSR_FCSR:     begin fflags <= write_data[4:0]; frm <= write_data[7:5]; mstatus[14:13] <= 2'b11; end
                    CSR_VSTART:   begin vstart <= {23'b0, write_data[8:0]}; mstatus[10:9] <= 2'b11; end
                    CSR_VXSAT:    begin vxsat <= write_data[0]; mstatus[10:9] <= 2'b11; end
                    CSR_VXRM:     begin vxrm <= write_data[1:0]; mstatus[10:9] <= 2'b11; end
                    CSR_VCSR:     begin vxsat <= write_data[0]; vxrm <= write_data[2:1]; mstatus[10:9] <= 2'b11; end
                    default: ;
                endcase
            end
        end
    end

    // Read logic
    always @(*) begin
        if (read_enable && csr_valid) begin
            case (csr_addr)
                CSR_MSTATUS:  read_data = mstatus_read;
                CSR_SSTATUS:  read_data = sstatus;
                CSR_MISA:     read_data = misa;
                CSR_MEDELEG:  read_data = medeleg;
                CSR_MIDELEG:  read_data = mideleg;
                CSR_MIE:      read_data = mie;
                CSR_SIE:      read_data = sie;
                CSR_MTVEC:    read_data = mtvec;
                CSR_MCOUNTEREN: read_data = mcounteren;
                CSR_MSCRATCH: read_data = mscratch;
                CSR_MEPC:     read_data = mepc;
                CSR_MCAUSE:   read_data = mcause;
                CSR_MTVAL:    read_data = mtval;
                CSR_MIP:      read_data = mip;
                CSR_MCOUNTINHIBIT: read_data = mcountinhibit;
                CSR_STVEC:    read_data = stvec;
                CSR_SCOUNTEREN: read_data = scounteren;
                CSR_SSCRATCH: read_data = sscratch;
                CSR_SEPC:     read_data = sepc;
                CSR_SCAUSE:   read_data = scause;
                CSR_STVAL:    read_data = stval;
                CSR_SATP:     read_data = satp;
                CSR_SIP:      read_data = sip;
                CSR_MCYCLE:   read_data = cycle_counter[31:0];
                CSR_MINSTRET: read_data = instret_counter[31:0];
                CSR_MCYCLEH:  read_data = cycle_counter[63:32];
                CSR_MINSTRETH: read_data = instret_counter[63:32];
                CSR_CYCLE:    read_data = cycle_counter[31:0];
                CSR_TIME:     read_data = cycle_counter[31:0];
                CSR_INSTRET:  read_data = instret_counter[31:0];
                CSR_CYCLEH:   read_data = cycle_counter[63:32];
                CSR_TIMEH:    read_data = cycle_counter[63:32];
                CSR_INSTRETH: read_data = instret_counter[63:32];
                CSR_PMPCFG0:  read_data = 32'h0;
                CSR_PMPADDR0: read_data = 32'h0;
                CSR_TSELECT:  read_data = {30'b0, tselect};
                CSR_TDATA1:   read_data = tdata1_read;
                CSR_TDATA2:   read_data = trigger_addr[tselect];
                CSR_TDATA3:   read_data = 32'h0;
                CSR_MVENDORID: read_data = 32'h0;
                CSR_MARCHID:   read_data = 32'h0;
                CSR_MIMPID:    read_data = 32'h0;
                CSR_MHARTID:   read_data = 32'h0;
                CSR_MTOPI:     read_data = 32'h0;
                CSR_SCOVTOVF:  read_data = 32'h0;
                CSR_FFLAGS:    read_data = {27'b0, fflags};
                CSR_FRM:       read_data = {29'b0, frm};
                CSR_FCSR:      read_data = {24'b0, frm, fflags};
                CSR_VSTART:    read_data = vstart;
                CSR_VXSAT:     read_data = {31'b0, vxsat};
                CSR_VXRM:      read_data = {30'b0, vxrm};
                CSR_VCSR:      read_data = {29'b0, vxrm, vxsat};
                CSR_VL:        read_data = vl;
                CSR_VTYPE:     read_data = vtype;
                CSR_VLENB:     read_data = 32'd64;
                default:      read_data = 32'h0;
            endcase
        end else begin
            read_data = 32'h0;
        end
    end


    // Architectural state exported to the core
    assign state_mstatus = mstatus;
    assign state_medeleg = medeleg;
    assign state_mideleg = mideleg;
    assign state_mie = mie;
    assign state_mip = mip;
    assign state_mtvec = mtvec;
    assign state_mepc = mepc;
    assign state_mcounteren = mcounteren;
    assign state_stvec = stvec;
    assign state_sepc = sepc;
    assign state_scounteren = scounteren;
    assign state_satp = satp;
    assign state_privilege_mode = privilege_mode;
    assign state_vl = vl;
    assign state_vtype = vtype;
    assign state_vstart = vstart;
    assign state_vxrm = vxrm;
    assign state_vs = mstatus[10:9];
    assign state_frm = frm;
    assign state_fs = mstatus[14:13];

endmodule
