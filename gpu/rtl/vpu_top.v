`default_nettype none
// TinyGPU v2: RVV vector unit for Synapse-32 (Zve32x + Zve32f subset, VLEN=512, ELEN=32).
//
// The CPU issues vector instructions from its MEM stage (in order, already past every trap
// point) into a 4-entry queue. This first version executes them through a single element
// sequencer: one element per cycle (multi-cycle for divides and memory). It is the
// correctness reference for the whole instruction set; the 16-lane datapath for the hot
// SEW=32 operations sits beside it (vpu_lanes) and must produce identical results.
//
// Vector memory accesses go through the core's data port (D-cache + D-TLB), so they are
// coherent with scalar code and translated with the same permissions. The CPU stays in
// MEM until a load/store finishes; a fault reports the element index for vstart.
module vpu_top #(
    parameter LANES = 4           // SIMD lanes for the SEW=32 fast path (4, 8 or 16)
) (
    input  wire        clk,
    input  wire        rst,

    // Issue (from the CPU MEM stage)
    input  wire        issue_valid,
    output wire        issue_ready,
    input  wire [31:0] issue_instr,
    input  wire [31:0] issue_rs1,
    input  wire [31:0] issue_rs2,
    input  wire [31:0] issue_vl,
    input  wire [31:0] issue_vtype,
    input  wire [31:0] issue_vstart,
    input  wire [1:0]  issue_vxrm,
    input  wire [2:0]  issue_frm,

    // Response for instructions the CPU waits on (scalar result, loads, stores)
    output reg         resp_valid,
    output reg  [31:0] resp_data,
    output reg         resp_fault,
    output reg         resp_fault_store,
    output reg  [31:0] resp_fault_addr,
    output reg  [31:0] resp_fault_vstart,
    output reg         resp_vl_valid,
    output reg  [31:0] resp_vl,

    output wire        busy,
    output reg         vxsat_set,
    output reg  [4:0]  fflags_set,      // accrued FP flags (pulses), to fcsr in the core

    // Data port (multiplexed onto the core's D-cache path while a vector access runs)
    output reg         mem_rd,
    output reg         mem_wr,
    output reg  [31:0] mem_addr,
    output wire [31:0] mem_addr_next,   // next cycle's address (D-cache early index)
    output wire        mem_own,         // the unit drives the data port (whole vector access)
    output reg  [31:0] mem_wdata,       // low-aligned
    output reg  [3:0]  mem_be,          // relative to mem_addr
    output reg  [2:0]  mem_load_type,
    input  wire        mem_rvalid,
    input  wire [31:0] mem_rdata,
    input  wire        mem_load_pf,
    input  wire        mem_store_pf
);
`include "vec_defines.vh"

    // ======================================================================
    // Issue queue
    // ======================================================================
    localparam QD = 4;
    reg [31:0] q_instr [0:QD-1];
    reg [31:0] q_rs1   [0:QD-1];
    reg [31:0] q_rs2   [0:QD-1];
    reg [31:0] q_vl    [0:QD-1];
    reg [31:0] q_vtype [0:QD-1];
    reg [31:0] q_vstart[0:QD-1];
    reg [1:0]  q_vxrm  [0:QD-1];
    reg [2:0]  q_frm   [0:QD-1];
    reg [1:0] q_head, q_tail;
    reg [2:0] q_count;
    assign issue_ready = (q_count != QD);
    wire q_push = issue_valid && issue_ready;
    reg  q_pop;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            q_head <= 2'd0;
            q_tail <= 2'd0;
            q_count <= 3'd0;
        end else begin
            if (q_push) begin
                q_instr[q_tail] <= issue_instr;
                q_rs1[q_tail] <= issue_rs1;
                q_rs2[q_tail] <= issue_rs2;
                q_vl[q_tail] <= issue_vl;
                q_vtype[q_tail] <= issue_vtype;
                q_vstart[q_tail] <= issue_vstart;
                q_vxrm[q_tail] <= issue_vxrm;
                q_frm[q_tail] <= issue_frm;
                q_tail <= q_tail + 2'd1;
            end
            if (q_pop) q_head <= q_head + 2'd1;
            q_count <= q_count + {2'b0, q_push} - {2'b0, q_pop};
        end
    end

    // ======================================================================
    // Vector register file: 16 banks (one per 32-bit word), three whole-register read ports
    // ======================================================================
    reg  [4:0]   rA, rB, rD;          // read port register numbers
    wire [511:0] vA, vB, vD;
    reg  [4:0]   w_reg;
    reg  [15:0]  w_en;
    reg  [511:0] w_data;
    wire [511:0] v0s;                 // v0 (mask) shadow
    vpu_vrf vrf (
        .clk(clk), .ra(rA), .rb(rB), .rd(rD), .va(vA), .vb(vB), .vd(vD),
        .wreg(w_reg), .wen(w_en), .wdata(w_data), .v0(v0s)
    );

    // Element of EEW 2^eew bytes at byte offset off of a 512-bit register, zero-extended
    function [31:0] ext;
        input [511:0] v;
        input [5:0] off;
        input [1:0] eew;
        reg [31:0] word;
        begin
            word = v[{off[5:2], 5'b0} +: 32];
            case (eew)
                2'd0: ext = {24'b0, word[{off[1:0], 3'b000} +: 8]};
                2'd1: ext = {16'b0, word[{off[1], 4'b0000} +: 16]};
                default: ext = word;
            endcase
        end
    endfunction

    // Register and byte offset of element idx (EEW 2^eew) in the group starting at base
    function [10:0] loc;
        input [4:0] base;
        input [9:0] idx;
        input [1:0] eew;
        reg [11:0] full;
        begin
            full = {1'b0, base, 6'b0} + ({2'b0, idx} << eew);
            loc = full[10:0];       // {reg[4:0], off[5:0]}
        end
    endfunction

    function [31:0] sx;  // sign-extend an element
        input [31:0] v;
        input [1:0] eew;
        begin
            case (eew)
                2'd0: sx = {{24{v[7]}}, v[7:0]};
                2'd1: sx = {{16{v[15]}}, v[15:0]};
                default: sx = v;
            endcase
        end
    endfunction

    function [31:0] zx;
        input [31:0] v;
        input [1:0] eew;
        begin
            case (eew)
                2'd0: zx = {24'b0, v[7:0]};
                2'd1: zx = {16'b0, v[15:0]};
                default: zx = v;
            endcase
        end
    endfunction

    // Fixed-point rounding increment for shifting v right by d (d < 64), per vxrm.
    function round_inc;
        input [63:0] v;
        input [5:0] d;
        input [1:0] rm;
        reg lsb, half, sticky;
        reg [63:0] below;
        begin
            if (d == 6'd0) round_inc = 1'b0;
            else begin
                lsb = v[d];
                half = v[d - 6'd1];
                below = (d >= 6'd2) ? (v & ((64'd1 << (d - 6'd1)) - 64'd1)) : 64'd0;
                sticky = (below != 64'd0);
                case (rm)
                    2'd0: round_inc = half;                       // rnu
                    2'd1: round_inc = half && (sticky || lsb);   // rne
                    2'd2: round_inc = 1'b0;                       // rdn
                    default: round_inc = !lsb && (half || sticky); // rod
                endcase
            end
        end
    endfunction

    // ======================================================================
    // Current instruction (head of queue)
    // ======================================================================
    wire [31:0] h_instr  = q_instr[q_head];
    wire [31:0] h_vtype  = q_vtype[q_head];
    wire        h_valid  = (q_count != 3'd0);

    wire       dec_is_vector, dec_legal, dec_is_load, dec_is_store, dec_scalar_dest;
    wire [4:0] dec_kind;
    wire [1:0] dec_eew_d, dec_eew_s2, dec_eew_s1;
    vec_decode vdec (
        .instr(h_instr), .vtype(h_vtype), .fp_enabled(1'b1), .frm(q_frm[q_head]),
        .is_vector(dec_is_vector), .legal(dec_legal), .kind(dec_kind),
        .is_load(dec_is_load), .is_store(dec_is_store), .scalar_dest(dec_scalar_dest),
        .uses_rs1(), .uses_rs2(), .uses_frs1(),
        .eew_d(dec_eew_d), .eew_s2(dec_eew_s2), .eew_s1(dec_eew_s1)
    );

    // Latched at start of execution
    reg [31:0] c_instr, c_rs1, c_rs2, c_vl, c_vtype, c_vstart;
    reg [1:0]  c_vxrm;
    reg [4:0]  c_kind;
    reg [1:0]  c_eew_d, c_eew_s2, c_eew_s1;
    reg        c_load, c_store, c_scalar;

    wire [5:0] c_funct6 = c_instr[31:26];
    wire [2:0] c_funct3 = c_instr[14:12];
    wire       c_vm     = c_instr[25];
    wire [4:0] c_vs2    = c_instr[24:20];
    wire [4:0] c_vs1    = c_instr[19:15];
    wire [4:0] c_vd     = c_instr[11:7];
    wire [1:0] c_sew    = c_vtype[4:3];
    wire [2:0] c_vlmul  = c_vtype[2:0];
    wire [2:0] c_nf     = c_instr[31:29];
    wire [1:0] c_mop    = c_instr[27:26];
    wire       c_ff     = (c_instr[24:20] == 5'b10000) && (c_mop == 2'b00);

    // VLMAX for the current vtype (elements of SEW)
    wire [9:0] per_reg = 10'd64 >> c_sew;
    reg  [9:0] c_vlmax;
    always @(*) begin
        case (c_vlmul)
            3'b001: c_vlmax = per_reg << 1;
            3'b010: c_vlmax = per_reg << 2;
            3'b011: c_vlmax = per_reg << 3;
            3'b111: c_vlmax = per_reg >> 1;
            3'b110: c_vlmax = per_reg >> 2;
            default: c_vlmax = per_reg;
        endcase
    end

    // Registers per data group for a segment field (EMUL of the data EEW, at least 1)
    reg [3:0] c_field_regs;
    always @(*) begin : field_regs
        reg signed [3:0] l2;
        l2 = $signed({c_vlmul[2], c_vlmul}) + $signed({2'b00, c_eew_d}) - $signed({2'b00, c_sew});
        case (l2)
            4'sd1: c_field_regs = 4'd2;
            4'sd2: c_field_regs = 4'd4;
            4'sd3: c_field_regs = 4'd8;
            default: c_field_regs = 4'd1;
        endcase
    end

    // Scalar operand for .vx/.vi forms
    wire [31:0] simm = {{27{c_vs1[4]}}, c_vs1};
    wire [31:0] uimm = {27'b0, c_vs1};
    wire is_vx = (c_funct3 == F3_OPIVX) || (c_funct3 == F3_OPMVX) || (c_funct3 == F3_OPFVF);
    wire is_vi = (c_funct3 == F3_OPIVI);
    wire is_vv = !is_vx && !is_vi;

    // ======================================================================
    // Scalar FP registers (RV32F/D): 32 x 64-bit, singles NaN-boxed
    // ======================================================================
    // Two 32 x 32 LUT-RAM halves (FLD writes them separately), one write port, three read
    // addresses: rs1 (also the .vf operand at dispatch), rs2 (also FSW/FSD data), rs3.
    reg [31:0] freg_lo [0:31];
    reg [31:0] freg_hi [0:31];
    integer fi;
    initial for (fi = 0; fi < 32; fi = fi + 1) begin freg_lo[fi] = 32'h0; freg_hi[fi] = 32'h0; end
    reg        fw_lo, fw_hi;
    reg [4:0]  fw_a;
    reg [63:0] fw_d;
    always @(posedge clk) begin
        if (fw_lo) freg_lo[fw_a] <= fw_d[31:0];
        if (fw_hi) freg_hi[fw_a] <= fw_d[63:32];
    end
    function [31:0] unbox;            // a single read from a register that is not boxed is the canonical NaN
        input [63:0] v;
        begin
            unbox = (v[63:32] == 32'hFFFFFFFF) ? v[31:0] : 32'h7FC00000;
        end
    endfunction
    wire [4:0] c_rs3 = c_instr[31:27];
    wire c_is_sfp_mem = (c_kind == VK_SFLD) || (c_kind == VK_SFST);
    wire c_dbl = c_is_sfp_mem ? (c_instr[14:12] == 3'b011) : (c_instr[26:25] == 2'b01);
    wire [2:0] c_srm = (c_instr[14:12] == 3'b111) ? c_frm : c_instr[14:12];
    wire [4:0]  sf1_a;                // c_vs1, or the head instruction's rs1 while dispatching
    wire [63:0] sf1 = {freg_hi[sf1_a], freg_lo[sf1_a]};     // rs1
    wire [63:0] sf2 = {freg_hi[c_vs2], freg_lo[c_vs2]};     // rs2
    wire [63:0] sf3 = {freg_hi[c_rs3], freg_lo[c_rs3]};     // rs3

    // ======================================================================
    // Sequencer state
    // ======================================================================
    localparam S_IDLE = 4'd0, S_RUN = 4'd1, S_DIV = 4'd2, S_MEM = 4'd3, S_DONE = 4'd4,
               S_FAST = 4'd5, S_DRAIN = 4'd6, S_SFP = 4'd7, S_SFDIV = 4'd8;
    reg [3:0]  state;
    reg [9:0]  e;          // element index
    reg [2:0]  fld;        // segment field
    reg [9:0]  evl;        // effective vl (elements processed: [vstart, evl))
    reg [31:0] acc;        // reductions, vcpop, vfirst, viota/compress counters
    reg        found;      // vfirst / vmsbf family
    reg        fault_hit;

    assign busy = h_valid || (state != S_IDLE);

    // Element index of the active mask bit
    wire mask_on = c_vm || v0s[e[8:0]];
    // Element offset for slides (vx/vi), saturating at 1023
    wire [31:0] slide_amt = is_vi ? uimm : c_rs1;
    wire [9:0]  slide_off = (slide_amt > 32'd1023) ? 10'd1023 : slide_amt[9:0];

    // ---------------- Operands of element e (through the three read ports) ----------------
    reg  [5:0]  offA, offB, offD;
    reg  [1:0]  eewA, eewB, eewD;
    wire [31:0] a_elem = ext(vA, offA, eewA);
    wire [31:0] b_elem = ext(vB, offB, eewB);
    wire [31:0] d_elem = ext(vD, offD, eewD);

    // ---------------- Element timing ----------------
    // Each element of the sequencer (S_RUN) and each scalar FP op (S_SFP) runs in phases:
    //   ph 0      fetch: the element's operands (and mask bits) are registered (mcx_*)
    //   ph 1      gathers only: refetch vs2 at the index just registered
    //   ph 2..4   compute, combinationally from mcx_* (and the instruction registers)
    //   end ph 4  capture: results, write decisions and flags are registered (mcy_*, mcz_*)
    //   ph 5      write back from the captured values; advance
    // The compute logic (FMA, multiplier, conversions, FP64) is therefore given three clock
    // cycles (four for scalar FP, which also uses ph 1) and is constrained as a multicycle path
    // from mcx_* / c_* / e / acc / v0 to mcy_* / mcz_* (see soc_timing.xdc). Nothing on those
    // paths may change between fetch and capture; the phase sequence guarantees it.
    localparam [2:0] PH_FETCH = 3'd0, PH_FETCH2 = 3'd1, PH_CAP = 3'd4, PH_WR = 3'd5;
    reg  [2:0]  ph;
    reg  [31:0] mcx_a, mcx_b, mcx_d;
    reg         mcx_ma, mcx_mb;          // mask bit e of the vs2 / vs1 registers
    reg  [63:0] mcx_s1, mcx_s2, mcx_s3;  // scalar FP operands
    reg  [31:0] red_next;                // reduction fold value (below)
    reg  [63:0] mcz_wd;                  // captured scalar FP results (below)
    reg  [31:0] mcz_resp;
    reg  [4:0]  mcz_fl;
    // vfredusum that saw no active element, with vs1[0] a NaN (found = some element was active)
    wire red_none_nan = (c_kind == VK_RED) && (c_funct3 == F3_OPFVV) && (c_funct6 == 6'b000001) &&
                        !found && !mask_on && (acc[30:23] == 8'hFF) && (acc[22:0] != 23'b0);
    wire [31:0] a_raw = mcx_a;
    wire [31:0] b_vec = mcx_b;
    wire [31:0] b_raw = is_vv ? b_vec : is_vi ? simm : c_rs1;
    wire [31:0] d_raw = mcx_d;

    // ======================================================================
    // Integer element operation (SEW / widening / narrowing)
    // ======================================================================
    reg [31:0] r_val;       // result element (EEW of destination)
    reg        r_bit;       // result mask bit
    reg        r_sat;       // saturation happened
    reg        r_write;     // element is written

    // Divider (shared with the scalar core's muldiv)
    wire div_op = (c_funct3 == F3_OPMVV || c_funct3 == F3_OPMVX) && (c_funct6[5:2] == 4'b1000) &&
                  (c_kind == VK_ELEM);
    reg  div_req;
    wire div_ready;
    wire [31:0] div_result;
    reg  [6:0] div_id;
    always @(*) begin
        case (c_funct6[1:0])
            2'b00: div_id = 7'h35;   // divu
            2'b01: div_id = 7'h34;   // div
            2'b10: div_id = 7'h37;   // remu
            default: div_id = 7'h36; // rem
        endcase
    end
    wire div_signed = c_funct6[0];
    muldiv vdivider (
        .clk(clk), .rst(rst),
        .req(div_req), .kill(1'b0), .advance(1'b1),
        .instr_id(div_id),
        .a(div_signed ? sx(a_raw, c_sew) : zx(a_raw, c_sew)),
        .b(div_signed ? sx(b_raw, c_sew) : zx(b_raw, c_sew)),
        .ready(div_ready), .result(div_result)
    );

    // One shared 33x33 signed multiplier for every integer multiply (vmul*, vmacc family,
    // widening multiplies, vsmul): operands are sign- or zero-extended to 33 bits per op.
    reg  signed [32:0] m_x, m_y;
    always @(*) begin : mul_operands
        reg [32:0] as33, au33, bs33, bu33, ds33;
        reg [31:0] t_as, t_bs, t_ds;
        t_as = sx(a_raw, c_eew_s2);
        t_bs = sx(b_raw, c_eew_s1);
        t_ds = sx(d_raw, c_eew_d);
        as33 = {t_as[31], t_as};
        au33 = {1'b0, zx(a_raw, c_eew_s2)};
        bs33 = {t_bs[31], t_bs};
        bu33 = {1'b0, zx(b_raw, c_eew_s1)};
        ds33 = {t_ds[31], t_ds};
        m_x = as33; m_y = bs33;                                   // vmul, vmulh, vwmul, vsmul
        case (c_funct6)
            6'b100100, 6'b111000: begin m_x = au33; m_y = bu33; end   // vmulhu, vwmulu
            6'b100110, 6'b111010: begin m_x = as33; m_y = bu33; end   // vmulhsu, vwmulsu
            6'b101001, 6'b101011: begin m_x = bs33; m_y = ds33; end   // vmadd, vnmsub
            6'b101101, 6'b101111: begin m_x = bs33; m_y = as33; end   // vmacc, vnmsac
            6'b111100: begin m_x = bu33; m_y = au33; end              // vwmaccu
            6'b111101: begin m_x = bs33; m_y = as33; end              // vwmacc
            6'b111110: begin m_x = bu33; m_y = as33; end              // vwmaccus
            6'b111111: begin m_x = bs33; m_y = au33; end              // vwmaccsu
            default: ;
        endcase
    end
    wire signed [65:0] m_prod = m_x * m_y;

    always @(*) begin : int_alu
        reg [63:0] as, au, bs, bu, ds, prod, wide, sum;
        reg [31:0] mask_sew;
        reg [5:0]  sh;
        reg signed [63:0] smax, smin;
        reg [63:0] umax;
        reg carry_in;
        reg [1:0] w;                 // EEW of the wide operand
        reg [5:0] sewbits;

        sewbits = 6'd8 << c_sew;
        mask_sew = (c_sew == 2'd2) ? 32'hFFFFFFFF : ((32'd1 << sewbits) - 32'd1);
        as = {32'b0, sx(a_raw, c_eew_s2)};
        as = {{32{as[31]}}, as[31:0]};
        au = {32'b0, zx(a_raw, c_eew_s2)};
        bs = {32'b0, sx(b_raw, c_eew_s1)};
        bs = {{32{bs[31]}}, bs[31:0]};
        bu = {32'b0, zx(b_raw, c_eew_s1)};
        ds = {32'b0, sx(d_raw, c_eew_d)};
        ds = {{32{ds[31]}}, ds[31:0]};
        smax = (64'sd1 <<< (sewbits - 6'd1)) - 64'sd1;
        smin = -(64'sd1 <<< (sewbits - 6'd1));
        umax = (64'd1 << sewbits) - 64'd1;
        carry_in = !c_vm && v0s[e[8:0]];
        sh = b_raw[5:0] & (sewbits - 6'd1);

        r_val = 32'b0;
        r_bit = 1'b0;
        r_sat = 1'b0;
        prod = 64'b0;
        wide = 64'b0;
        sum = 64'b0;
        w = 2'd0;

        if (c_funct3 == F3_OPIVV || c_funct3 == F3_OPIVX || c_funct3 == F3_OPIVI) begin
            case (c_funct6)
                6'b000000: r_val = as[31:0] + bs[31:0];                     // vadd
                6'b000010: r_val = as[31:0] - bs[31:0];                     // vsub
                6'b000011: r_val = bs[31:0] - as[31:0];                     // vrsub
                6'b000100: r_val = (au < bu) ? au[31:0] : bu[31:0];         // vminu
                6'b000101: r_val = ($signed(as) < $signed(bs)) ? as[31:0] : bs[31:0];
                6'b000110: r_val = (au > bu) ? au[31:0] : bu[31:0];         // vmaxu
                6'b000111: r_val = ($signed(as) > $signed(bs)) ? as[31:0] : bs[31:0];
                6'b001001: r_val = a_raw & b_raw;
                6'b001010: r_val = a_raw | b_raw;
                6'b001011: r_val = a_raw ^ b_raw;
                6'b010000: r_val = as[31:0] + bs[31:0] + {31'b0, carry_in};  // vadc
                6'b010010: r_val = as[31:0] - bs[31:0] - {31'b0, carry_in};  // vsbc
                6'b010001: begin                                              // vmadc
                    sum = au + bu + {63'b0, carry_in};
                    r_bit = sum[sewbits];
                end
                6'b010011: begin                                              // vmsbc
                    r_bit = (au < bu + {63'b0, carry_in});
                end
                6'b010111: r_val = (c_vm || v0s[e[8:0]]) ? b_raw : a_raw;    // vmerge / vmv.v
                6'b011000: r_bit = (au[31:0] & mask_sew) == (bu[31:0] & mask_sew);  // vmseq
                6'b011001: r_bit = (au[31:0] & mask_sew) != (bu[31:0] & mask_sew);  // vmsne
                6'b011010: r_bit = au < bu;                                   // vmsltu
                6'b011011: r_bit = $signed(as) < $signed(bs);                 // vmslt
                6'b011100: r_bit = au <= bu;                                  // vmsleu
                6'b011101: r_bit = $signed(as) <= $signed(bs);                // vmsle
                6'b011110: r_bit = au > bu;                                   // vmsgtu
                6'b011111: r_bit = $signed(as) > $signed(bs);                 // vmsgt
                6'b100000: begin                                              // vsaddu
                    sum = au + bu;
                    if (sum > umax) begin r_val = umax[31:0]; r_sat = 1'b1; end
                    else r_val = sum[31:0];
                end
                6'b100001: begin                                              // vsadd
                    sum = as + bs;
                    if ($signed(sum) > smax) begin r_val = smax[31:0]; r_sat = 1'b1; end
                    else if ($signed(sum) < smin) begin r_val = smin[31:0]; r_sat = 1'b1; end
                    else r_val = sum[31:0];
                end
                6'b100010: begin                                              // vssubu
                    if (au < bu) begin r_val = 32'b0; r_sat = 1'b1; end
                    else r_val = au[31:0] - bu[31:0];
                end
                6'b100011: begin                                              // vssub
                    sum = as - bs;
                    if ($signed(sum) > smax) begin r_val = smax[31:0]; r_sat = 1'b1; end
                    else if ($signed(sum) < smin) begin r_val = smin[31:0]; r_sat = 1'b1; end
                    else r_val = sum[31:0];
                end
                6'b100101: r_val = a_raw << (is_vi ? uimm[5:0] & (sewbits - 6'd1) : sh); // vsll
                6'b100111: begin                                              // vsmul
                    prod = m_prod[63:0];
                    sum = $signed(prod) >>> (sewbits - 6'd1);
                    sum = sum + {63'b0, round_inc(prod, sewbits - 6'd1, c_vxrm)};
                    if ($signed(sum) > smax) begin r_val = smax[31:0]; r_sat = 1'b1; end
                    else r_val = sum[31:0];
                end
                6'b101000: r_val = au[31:0] >> (is_vi ? uimm[5:0] & (sewbits - 6'd1) : sh);  // vsrl
                6'b101001: r_val = $signed(as[31:0]) >>> (is_vi ? uimm[5:0] & (sewbits - 6'd1) : sh);
                6'b101010: begin                                              // vssrl
                    sh = (is_vi ? uimm[5:0] : b_raw[5:0]) & (sewbits - 6'd1);
                    r_val = (au >> sh) + {63'b0, round_inc(au, sh, c_vxrm)};
                end
                6'b101011: begin                                              // vssra
                    sh = (is_vi ? uimm[5:0] : b_raw[5:0]) & (sewbits - 6'd1);
                    sum = $signed(as) >>> sh;
                    r_val = sum[31:0] + {31'b0, round_inc(as, sh, c_vxrm)};
                end
                6'b101100, 6'b101101, 6'b101110, 6'b101111: begin
                    // narrowing: vs2 is 2*SEW wide
                    sh = (is_vi ? uimm[5:0] : b_raw[5:0]) & ((sewbits << 1) - 6'd1);
                    case (c_funct6[1:0])
                        2'b00: r_val = au >> sh;                              // vnsrl
                        2'b01: begin sum = $signed(as) >>> sh; r_val = sum[31:0]; end  // vnsra
                        2'b10: begin                                          // vnclipu
                            sum = (au >> sh) + {63'b0, round_inc(au, sh, c_vxrm)};
                            if (sum > umax) begin r_val = umax[31:0]; r_sat = 1'b1; end
                            else r_val = sum[31:0];
                        end
                        default: begin                                        // vnclip
                            sum = $signed(as) >>> sh;     // keep the shift arithmetic
                            sum = sum + {63'b0, round_inc(as, sh, c_vxrm)};
                            if ($signed(sum) > smax) begin r_val = smax[31:0]; r_sat = 1'b1; end
                            else if ($signed(sum) < smin) begin r_val = smin[31:0]; r_sat = 1'b1; end
                            else r_val = sum[31:0];
                        end
                    endcase
                end
                default: r_val = 32'b0;
            endcase
        end else if (c_funct3 == F3_OPMVV || c_funct3 == F3_OPMVX) begin
            case (c_funct6)
                6'b001000: begin sum = au + bu; r_val = sum[32:1] + {31'b0, round_inc(sum, 6'd1, c_vxrm)}; end // vaaddu
                6'b001001: begin sum = as + bs; r_val = sum[32:1] + {31'b0, round_inc(sum, 6'd1, c_vxrm)}; end // vaadd
                6'b001010: begin sum = au - bu; r_val = sum[32:1] + {31'b0, round_inc(sum, 6'd1, c_vxrm)}; end // vasubu
                6'b001011: begin sum = as - bs; r_val = sum[32:1] + {31'b0, round_inc(sum, 6'd1, c_vxrm)}; end // vasub
                // High halves: bits [2*SEW-1:SEW] of the 64-bit product (truncated on write)
                6'b100100: begin prod = m_prod[63:0]; wide = prod >> sewbits; r_val = wide[31:0]; end  // vmulhu
                6'b100101: begin prod = m_prod[63:0]; r_val = prod[31:0]; end                         // vmul
                6'b100110: begin prod = m_prod[63:0]; wide = prod >> sewbits; r_val = wide[31:0]; end  // vmulhsu
                6'b100111: begin prod = m_prod[63:0]; wide = prod >> sewbits; r_val = wide[31:0]; end  // vmulh
                6'b101001: begin prod = m_prod[63:0]; r_val = prod[31:0] + as[31:0]; end          // vmadd
                6'b101011: begin prod = m_prod[63:0]; r_val = as[31:0] - prod[31:0]; end          // vnmsub
                6'b101101: begin prod = m_prod[63:0]; r_val = prod[31:0] + ds[31:0]; end          // vmacc
                6'b101111: begin prod = m_prod[63:0]; r_val = ds[31:0] - prod[31:0]; end          // vnmsac
                // widening: operands at SEW (or 2*SEW for .w), result 2*SEW
                6'b110000: r_val = au[31:0] + bu[31:0];                       // vwaddu
                6'b110001: r_val = as[31:0] + bs[31:0];                       // vwadd
                6'b110010: r_val = au[31:0] - bu[31:0];                       // vwsubu
                6'b110011: r_val = as[31:0] - bs[31:0];                       // vwsub
                6'b110100: r_val = au[31:0] + bu[31:0];                       // vwaddu.w
                6'b110101: r_val = as[31:0] + bs[31:0];                       // vwadd.w
                6'b110110: r_val = au[31:0] - bu[31:0];                       // vwsubu.w
                6'b110111: r_val = as[31:0] - bs[31:0];                       // vwsub.w
                6'b111000: begin prod = m_prod[63:0]; r_val = prod[31:0]; end      // vwmulu
                6'b111010: begin prod = m_prod[63:0]; r_val = prod[31:0]; end      // vwmulsu (vs2 signed)
                6'b111011: begin prod = m_prod[63:0]; r_val = prod[31:0]; end      // vwmul
                6'b111100: begin prod = m_prod[63:0]; r_val = prod[31:0] + d_raw; end   // vwmaccu
                6'b111101: begin prod = m_prod[63:0]; r_val = prod[31:0] + d_raw; end   // vwmacc
                6'b111110: begin prod = m_prod[63:0]; r_val = prod[31:0] + d_raw; end   // vwmaccus (rs1 unsigned)
                6'b111111: begin prod = m_prod[63:0]; r_val = prod[31:0] + d_raw; end   // vwmaccsu (vs1 signed)
                default: r_val = 32'b0;
            endcase
        end
    end

    // Reduction step: acc op element
    function [31:0] red_step;
        input [5:0] f6;
        input [31:0] accv;
        input [31:0] el;      // sign- or zero-extended as the op needs
        input [1:0] sew;
        reg [31:0] as_, es_;
        begin
            as_ = sx(accv, sew);
            es_ = el;
            case (f6[2:0])
                3'd0: red_step = accv + el;                                    // sum
                3'd1: red_step = accv & el;                                    // and
                3'd2: red_step = accv | el;                                    // or
                3'd3: red_step = accv ^ el;                                    // xor
                3'd4: red_step = (zx(accv, sew) < el) ? accv : el;             // minu
                3'd5: red_step = ($signed(as_) < $signed(es_)) ? accv : el;    // min
                3'd6: red_step = (zx(accv, sew) > el) ? accv : el;             // maxu
                default: red_step = ($signed(as_) > $signed(es_)) ? accv : el; // max
            endcase
        end
    endfunction

    // ======================================================================
    // FP32 element operation (shared FP unit)
    // ======================================================================
    // Scalar single ops reuse the vector FP32 unit through its vector encodings:
    // a = rs1 (or rs3 for the fused forms), b = rs1/rs2, d = rs2.
    wire c_sfp = (c_kind == VK_SFP) || (c_kind == VK_SFPX);
    wire [4:0] sfn = c_instr[31:27];
    wire [6:0] sop = c_instr[6:0];
    reg  [5:0] sf_f6;
    reg  [4:0] sf_v1;
    reg  [31:0] sf_a, sf_b, sf_d;
    always @(*) begin
        sf_f6 = 6'b000000; sf_v1 = 5'd0;
        sf_a = unbox(mcx_s1); sf_b = unbox(mcx_s2); sf_d = 32'b0;
        if (sop != 7'b1010011) begin
            // fused: rs1*rs2 + rs3 -> vfmadd family (b*d + a)
            sf_b = unbox(mcx_s1); sf_d = unbox(mcx_s2); sf_a = unbox(mcx_s3);
            case (sop)
                7'b1000011: sf_f6 = 6'b101000;   // fmadd  -> vfmadd
                7'b1000111: sf_f6 = 6'b101010;   // fmsub  -> vfmsub
                7'b1001011: sf_f6 = 6'b101011;   // fnmsub -> vfnmsub
                default:    sf_f6 = 6'b101001;   // fnmadd -> vfnmadd
            endcase
        end else begin
            case (sfn)
                5'b00000: sf_f6 = 6'b000000;
                5'b00001: sf_f6 = 6'b000010;
                5'b00010: sf_f6 = 6'b100100;
                5'b00100: sf_f6 = (c_instr[14:12] == 3'd0) ? 6'b001000 : (c_instr[14:12] == 3'd1) ? 6'b001001 : 6'b001010;
                5'b00101: sf_f6 = (c_instr[14:12] == 3'd0) ? 6'b000100 : 6'b000110;
                5'b10100: sf_f6 = (c_instr[14:12] == 3'd2) ? 6'b011000 : (c_instr[14:12] == 3'd1) ? 6'b011011 : 6'b011001;
                5'b11100: begin sf_f6 = 6'b010011; sf_v1 = 5'd16; end                 // fclass
                5'b11000: begin sf_f6 = 6'b010010; sf_v1 = c_instr[20] ? 5'd0 : 5'd1; end   // fcvt.w[u].s
                5'b11010: begin sf_f6 = 6'b010010; sf_v1 = c_instr[20] ? 5'd2 : 5'd3; sf_a = c_rs1; end
                default: ;
            endcase
        end
    end
    wire [31:0] fp_a = c_sfp ? sf_a : a_raw;
    wire [31:0] fp_b = c_sfp ? sf_b : (c_kind == VK_RED) ? acc : is_vv ? b_vec : c_rs1;
    wire [31:0] fp_d = c_sfp ? sf_d : d_raw;
    wire [31:0] fp_res;
    wire        fp_bit;
    wire [4:0]  fp_flags;
    vpu_fp32 fpu (
        .funct6(c_sfp ? sf_f6 : c_funct6), .vs1_field(c_sfp ? sf_v1 : c_vs1), .is_vf(!is_vv), .sew(c_sew),
        .a(fp_a), .b(fp_b), .d(fp_d), .rm(c_sfp ? c_srm : c_frm),
        .result(fp_res), .result_bit(fp_bit), .flags(fp_flags)
    );

    // Double precision and the S<->D conversions
    reg  [4:0] d_op;
    always @(*) begin
        d_op = 5'd0;
        if (sop != 7'b1010011) begin
            case (sop)
                7'b1000011: d_op = 5'd3;
                7'b1000111: d_op = 5'd4;
                7'b1001011: d_op = 5'd5;
                default:    d_op = 5'd6;
            endcase
        end else begin
            case (sfn)
                5'b00000: d_op = 5'd0;
                5'b00001: d_op = 5'd1;
                5'b00010: d_op = 5'd2;
                5'b00100: d_op = (c_instr[14:12] == 3'd0) ? 5'd7 : (c_instr[14:12] == 3'd1) ? 5'd8 : 5'd9;
                5'b00101: d_op = (c_instr[14:12] == 3'd0) ? 5'd10 : 5'd11;
                5'b10100: d_op = (c_instr[14:12] == 3'd2) ? 5'd12 : (c_instr[14:12] == 3'd1) ? 5'd13 : 5'd14;
                5'b11100: d_op = 5'd15;
                5'b11000: d_op = c_instr[20] ? 5'd17 : 5'd16;
                5'b11010: d_op = c_instr[20] ? 5'd19 : 5'd18;
                5'b01000: d_op = c_dbl ? 5'd21 : 5'd20;    // fcvt.d.s / fcvt.s.d
                default: ;
            endcase
        end
    end
    wire [63:0] fd_res;
    wire [4:0]  fd_flags;
    vpu_fp64 fpu64 (
        .op(d_op), .x(mcx_s1), .y(mcx_s2), .z(mcx_s3),
        .xi((sfn == 5'b01000) ? unbox(mcx_s1) : c_rs1), .rm(c_srm),
        .result(fd_res), .flags(fd_flags)
    );
    wire sf_is_div  = (sop == 7'b1010011) && (sfn == 5'b00011 || sfn == 5'b01011);
    reg  sf_div_start;
    wire fds32_done, fds64_done;
    wire [31:0] fds32_res;
    wire [63:0] fds64_res;
    wire [4:0] fds32_fl, fds64_fl;
    assign fds32_done = fdiv_done;
    assign fds32_res = fdiv_result;
    assign fds32_fl = fdiv_flags;
    fp_divsqrt #(.EW(11), .MW(52)) sdiv64 (
        .clk(clk), .rst(rst), .start(sf_div_start && c_dbl), .is_sqrt(sfn == 5'b01011),
        .x(sf1), .y(sf2), .rm(c_srm),
        .done(fds64_done), .result(fds64_res), .flags(fds64_fl)
    );
    reg [2:0] c_frm;
    reg [4:0] fflags_acc;

    // FP divide / square root (iterative)
    wire fdiv_op = (c_funct3 == F3_OPFVV || c_funct3 == F3_OPFVF) && c_kind == VK_ELEM &&
                   (c_funct6 == 6'b100000 || c_funct6 == 6'b100001 ||
                    (c_funct6 == 6'b010011 && c_vs1 == 5'd0));
    reg  fdiv_start;
    reg  fdiv_pending;
    wire fdiv_done;
    wire [31:0] fdiv_result;
    wire [4:0] fdiv_flags;
    // One FP32 divide/sqrt unit, shared by vfdiv/vfrdiv/vfsqrt and scalar fdiv.s/fsqrt.s
    // (the coprocessor runs one instruction at a time, so they never overlap).
    wire c_sfp_div = (c_kind == VK_SFP) && (c_instr[6:0] == 7'b1010011) &&
                     (c_instr[31:27] == 5'b00011 || c_instr[31:27] == 5'b01011);
    fp_divsqrt #(.EW(8), .MW(23)) fdivsqrt (
        .clk(clk), .rst(rst),
        .start(c_sfp_div ? (sf_div_start && !c_dbl) : fdiv_start),
        .is_sqrt(c_sfp_div ? (c_instr[31:27] == 5'b01011) : (c_funct6 == 6'b010011)),
        .x(c_sfp_div ? unbox(sf1) : (c_funct6 == 6'b100001 ? fp_b : fp_a)),
        .y(c_sfp_div ? unbox(sf2) : (c_funct6 == 6'b100001 ? fp_a : fp_b)),
        .rm(c_sfp_div ? c_srm : c_frm),
        .done(fdiv_done), .result(fdiv_result), .flags(fdiv_flags)
    );

    // ======================================================================
    // Memory address generation
    // ======================================================================
    wire [1:0]  mem_eew = (c_kind == VK_MEM_INDEXED) ? c_sew : c_is_sfp_mem ? 2'd2 : c_eew_d;   // data element size
    wire [31:0] mem_bytes = 32'd1 << mem_eew;
    wire [31:0] idx_val = a_elem;
    // Accesses stream: when one completes and the next (element, field) is active, its address
    // is presented in the same cycle (also as the D-cache's early index), so hits take one cycle.
    wire mem_whole = (c_kind == VK_MEM_WHOLE) || (c_kind == VK_MEM_MASK) || c_is_sfp_mem;
    wire nx_fld_adv = !mem_whole && (fld != c_nf);
    wire [9:0] nx_e = nx_fld_adv ? e : e + 10'd1;
    wire [2:0] nx_f = nx_fld_adv ? fld + 3'd1 : 3'd0;
    wire nx_active = c_vm || mem_whole || v0s[nx_e[8:0]];
    wire mem_busy_req = (mem_rd || mem_wr) && !mem_setup;
    wire mem_chain = (state == S_MEM) && mem_busy_req && mem_rvalid && !mem_load_pf && !mem_store_pf &&
                     !(!nx_fld_adv && last_elem) && nx_active;
    // While an access is outstanding the address/data logic already works on the next element,
    // so a hit only selects it (mem_chain) instead of starting the index -> address chain.
    wire [9:0] ae = mem_busy_req ? nx_e : e;       // element whose address is generated
    wire [2:0] af = mem_busy_req ? nx_f : fld;
    reg  [31:0] elem_addr;
    always @(*) begin
        case (c_kind)
            VK_MEM_STRIDED: elem_addr = c_rs1 + c_rs2 * {22'b0, ae} + {29'b0, af} * mem_bytes;
            VK_MEM_INDEXED: elem_addr = c_rs1 + idx_val + {29'b0, af} * mem_bytes;
            VK_MEM_WHOLE, VK_MEM_MASK: elem_addr = c_rs1 + ({22'b0, ae} << c_eew_d);
            VK_SFLD, VK_SFST: elem_addr = c_rs1 + {20'b0, ae, 2'b00};
            default:        elem_addr = c_rs1 + ({22'b0, ae} * ({29'b0, c_nf} + 32'd1) + {29'b0, af}) * mem_bytes;
        endcase
    end
    // Data register of field fld (loads write it), and of the field whose data a store sends next
    wire [4:0] fld_reg = c_vd + fld * c_field_regs;
    wire [4:0] st_reg  = c_vd + af * c_field_regs;
    wire [31:0] st_data = c_is_sfp_mem ? (ae[0] ? sf2[63:32] : sf2[31:0]) : d_elem;

    // An access is set up one cycle before it is requested, so the D-cache's early index
    // (mem_addr_next) always matches the address it then answers for.
    reg mem_setup;
    assign mem_addr_next = mem_chain ? elem_addr : mem_addr;
    assign mem_own = (state == S_MEM);

    // ======================================================================
    // Main sequencer
    // ======================================================================
    reg        wr_en;
    reg [8:0]  wr_word;
    reg [31:0] wr_mask;
    reg [31:0] wr_val;

    // Writes always land in the register on read port D, at its locator (offD/eewD), or at
    // mask bit e of it. The rest of the word is merged from the port's old value.
    task put_elem;
        input [31:0] val;
        begin
            wr_en = 1'b1;
            wr_word = {5'b0, offD[5:2]};
            case (eewD)
                2'd0: begin wr_mask = 32'hFF << {offD[1:0], 3'b0}; wr_val = {4{val[7:0]}}; end
                2'd1: begin wr_mask = 32'hFFFF << {offD[1], 4'b0}; wr_val = {2{val[15:0]}}; end
                default: begin wr_mask = 32'hFFFFFFFF; wr_val = val; end
            endcase
        end
    endtask

    task put_bit;
        input b;
        begin
            wr_en = 1'b1;
            wr_word = {5'b0, e[8:5]};
            wr_mask = 32'd1 << e[4:0];
            wr_val = {32{b}};
        end
    endtask

    // Effective vl of the instruction being started
    reg [9:0] start_evl;
    always @(*) begin
        case (dec_kind)
            VK_MEM_WHOLE: start_evl = (({7'b0, h_instr[31:29]} + 10'd1) << 6) >> dec_eew_d;
            VK_MEM_MASK:  start_evl = (q_vl[q_head][9:0] + 10'd7) >> 3;
            VK_VMVNR:     start_evl = ({7'b0, h_instr[17:15]} + 10'd1) << 4;   // words
            VK_SFLD, VK_SFST: start_evl = (h_instr[14:12] == 3'b011) ? 10'd2 : 10'd1;
            default:      start_evl = q_vl[q_head][9:0];
        endcase
    end

    // ======================================================================
    // Fast path: SEW=32 elementwise integer / FP32 ops on the SIMD lanes
    // ======================================================================
    localparam BPR = 16 / LANES;                 // beats per 512-bit register
    wire [5:0] h_f6 = h_instr[31:26];
    wire [2:0] h_f3 = h_instr[14:12];
    reg h_fast;
    always @(*) begin
        h_fast = 1'b0;
        if (dec_kind == VK_ELEM && h_vtype[5:3] == 3'd2) begin
            case (h_f3)
                F3_OPIVV, F3_OPIVX, F3_OPIVI:
                    case (h_f6)
                        6'b000000, 6'b000010, 6'b000011, 6'b000100, 6'b000101, 6'b000110, 6'b000111,
                        6'b001001, 6'b001010, 6'b001011, 6'b100101, 6'b101000, 6'b101001, 6'b010111:
                            h_fast = 1'b1;
                        default: ;
                    endcase
                F3_OPMVV, F3_OPMVX:
                    case (h_f6)
                        6'b100100, 6'b100101, 6'b100110, 6'b100111, 6'b101001, 6'b101011, 6'b101101, 6'b101111:
                            h_fast = 1'b1;
                        default: ;
                    endcase
                F3_OPFVV, F3_OPFVF:
                    case (h_f6)
                        6'b000000, 6'b000010, 6'b000100, 6'b000110, 6'b001000, 6'b001001, 6'b001010, 6'b100100,
                        6'b100111, 6'b101000, 6'b101001, 6'b101010, 6'b101011, 6'b101100, 6'b101101, 6'b101110,
                        6'b101111:
                            h_fast = 1'b1;
                        default: ;
                    endcase
                default: ;
            endcase
        end
    end
    wire [3:0] h_nregs = (h_vtype[2:0] == 3'b001) ? 4'd2 : (h_vtype[2:0] == 3'b010) ? 4'd4 :
                         (h_vtype[2:0] == 3'b011) ? 4'd8 : 4'd1;

    reg  [4:0] f_beat, f_last;
    reg  [2:0] f_drain;
    wire [3:0] f_reg  = f_beat / BPR;
    wire [3:0] f_half = f_beat % BPR;
    wire [9:0] f_base = {f_reg, 4'b0} + f_half * LANES;    // first element of the beat
    wire f_merge = (c_funct3 == F3_OPIVV || c_funct3 == F3_OPIVX || c_funct3 == F3_OPIVI) &&
                   (c_funct6 == 6'b010111);
    wire [31:0] f_scalar = is_vi ? simm : c_rs1;
    reg  [LANES-1:0] f_act, f_sel;
    integer fl;
    always @(*) begin
        for (fl = 0; fl < LANES; fl = fl + 1) begin
            f_sel[fl] = c_vm || v0s[f_base + fl];
            f_act[fl] = ({22'b0, f_base} + fl >= c_vstart) && ({22'b0, f_base} + fl < c_vl) &&
                        (f_merge || f_sel[fl]);
        end
    end
    wire [32*LANES-1:0] f_a = vA >> (f_half * LANES * 32);
    wire [32*LANES-1:0] f_bv = vB >> (f_half * LANES * 32);
    wire [32*LANES-1:0] f_d = vD >> (f_half * LANES * 32);

    wire                 ln_valid;
    wire [32*LANES-1:0]  ln_result;
    wire [LANES-1:0]     ln_active;
    wire [4:0]           ln_wreg;
    wire [15:0]          ln_wword;
    wire [4:0]           ln_fflags;
    vpu_lanes #(.LANES(LANES)) lanes (
        .clk(clk), .rst(rst),
        .in_valid(state == S_FAST), .funct3(c_funct3), .funct6(c_funct6), .frm(c_frm),
        .a(f_a), .b(is_vv ? f_bv : {LANES{f_scalar}}), .d(f_d),
        .active(f_act), .sel(f_sel),
        .wreg_in(c_vd + {1'b0, f_reg}), .wword_in({6'b0, f_base[3:0]} & 16'h000F),
        .out_valid(ln_valid), .result(ln_result), .out_active(ln_active),
        .wreg_out(ln_wreg), .wword_out(ln_wword), .fflags(ln_fflags)
    );

    // ======================================================================
    // Read-port addressing: each port reads the register holding the element it needs
    // ======================================================================
    always @(*) begin : port_addr
        reg [10:0] la, lb, ld;
        reg [9:0] src;
        reg [31:0] gidx;
        la = loc(c_vs2, e, c_eew_s2); eewA = c_eew_s2;
        lb = loc(c_vs1, e, c_eew_s1); eewB = c_eew_s1;
        ld = loc(c_vd, e, c_eew_d);   eewD = c_eew_d;
        src = 10'd0;
        gidx = 32'd0;
        case (c_kind)
            VK_SLIDEUP: begin
                src = (c_funct6 == 6'b001110 && (c_funct3 == F3_OPMVX || c_funct3 == F3_OPFVF)) ?
                      e - 10'd1 : e - slide_off;
                la = loc(c_vs2, src, c_sew); eewA = c_sew;
            end
            VK_SLIDEDOWN: begin
                src = (c_funct6 == 6'b001111 && (c_funct3 == F3_OPMVX || c_funct3 == F3_OPFVF)) ?
                      e + 10'd1 : e + slide_off;
                la = loc(c_vs2, src, c_sew); eewA = c_sew;
            end
            VK_GATHER, VK_GATHER16: begin
                gidx = is_vv ? mcx_b : is_vi ? uimm : c_rs1;
                la = loc(c_vs2, gidx[9:0], c_sew); eewA = c_sew;
            end
            VK_VMVNR: begin
                la = loc(c_vs2, e, 2'd2); eewA = 2'd2;
                ld = loc(c_vd, e, 2'd2);  eewD = 2'd2;
            end
            VK_MASKLOGIC: begin la = {c_vs2, 6'b0}; lb = {c_vs1, 6'b0}; ld = {c_vd, 6'b0}; end
            VK_MSETBIT:   begin la = {c_vs2, 6'b0}; ld = {c_vd, 6'b0}; end
            VK_IOTA:      la = {c_vs2, 6'b0};
            VK_XUNARY:    la = (c_vs1 == 5'd0) ? loc(c_vs2, 10'd0, c_sew) : {c_vs2, 6'b0};
            VK_COMPRESS: begin
                la = loc(c_vs2, e, c_sew); eewA = c_sew;
                lb = {c_vs1, 6'b0};
                ld = loc(c_vd, acc[9:0], c_sew); eewD = c_sew;
            end
            VK_MASKDEST:  ld = {c_vd, 6'b0};
            VK_RED, VK_WRED: ld = loc(c_vd, 10'd0, c_eew_d);
            VK_SUNARY: begin ld = loc(c_vd, 10'd0, c_sew); eewD = c_sew; end
            default: ;
        endcase
        if (state == S_MEM) begin
            la = loc(c_vs2, ae, c_eew_s2); eewA = c_eew_s2;
            if (mem_whole) begin
                ld = loc(c_vd, c_store ? ae : e, c_eew_d); eewD = c_eew_d;
            end else if (c_store) begin
                ld = loc(st_reg, ae, mem_eew); eewD = mem_eew;
            end else begin
                ld = loc(fld_reg, e, mem_eew); eewD = mem_eew;
            end
        end
        if (state == S_IDLE) begin
            lb = loc(h_instr[19:15], 10'd0, dec_eew_d); eewB = dec_eew_d;
        end
        rA = la[10:6]; offA = la[5:0];
        rB = lb[10:6]; offB = lb[5:0];
        rD = ld[10:6]; offD = ld[5:0];
        if (state == S_FAST) begin
            rA = c_vs2 + {1'b0, f_reg};
            rB = c_vs1 + {1'b0, f_reg};
            rD = c_vd + {1'b0, f_reg};
        end
    end

    // ---------------- Captured element results (end of ph 4) ----------------
    reg        mcy_wr_en;
    reg [8:0]  mcy_wr_word;
    reg [31:0] mcy_wr_mask, mcy_wr_val, mcy_red;
    reg [4:0]  mcy_fl;
    reg        mcy_sat;
    always @(posedge clk) begin
        if (state == S_RUN && ph == PH_CAP) begin
            mcy_wr_en <= wr_en;
            mcy_wr_word <= wr_word;
            mcy_wr_mask <= wr_mask;
            mcy_wr_val <= wr_val;
            mcy_red <= red_next;
            mcy_fl <= fp_flags;
            mcy_sat <= r_sat;
        end
    end
    wire        wp_run  = (state == S_RUN);
    wire        wp_en   = wp_run ? (ph == PH_WR && mcy_wr_en) : wr_en;
    wire [8:0]  wp_word = wp_run ? mcy_wr_word : wr_word;
    wire [31:0] wp_mask = wp_run ? mcy_wr_mask : wr_mask;
    wire [31:0] wp_val  = wp_run ? mcy_wr_val : wr_val;

    // Operand fetch (ph 0, and ph 1 for gathers)
    always @(posedge clk) begin
        if (state == S_RUN && (ph == PH_FETCH || ph == PH_FETCH2)) begin
            if (ph == PH_FETCH) begin
                mcx_b <= b_elem;
                mcx_d <= d_elem;
                mcx_mb <= vB[e[8:0]];
            end
            mcx_a <= a_elem;
            mcx_ma <= vA[e[8:0]];
        end
        if (state == S_SFP && ph == PH_FETCH) begin
            mcx_s1 <= sf1;
            mcx_s2 <= sf2;
            mcx_s3 <= sf3;
        end
    end

    // ---------------- Write port: lanes, or the element sequencer's merged word ----------------
    integer wl;
    always @(*) begin
        w_reg = rD;
        w_en = 16'b0;
        w_data = 512'b0;
        if (ln_valid) begin
            w_reg = ln_wreg;
            for (wl = 0; wl < LANES; wl = wl + 1) begin
                w_en[ln_wword[3:0] + wl] = ln_active[wl];
                w_data[(ln_wword[3:0] + wl) * 32 +: 32] = ln_result[32*wl +: 32];
            end
        end else if (wp_en) begin
            w_en[wp_word[3:0]] = 1'b1;
            w_data = {16{(vD[{wp_word[3:0], 5'b0} +: 32] & ~wp_mask) | (wp_val & wp_mask)}};
        end
    end

    // ---------------- FP register write port (same cycle as the result) ----------------
    assign sf1_a = (state == S_IDLE) ? h_instr[19:15] : c_vs1;
    always @(*) begin
        fw_lo = 1'b0;
        fw_hi = 1'b0;
        fw_a = c_vd;
        fw_d = 64'b0;
        case (state)
            S_SFP: if (!sf_is_div && c_kind != VK_SFPX && ph == PH_WR) begin
                fw_lo = 1'b1;
                fw_hi = 1'b1;
                fw_d = mcz_wd;
            end
            S_SFDIV: if (c_dbl ? fds64_done : fds32_done) begin
                fw_lo = 1'b1;
                fw_hi = 1'b1;
                fw_d = c_dbl ? fds64_res : {32'hFFFFFFFF, fds32_res};
            end
            S_MEM: if (c_is_sfp_mem && c_load && mem_busy_req && mem_rvalid && !mem_load_pf) begin
                // flw: whole register (boxed); fld: low word, then high word
                fw_lo = !c_dbl || !e[0];
                fw_hi = !c_dbl || e[0];
                fw_d = c_dbl ? {mem_rdata, mem_rdata} : {32'hFFFFFFFF, mem_rdata};
            end
            S_DONE: if (c_kind == VK_XUNARY && c_funct3 == F3_OPFVV) begin
                fw_lo = 1'b1;
                fw_hi = 1'b1;
                fw_d = {32'hFFFFFFFF, a_elem};                                                 // vfmv.f.s
            end
            default: ;
        endcase
    end

    // Scalar FP results, captured at the end of ph 4 and used in ph 5
    reg [63:0] sfp_wd;
    reg [31:0] sfp_resp;
    reg [4:0]  sfp_fl;
    always @(*) begin
        if (sfn == 5'b11110 && sop == 7'b1010011) sfp_wd = {32'hFFFFFFFF, c_rs1};               // fmv.w.x
        else if (sop == 7'b1010011 && sfn == 5'b01000)
            sfp_wd = c_dbl ? fd_res : {32'hFFFFFFFF, fd_res[31:0]};                            // cvt.d.s / cvt.s.d
        else if (c_dbl) sfp_wd = fd_res;
        else sfp_wd = {32'hFFFFFFFF, fp_res};
        if (sfn == 5'b11100 && c_instr[14:12] == 3'd0) sfp_resp = mcx_s1[31:0];                 // fmv.x.w
        else sfp_resp = c_dbl ? fd_res[31:0] : (sfn == 5'b10100) ? {31'b0, fp_bit} : fp_res;
        if (c_kind == VK_SFPX) sfp_fl = c_dbl ? fd_flags : fp_flags;
        else if (sfn == 5'b11110 && sop == 7'b1010011) sfp_fl = 5'b0;                           // fmv.w.x
        else if ((sop == 7'b1010011 && sfn == 5'b01000) || c_dbl) sfp_fl = fd_flags;
        else sfp_fl = fp_flags;
    end
    always @(posedge clk) begin
        if (state == S_SFP && ph == PH_CAP) begin
            mcz_wd <= sfp_wd;
            mcz_resp <= sfp_resp;
            mcz_fl <= sfp_fl;
        end
    end

    // Combinational per-cycle decisions
    reg        step_done;      // this element is finished this cycle
    always @(*) fdiv_start = (state == S_RUN) && (ph == PH_WR) && fdiv_op && mask_on;
    always @(*) sf_div_start = (state == S_SFP) && (ph == PH_FETCH) && sf_is_div;
    reg        last_elem;
    reg [31:0] sel_val;

    always @(*) begin
        wr_en = 1'b0;
        wr_word = 9'b0;
        wr_mask = 32'b0;
        wr_val = 32'b0;
        step_done = 1'b0;
        div_req = 1'b0;
        sel_val = 32'b0;
        last_elem = (e + 10'd1 >= evl) || (c_kind == VK_SUNARY);

        if (state == S_RUN) begin
            step_done = 1'b1;
            case (c_kind)
                VK_ELEM, VK_WIDEN, VK_NARROW, VK_EXT: begin
                    if ((c_funct3 == F3_OPFVV || c_funct3 == F3_OPFVF) && fdiv_op) begin
                        step_done = !mask_on;                            // S_DIV finishes it
                    end else if (c_funct3 == F3_OPFVV || c_funct3 == F3_OPFVF) begin
                        if (c_funct6 == 6'b010111) begin                 // vfmerge / vfmv.v.f
                            put_elem((c_vm || v0s[e[8:0]]) ? c_rs1 : a_raw);
                        end else if (mask_on) put_elem(fp_res);
                    end else if (c_kind == VK_EXT) begin
                        if (mask_on) put_elem(c_vs1[0] ? sx(a_raw, c_eew_s2) : zx(a_raw, c_eew_s2));
                    end else if (div_op || fdiv_op) begin
                        if (mask_on) begin
                            div_req = div_op && (ph == PH_WR);
                            step_done = 1'b0;    // S_DIV finishes it
                        end
                    end else if (c_funct6 == 6'b010111 || c_funct6 == 6'b010000 || c_funct6 == 6'b010010) begin
                        // merge, vadc, vsbc: all body elements written, v0 is data
                        if (c_funct3 != F3_OPMVV && c_funct3 != F3_OPMVX) put_elem(r_val);
                        else if (mask_on) put_elem(r_val);
                    end else if (mask_on) begin
                        put_elem(r_val);
                    end
                end
                VK_MASKDEST: begin
                    if (c_funct3 == F3_OPFVV || c_funct3 == F3_OPFVF) begin
                        if (mask_on) put_bit(fp_bit);
                    end else if (c_funct6 == 6'b010001 || c_funct6 == 6'b010011) begin
                        put_bit(r_bit);      // vmadc/vmsbc: v0 is carry-in
                    end else if (mask_on) put_bit(r_bit);
                end
                VK_MASKLOGIC: begin
                    case (c_funct6[2:0])
                        3'b000: put_bit(mcx_ma & ~mcx_mb);   // vmandn
                        3'b001: put_bit(mcx_ma & mcx_mb);    // vmand
                        3'b010: put_bit(mcx_ma | mcx_mb);    // vmor
                        3'b011: put_bit(mcx_ma ^ mcx_mb);    // vmxor
                        3'b100: put_bit(mcx_ma | ~mcx_mb);   // vmorn
                        3'b101: put_bit(~(mcx_ma & mcx_mb)); // vmnand
                        3'b110: put_bit(~(mcx_ma | mcx_mb)); // vmnor
                        default: put_bit(~(mcx_ma ^ mcx_mb));// vmxnor
                    endcase
                end
                VK_RED, VK_WRED: begin
                    // acc already holds vs1[0]; fold active elements, write vd[0] at the end.
                    // vfredusum with no active element and a NaN vs1[0] gives the canonical
                    // NaN (NV if it was signaling), as Spike does.
                    if (last_elem) put_elem(red_none_nan ? 32'h7FC00000 : red_next);
                end
                VK_SLIDEUP: begin
                    if (c_funct6 == 6'b001110 && (c_funct3 == F3_OPMVX || c_funct3 == F3_OPFVF)) begin
                        if (mask_on) put_elem((e == 10'd0) ? c_rs1 : a_raw);
                    end else if (e >= slide_off && mask_on) begin
                        put_elem(a_raw);
                    end
                end
                VK_SLIDEDOWN: begin
                    if (c_funct6 == 6'b001111 && (c_funct3 == F3_OPMVX || c_funct3 == F3_OPFVF)) begin
                        if (mask_on) put_elem((e + 10'd1 == evl) ? c_rs1 : a_raw);
                    end else if (mask_on) begin
                        put_elem(({1'b0, e} + {1'b0, slide_off} < {1'b0, c_vlmax} && slide_amt < 32'd1024) ?
                                 a_raw : 32'b0);
                    end
                end
                VK_GATHER, VK_GATHER16: begin
                    if (mask_on) begin
                        sel_val = is_vv ? zx(b_vec, c_eew_s1) : is_vi ? uimm : c_rs1;
                        put_elem((sel_val < {22'b0, c_vlmax}) ? a_raw : 32'b0);
                    end
                end
                VK_COMPRESS: begin
                    if (mcx_mb) put_elem(a_raw);
                end
                VK_VMVNR: put_elem(a_raw);
                VK_XUNARY: ;    // result computed below, no writes
                VK_SUNARY: begin
                    if (evl != 10'd0) put_elem(c_rs1);
                end
                VK_MSETBIT: begin
                    if (mask_on) begin
                        case (c_vs1[1:0])
                            2'd1: put_bit(!found && !mcx_ma);                 // vmsbf
                            2'd3: put_bit(!found);                                      // vmsif
                            default: put_bit(!found && mcx_ma);               // vmsof
                        endcase
                    end
                end
                VK_IOTA: if (mask_on) put_elem(acc);
                VK_VID:  if (mask_on) put_elem({22'b0, e});
                default: ;
            endcase
        end else if (state == S_DIV) begin
            if (fdiv_op) begin
                if (fdiv_done) begin
                    step_done = 1'b1;
                    put_elem(fdiv_result);
                end
            end else begin
                div_req = 1'b1;
                if (div_ready) begin
                    step_done = 1'b1;
                    put_elem(div_result);
                end
            end
        end else if (state == S_MEM) begin
            if (c_load && mem_rvalid && !mem_load_pf && !c_is_sfp_mem) begin
                put_elem(mem_rdata);
            end
        end
    end

    // Reduction fold value (acc op active element)
    always @(*) begin
        red_next = acc;
        if (c_kind == VK_RED && (c_funct3 == F3_OPFVV)) begin
            red_next = mask_on ? fp_res : acc;
        end else if (c_kind == VK_RED) begin
            red_next = mask_on ? red_step(c_funct6, acc,
                                          (c_funct6[2:0] == 3'd5 || c_funct6[2:0] == 3'd7) ? sx(a_raw, c_sew) : zx(a_raw, c_sew),
                                          c_sew) : acc;
        end else if (c_kind == VK_WRED) begin
            red_next = mask_on ? acc + (c_funct6[0] ? sx(a_raw, c_sew) : zx(a_raw, c_sew)) : acc;
        end
    end

    // Does this kind start at vstart (true) or element 0?
    wire h_from_zero = (dec_kind == VK_RED) || (dec_kind == VK_WRED) || (dec_kind == VK_COMPRESS) ||
                       (dec_kind == VK_VMVNR) ||
                       (dec_kind == VK_XUNARY) || (dec_kind == VK_IOTA) || (dec_kind == VK_MSETBIT);

    // Sequential
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            e <= 10'd0;
            fld <= 3'd0;
            evl <= 10'd0;
            acc <= 32'd0;
            found <= 1'b0;
            q_pop <= 1'b0;
            resp_valid <= 1'b0;
            resp_data <= 32'b0;
            resp_fault <= 1'b0;
            resp_fault_store <= 1'b0;
            resp_fault_addr <= 32'b0;
            resp_fault_vstart <= 32'b0;
            resp_vl_valid <= 1'b0;
            resp_vl <= 32'b0;
            vxsat_set <= 1'b0;
            mem_rd <= 1'b0;
            mem_wr <= 1'b0;
            mem_addr <= 32'b0;
            mem_wdata <= 32'b0;
            mem_be <= 4'b0;
            mem_load_type <= 3'b0;
            fault_hit <= 1'b0;
            mem_setup <= 1'b0;
            fflags_acc <= 5'b0;
            fflags_set <= 5'b0;
            f_beat <= 5'd0;
            f_last <= 5'd0;
            f_drain <= 3'd0;
            ph <= PH_FETCH;
            c_instr <= 32'b0;
            c_rs1 <= 32'b0;
            c_rs2 <= 32'b0;
            c_vl <= 32'b0;
            c_vtype <= 32'b0;
            c_vstart <= 32'b0;
            c_vxrm <= 2'b0;
            c_frm <= 3'b0;
            c_kind <= VK_NONE;
            c_eew_d <= 2'b0;
            c_eew_s2 <= 2'b0;
            c_eew_s1 <= 2'b0;
            c_load <= 1'b0;
            c_store <= 1'b0;
            c_scalar <= 1'b0;
        end else begin
            q_pop <= 1'b0;
            fflags_set <= ln_valid ? ln_fflags : 5'b0;
            resp_valid <= 1'b0;
            resp_fault <= 1'b0;
            resp_vl_valid <= 1'b0;
            vxsat_set <= 1'b0;


            case (state)
                S_IDLE: if (h_valid && !q_pop) begin
                    c_instr <= h_instr;
                    // .vf forms take f[rs1] (NaN-unboxed); everything else the x value
                    c_rs1 <= (h_instr[6:0] == 7'b1010111 && h_instr[14:12] == F3_OPFVF) ?
                             unbox(sf1) : q_rs1[q_head];
                    c_rs2 <= q_rs2[q_head];
                    c_vl <= q_vl[q_head];
                    c_vtype <= h_vtype;
                    c_vstart <= q_vstart[q_head];
                    c_vxrm <= q_vxrm[q_head];
                    c_frm <= q_frm[q_head];
                    c_kind <= dec_kind;
                    c_eew_d <= dec_eew_d;
                    c_eew_s2 <= dec_eew_s2;
                    c_eew_s1 <= dec_eew_s1;
                    c_load <= dec_is_load;
                    c_store <= dec_is_store;
                    c_scalar <= dec_scalar_dest;
                    evl <= start_evl;
                    e <= h_from_zero ? 10'd0 : q_vstart[q_head][9:0];
                    fld <= 3'd0;
                    found <= 1'b0;
                    fault_hit <= 1'b0;
                    ph <= PH_FETCH;
                    // Reductions start from vs1[0] (2*SEW for widening reductions)
                    acc <= (dec_kind == VK_RED || dec_kind == VK_WRED) ? b_elem : 32'd0;
                    if (dec_kind == VK_SFP || dec_kind == VK_SFPX) begin
                        state <= S_SFP;
                    end else if (dec_is_load || dec_is_store) begin
                        state <= (q_vstart[q_head][9:0] >= start_evl) ? S_DONE : S_MEM;
                    end else if (dec_kind == VK_XUNARY && h_instr[19:15] == 5'd0) begin
                        state <= S_DONE;     // vmv.x.s / vfmv.f.s: no loop
                    end else if (h_fast) begin
                        f_beat <= 5'd0;
                        f_last <= h_nregs * BPR - 5'd1;
                        state <= S_FAST;
                    end else begin
                        state <= ((h_from_zero ? 10'd0 : q_vstart[q_head][9:0]) >= start_evl) ? S_DONE : S_RUN;
                    end
                end

                S_RUN: if (ph == PH_FETCH) begin
                    ph <= (c_kind == VK_GATHER || c_kind == VK_GATHER16) ? PH_FETCH2 : 3'd2;
                end else if (ph != PH_WR) begin
                    ph <= ph + 3'd1;
                end else begin
                    ph <= PH_FETCH;
                    if ((c_funct3 == F3_OPFVV || c_funct3 == F3_OPFVF) && !fdiv_op && mask_on &&
                        c_kind != VK_SLIDEUP && c_kind != VK_SLIDEDOWN && c_kind != VK_XUNARY &&
                        c_kind != VK_SUNARY && c_funct6 != 6'b010111)
                        fflags_set <= mcy_fl;
                    if (mcy_sat && mask_on && (c_kind == VK_ELEM || c_kind == VK_NARROW)) vxsat_set <= 1'b1;
                    if (c_kind == VK_RED || c_kind == VK_WRED) acc <= mcy_red;
                    if (c_kind == VK_RED && mask_on) found <= 1'b1;
                    if (red_none_nan && last_elem && !acc[22]) fflags_set <= 5'b10000;
                    if (c_kind == VK_COMPRESS && mcx_mb) acc <= acc + 32'd1;
                    if (c_kind == VK_IOTA && mask_on && mcx_ma) acc <= acc + 32'd1;
                    if (c_kind == VK_XUNARY && mask_on && mcx_ma) begin
                        if (c_vs1 == 5'd16) acc <= acc + 32'd1;                     // vcpop
                        else if (!found) begin acc <= {22'b0, e}; found <= 1'b1; end // vfirst
                    end
                    if (c_kind == VK_MSETBIT && mask_on && mcx_ma) found <= 1'b1;
                    if (fdiv_op && mask_on) begin
                        state <= S_DIV;
                    end else if (div_req && !step_done) begin
                        state <= S_DIV;
                    end else if (last_elem) begin
                        state <= S_DONE;
                    end else begin
                        e <= e + 10'd1;
                    end
                end

                S_SFP: begin
                    if (sf_is_div) begin
                        state <= S_SFDIV;
                    end else if (ph != PH_WR) begin
                        ph <= ph + 3'd1;
                    end else begin
                        // write f[rd] (port below) or answer with an x value
                        ph <= PH_FETCH;
                        if (c_kind == VK_SFPX) resp_data <= mcz_resp;
                        fflags_set <= mcz_fl;
                        state <= S_DONE;
                    end
                end

                S_SFDIV: begin
                    if (c_dbl ? fds64_done : fds32_done) begin
                        fflags_set <= c_dbl ? fds64_fl : fds32_fl;
                        state <= S_DONE;
                    end
                end

                S_FAST: begin
                    if (f_beat == f_last) begin
                        f_drain <= 3'd4;   // lane pipeline depth
                        state <= S_DRAIN;
                    end else f_beat <= f_beat + 5'd1;
                end

                S_DRAIN: begin
                    if (f_drain == 3'd0) state <= S_DONE;
                    else f_drain <= f_drain - 3'd1;
                end

                S_DIV: if (fdiv_op ? fdiv_done : div_ready) begin
                    if (fdiv_op) fflags_set <= fdiv_flags;
                    if (last_elem) state <= S_DONE;
                    else begin
                        e <= e + 10'd1;
                        state <= S_RUN;
                    end
                end

                S_MEM: begin
                    // Skip inactive elements; issue one access per (element, field)
                    if (mem_setup) begin
                        mem_setup <= 1'b0;
                        mem_rd <= c_load;
                        mem_wr <= c_store;
                    end else if (!mem_rd && !mem_wr) begin
                        if (!(c_vm || mem_whole) && !v0s[e[8:0]]) begin
                            if (last_elem) state <= S_DONE;
                            else e <= e + 10'd1;
                        end else begin
                            mem_addr <= elem_addr;
                            mem_setup <= 1'b1;
                            mem_wdata <= st_data;
                            mem_be <= (c_is_sfp_mem) ? 4'b1111 :
                                      (mem_eew == 2'd0 || c_kind == VK_MEM_MASK) ? 4'b0001 :
                                      (mem_eew == 2'd1) ? 4'b0011 : 4'b1111;
                            mem_load_type <= c_is_sfp_mem ? 3'b010 :
                                             ((c_kind == VK_MEM_WHOLE ? c_eew_d : mem_eew) == 2'd0) ? 3'b100 :
                                             ((c_kind == VK_MEM_WHOLE ? c_eew_d : mem_eew) == 2'd1) ? 3'b101 : 3'b010;
                        end
                    end else if (mem_load_pf || mem_store_pf) begin
                        mem_rd <= 1'b0;
                        mem_wr <= 1'b0;
                        if (c_ff && c_load && e != 10'd0) begin
                            // fault-only-first: trim vl, no trap
                            resp_vl_valid <= 1'b1;
                            resp_vl <= {22'b0, e};
                            state <= S_DONE;
                        end else begin
                            fault_hit <= 1'b1;
                            resp_fault_store <= mem_store_pf;
                            resp_fault_addr <= mem_addr;
                            resp_fault_vstart <= {22'b0, e};
                            state <= S_DONE;
                        end
                    end else if (mem_rvalid) begin
                        if (mem_chain) begin
                            // next access goes out at once
                            mem_addr <= elem_addr;
                            mem_wdata <= st_data;
                        end else begin
                            mem_rd <= 1'b0;
                            mem_wr <= 1'b0;
                        end
                        if (nx_fld_adv) begin
                            fld <= fld + 3'd1;
                        end else begin
                            fld <= 3'd0;
                            if (last_elem) state <= S_DONE;
                            else e <= e + 10'd1;
                        end
                    end
                end

                S_DONE: begin
                    q_pop <= 1'b1;
                    state <= S_IDLE;
                    if (c_scalar || c_load || c_store) begin
                        resp_valid <= 1'b1;
                        resp_fault <= fault_hit;
                        if (c_kind == VK_XUNARY) begin
                            if (c_vs1 == 5'd0) resp_data <= (c_funct3 == F3_OPFVV) ? a_elem : sx(a_elem, c_sew);
                            else if (c_vs1 == 5'd16) resp_data <= acc;
                            else resp_data <= found ? acc : 32'hFFFFFFFF;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
