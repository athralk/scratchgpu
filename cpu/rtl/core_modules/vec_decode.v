`default_nettype none
// Vector instruction classification and legality (Zve32x/Zve32f, VLEN=512, ELEN=32).
//
// Shared by the CPU (EX stage: precise illegal-instruction traps) and the vector unit
// (execution). Everything here is combinational on the instruction word and vtype.
//
// vtype: {vill, 23'b0, vma, vta, vsew[2:0], vlmul[2:0]}
module vec_decode (
    input  wire [31:0] instr,
    input  wire [31:0] vtype,
    input  wire        fp_enabled,      // mstatus.FS != Off (scalar FP and vector FP)
    input  wire [2:0]  frm,             // dynamic rounding mode (rm = 7 must name a valid one)
    output reg         is_vector,       // OP-V (not vsetvl), or a vector load/store
    output reg         legal,
    output reg  [4:0]  kind,
    output reg         is_load,
    output reg         is_store,
    output reg         scalar_dest,     // writes x[rd] (vmv.x.s, vcpop.m, vfirst.m)
    output reg         uses_rs1,        // reads x[rs1] (OPIVX/OPMVX, vector memory base)
    output reg         uses_rs2,        // reads x[rs2] (strided stride)
    output reg         uses_frs1,       // reads f[rs1] (OPFVF)
    output reg  [1:0]  eew_d,           // log2(bytes) of destination elements (mask dests: n/a)
    output reg  [1:0]  eew_s2,          // of vs2 elements
    output reg  [1:0]  eew_s1           // of vs1 elements
);
`include "vec_defines.vh"

    wire [6:0] opcode = instr[6:0];
    wire [2:0] funct3 = instr[14:12];
    wire [5:0] funct6 = instr[31:26];
    wire       vm     = instr[25];
    wire [4:0] vs2    = instr[24:20];
    wire [4:0] vs1    = instr[19:15];
    wire [4:0] vd     = instr[11:7];

    wire       vill  = vtype[31];
    wire [2:0] vsew  = vtype[5:3];
    wire [2:0] vlmul = vtype[2:0];
    wire [1:0] sew_l2 = vsew[1:0];               // 0:e8 1:e16 2:e32

    // LMUL as log2 in [-3, 3]
    wire signed [3:0] lmul_l2 = {vlmul[2], vlmul};

    // EMUL (log2) of a group whose EEW (log2 bytes) differs from SEW
    function signed [3:0] emul_l2;
        input [1:0] eew;
        begin
            emul_l2 = lmul_l2 + $signed({2'b00, eew}) - $signed({2'b00, sew_l2});
        end
    endfunction

    // Register group base must be aligned to EMUL when EMUL > 1.
    function aligned;
        input [4:0] r;
        input signed [3:0] l2;
        begin
            case (l2)
                4'sd1: aligned = (r[0] == 1'b0);
                4'sd2: aligned = (r[1:0] == 2'b0);
                4'sd3: aligned = (r[2:0] == 3'b0);
                default: aligned = 1'b1;
            endcase
        end
    endfunction

    function emul_ok;
        input signed [3:0] l2;
        begin
            emul_ok = (l2 >= -4'sd3) && (l2 <= 4'sd3);
        end
    endfunction

    // Number of registers spanned by a group (EMUL<1 counts as 1)
    function [3:0] nregs;
        input signed [3:0] l2;
        begin
            case (l2)
                4'sd1: nregs = 4'd2;
                4'sd2: nregs = 4'd4;
                4'sd3: nregs = 4'd8;
                default: nregs = 4'd1;
            endcase
        end
    endfunction

    // Groups [a, a+na) and [b, b+nb) overlap
    function overlap;
        input [4:0] a;
        input [3:0] na;
        input [4:0] b;
        input [3:0] nb;
        begin
            overlap = ({1'b0, a} < ({1'b0, b} + {2'b0, nb})) && ({1'b0, b} < ({1'b0, a} + {2'b0, na}));
        end
    endfunction

    // Memory access fields
    wire [2:0] width = funct3;
    wire       mew   = instr[28];
    wire [1:0] mop   = instr[27:26];
    wire [2:0] nf    = instr[31:29];
    wire [4:0] lumop = instr[24:20];
    reg  [1:0] mem_eew;
    reg        mem_width_ok;
    always @(*) begin
        mem_width_ok = 1'b1;
        case (width)
            3'b000: mem_eew = 2'd0;
            3'b101: mem_eew = 2'd1;
            3'b110: mem_eew = 2'd2;
            default: begin mem_eew = 2'd0; mem_width_ok = 1'b0; end
        endcase
    end

    // Static rm 0-4 are valid; 7 selects frm, which must itself be 0-4
    wire rm_ok = (funct3 <= 3'd4) || (funct3 == 3'd7 && frm <= 3'd4);
    reg signed [3:0] l2_d, l2_s2, l2_s1;
    reg chk_s2, chk_s1, chk_d;
    reg mask_dest;                           // destination is a mask register (1 reg)
    reg [7:0] seg_regs;                      // registers spanned by a segment access

    always @(*) begin
        is_vector = 1'b0;
        legal = 1'b0;
        kind = VK_NONE;
        is_load = 1'b0;
        is_store = 1'b0;
        scalar_dest = 1'b0;
        uses_rs1 = 1'b0;
        uses_rs2 = 1'b0;
        uses_frs1 = 1'b0;
        eew_d = sew_l2;
        eew_s2 = sew_l2;
        eew_s1 = sew_l2;
        chk_s2 = 1'b1;
        chk_s1 = 1'b0;
        chk_d = 1'b1;
        mask_dest = 1'b0;

        // ---------------- Scalar FP (RV32F / RV32D) ----------------
        if ((opcode == 7'b0000111 || opcode == 7'b0100111) && (width == 3'b010 || width == 3'b011)) begin
            is_vector = 1'b1;
            is_load = (opcode == 7'b0000111);
            is_store = !is_load;
            kind = is_load ? VK_SFLD : VK_SFST;
            uses_rs1 = 1'b1;
            legal = fp_enabled;
        end else if (opcode == 7'b1000011 || opcode == 7'b1000111 || opcode == 7'b1001011 ||
                     opcode == 7'b1001111) begin
            // fmadd / fmsub / fnmsub / fnmadd
            is_vector = 1'b1;
            kind = VK_SFP;
            legal = fp_enabled && (instr[26:25] == 2'b00 || instr[26:25] == 2'b01) && rm_ok;
        end else if (opcode == 7'b1010011) begin
            is_vector = 1'b1;
            kind = VK_SFP;
            legal = fp_enabled && (instr[26:25] == 2'b00 || instr[26:25] == 2'b01);
            case (instr[31:27])
                5'b00000, 5'b00001, 5'b00010, 5'b00011: legal = legal && rm_ok;          // add sub mul div
                5'b01011: legal = legal && rm_ok && (instr[24:20] == 5'd0);           // sqrt
                5'b00100: legal = legal && (funct3 <= 3'd2);                          // sgnj
                5'b00101: legal = legal && (funct3 <= 3'd1);                          // min/max
                5'b10100: begin                                                       // feq/flt/fle
                    kind = VK_SFPX; scalar_dest = 1'b1;
                    legal = legal && (funct3 <= 3'd2);
                end
                5'b11100: begin                                                       // fmv.x.w / fclass
                    kind = VK_SFPX; scalar_dest = 1'b1;
                    legal = legal && (instr[24:20] == 5'd0) &&
                            (funct3 == 3'd1 || (funct3 == 3'd0 && instr[26:25] == 2'b00));
                end
                5'b11000: begin                                                       // fcvt.w[u].fmt
                    kind = VK_SFPX; scalar_dest = 1'b1;
                    legal = legal && rm_ok && (instr[24:21] == 4'd0);
                end
                5'b11010: begin                                                       // fcvt.fmt.w[u]
                    uses_rs1 = 1'b1;
                    legal = legal && rm_ok && (instr[24:21] == 4'd0);
                end
                5'b11110: begin                                                       // fmv.w.x
                    uses_rs1 = 1'b1;
                    legal = legal && (funct3 == 3'd0) && (instr[24:20] == 5'd0) && (instr[26:25] == 2'b00);
                end
                5'b01000: legal = legal && rm_ok &&                                   // fcvt.s.d / fcvt.d.s
                                  ((instr[26:25] == 2'b00 && instr[24:20] == 5'd1) ||
                                   (instr[26:25] == 2'b01 && instr[24:20] == 5'd0));
                default: legal = 1'b0;
            endcase
        end

        // ---------------- Vector loads / stores ----------------
        else if ((opcode == 7'b0000111 || opcode == 7'b0100111) && mem_width_ok) begin
            is_vector = 1'b1;
            is_load = (opcode == 7'b0000111);
            is_store = !is_load;
            uses_rs1 = 1'b1;
            uses_rs2 = (mop == 2'b10);
            chk_s2 = 1'b0;
            if (mop == 2'b00 && lumop == 5'b01000) begin
                // Whole-register load/store: nf+1 in {1,2,4,8}, EEW from width, ignores vtype
                kind = VK_MEM_WHOLE;
                eew_d = mem_eew;
                legal = !mew && (nf == 3'd0 || nf == 3'd1 || nf == 3'd3 || nf == 3'd7) &&
                        (nf == 3'd0 || (vd & {2'b0, nf}) == 5'b0) &&
                        (is_load || mem_eew == 2'd0);
                chk_d = 1'b0;
            end else if (mop == 2'b00 && lumop == 5'b01011) begin
                // vlm.v / vsm.v: EEW=8, evl = ceil(vl/8), always unmasked
                kind = VK_MEM_MASK;
                eew_d = 2'd0;
                legal = !mew && !vill && (nf == 3'd0) && vm && (mem_eew == 2'd0);
                chk_d = 1'b0;
            end else begin
                kind = (mop == 2'b00) ? VK_MEM_UNIT : (mop == 2'b10) ? VK_MEM_STRIDED : VK_MEM_INDEXED;
                if (kind == VK_MEM_INDEXED) begin
                    // Data EEW = SEW, index EEW = width
                    eew_d = sew_l2;
                    eew_s2 = mem_eew;
                    chk_s2 = 1'b1;
                end else begin
                    eew_d = mem_eew;
                end
                legal = !mew && !vill &&
                        ((mop != 2'b00) || lumop == 5'b00000 || (lumop == 5'b10000 && is_load));
            end
        end

        // ---------------- OP-V arithmetic ----------------
        else if (opcode == 7'b1010111 && funct3 != 3'b111) begin
            is_vector = 1'b1;
            uses_rs1 = (funct3 == F3_OPIVX) || (funct3 == F3_OPMVX);
            uses_frs1 = (funct3 == F3_OPFVF);
            chk_s1 = (funct3 == F3_OPIVV) || (funct3 == F3_OPMVV) || (funct3 == F3_OPFVV);
            kind = VK_ELEM;
            legal = !vill;
            case (funct3)
                F3_OPIVV, F3_OPIVX, F3_OPIVI: begin
                    case (funct6)
                        6'b000000, 6'b000010, 6'b000011, 6'b000100, 6'b000101, 6'b000110,
                        6'b000111, 6'b001001, 6'b001010, 6'b001011, 6'b100000, 6'b100001,
                        6'b100010, 6'b100011, 6'b100101, 6'b100111, 6'b101000, 6'b101001,
                        6'b101010, 6'b101011: begin
                            // add sub rsub minu min maxu max and or xor sadd(u) ssub(u) sll smul
                            // srl sra ssrl ssra (and vmv<nr>r at 100111 for OPIVI)
                            if (funct6 == 6'b000010 && funct3 == F3_OPIVI) legal = 1'b0; // no vsub.vi
                            if (funct6 == 6'b000011 && funct3 == F3_OPIVV) legal = 1'b0; // no vrsub.vv
                            if ((funct6 == 6'b000100 || funct6 == 6'b000101 || funct6 == 6'b000110 ||
                                 funct6 == 6'b000111) && funct3 == F3_OPIVI) legal = 1'b0;    // no min/max .vi
                            if ((funct6 == 6'b100010 || funct6 == 6'b100011) && funct3 == F3_OPIVI) legal = 1'b0;
                            if (funct6 == 6'b100111) begin
                                if (funct3 == F3_OPIVI) begin
                                    // vmv<nr>r.v: nr = simm+1 in {1,2,4,8}, unmasked, ignores vtype
                                    kind = VK_VMVNR;
                                    legal = vm && (vs1 == 5'd0 || vs1 == 5'd1 || vs1 == 5'd3 || vs1 == 5'd7) &&
                                            ((vd & vs1) == 5'b0) && ((vs2 & vs1) == 5'b0);
                                    chk_d = 1'b0;
                                    chk_s2 = 1'b0;
                                end else if (funct3 == F3_OPIVX) begin
                                    legal = !vill;  // vsmul.vx
                                end else begin
                                    legal = !vill;  // vsmul.vv
                                end
                            end
                        end
                        6'b001100: kind = VK_GATHER;            // vrgather
                        6'b001110: begin                         // vrgatherei16.vv / vslideup
                            if (funct3 == F3_OPIVV) begin
                                kind = VK_GATHER16;
                                eew_s1 = 2'd1;
                            end else kind = VK_SLIDEUP;
                        end
                        6'b001111: begin
                            kind = VK_SLIDEDOWN;
                            if (funct3 == F3_OPIVV) legal = 1'b0;
                        end
                        6'b010000, 6'b010010: begin             // vadc, vsbc (vm must be 0)
                            legal = !vill && !vm && (vd != 5'd0);
                            if (funct6 == 6'b010010 && funct3 == F3_OPIVI) legal = 1'b0;
                        end
                        6'b010001, 6'b010011: begin             // vmadc, vmsbc -> mask
                            kind = VK_MASKDEST;
                            mask_dest = 1'b1;
                            if (funct6 == 6'b010011 && funct3 == F3_OPIVI) legal = 1'b0;
                        end
                        6'b010111: begin                         // vmerge / vmv.v.*
                            if (vm && vs2 != 5'd0) legal = 1'b0;
                            if (!vm && vd == 5'd0) legal = 1'b0;
                        end
                        6'b011000, 6'b011001, 6'b011010, 6'b011011, 6'b011100, 6'b011101,
                        6'b011110, 6'b011111: begin             // compares -> mask
                            kind = VK_MASKDEST;
                            mask_dest = 1'b1;
                            if ((funct6 == 6'b011010 || funct6 == 6'b011011) && funct3 == F3_OPIVI) legal = 1'b0;
                            if ((funct6 == 6'b011110 || funct6 == 6'b011111) && funct3 == F3_OPIVV) legal = 1'b0;
                        end
                        6'b101100, 6'b101101, 6'b101110, 6'b101111: begin
                            // vnsrl vnsra vnclipu vnclip: vs2 is 2*SEW
                            kind = VK_NARROW;
                            eew_s2 = sew_l2 + 2'd1;
                            if (sew_l2 == 2'd2) legal = 1'b0;
                        end
                        6'b110000, 6'b110001: begin             // vwredsumu, vwredsum (OPIVV only)
                            kind = VK_WRED;
                            eew_d = sew_l2 + 2'd1;
                            if (funct3 != F3_OPIVV || sew_l2 == 2'd2) legal = 1'b0;
                            chk_s1 = 1'b0;
                            chk_d = 1'b0;
                        end
                        default: legal = 1'b0;
                    endcase
                end
                F3_OPMVV, F3_OPMVX: begin
                    case (funct6)
                        6'b000000, 6'b000001, 6'b000010, 6'b000011, 6'b000100, 6'b000101,
                        6'b000110, 6'b000111: begin             // reductions
                            kind = VK_RED;
                            if (funct3 != F3_OPMVV) legal = 1'b0;
                            chk_s1 = 1'b0;
                            chk_d = 1'b0;
                        end
                        6'b001000, 6'b001001, 6'b001010, 6'b001011: ;   // vaaddu vaadd vasubu vasub
                        6'b001110: begin kind = VK_SLIDEUP; if (funct3 != F3_OPMVX) legal = 1'b0; end   // vslide1up
                        6'b001111: begin kind = VK_SLIDEDOWN; if (funct3 != F3_OPMVX) legal = 1'b0; end // vslide1down
                        6'b010000: begin
                            if (funct3 == F3_OPMVV) begin
                                // VWXUNARY0: vmv.x.s (vs1=0), vcpop.m (16), vfirst.m (17)
                                kind = VK_XUNARY;
                                scalar_dest = 1'b1;
                                chk_d = 1'b0;
                                chk_s2 = 1'b0;
                                chk_s1 = 1'b0;
                                if (!(vs1 == 5'd0 && vm) && vs1 != 5'd16 && vs1 != 5'd17) legal = 1'b0;
                            end else begin
                                // VRXUNARY0: vmv.s.x (vs2=0, unmasked)
                                kind = VK_SUNARY;
                                chk_d = 1'b0;
                                chk_s2 = 1'b0;
                                if (vs2 != 5'd0 || !vm) legal = 1'b0;
                            end
                        end
                        6'b010010: begin
                            // VXUNARY0: vzext/vsext vf2 (6,7), vf4 (4,5)
                            kind = VK_EXT;
                            chk_s1 = 1'b0;
                            if (funct3 != F3_OPMVV) legal = 1'b0;
                            if (vs1 == 5'd6 || vs1 == 5'd7) begin
                                if (sew_l2 == 2'd0) legal = 1'b0;
                                eew_s2 = sew_l2 - 2'd1;
                            end else if (vs1 == 5'd4 || vs1 == 5'd5) begin
                                if (sew_l2 != 2'd2) legal = 1'b0;
                                eew_s2 = 2'd0;
                            end else legal = 1'b0;
                        end
                        6'b010100: begin
                            // VMUNARY0: vmsbf(1) vmsof(2) vmsif(3) viota(16) vid(17)
                            chk_s1 = 1'b0;
                            if (funct3 != F3_OPMVV) legal = 1'b0;
                            if (vs1 == 5'd1 || vs1 == 5'd2 || vs1 == 5'd3) begin
                                kind = VK_MSETBIT;
                                mask_dest = 1'b1;
                                chk_s2 = 1'b0;
                                if (vd == vs2) legal = 1'b0;
                                if (!vm && vd == 5'd0) legal = 1'b0;
                            end else if (vs1 == 5'd16) begin
                                kind = VK_IOTA;
                                chk_s2 = 1'b0;
                                if (!vm && vd == 5'd0) legal = 1'b0;
                            end else if (vs1 == 5'd17) begin
                                kind = VK_VID;
                                chk_s2 = 1'b0;
                                if (vs2 != 5'd0) legal = 1'b0;
                                if (!vm && vd == 5'd0) legal = 1'b0;
                            end else legal = 1'b0;
                        end
                        6'b010111: begin                         // vcompress.vm (unmasked)
                            kind = VK_COMPRESS;
                            chk_s1 = 1'b0;
                            if (funct3 != F3_OPMVV || !vm) legal = 1'b0;
                        end
                        6'b011000, 6'b011001, 6'b011010, 6'b011011, 6'b011100, 6'b011101,
                        6'b011110, 6'b011111: begin             // mask logic (unmasked)
                            kind = VK_MASKLOGIC;
                            mask_dest = 1'b1;
                            chk_s1 = 1'b0;
                            chk_s2 = 1'b0;
                            if (funct3 != F3_OPMVV || !vm) legal = 1'b0;
                        end
                        6'b100000, 6'b100001, 6'b100010, 6'b100011: ;   // divu div remu rem
                        6'b100100, 6'b100101, 6'b100110, 6'b100111: ;   // mulhu mul mulhsu mulh
                        6'b101001, 6'b101011, 6'b101101, 6'b101111: ;   // vmadd vnmsub vmacc vnmsac
                        6'b110000, 6'b110001, 6'b110010, 6'b110011, 6'b111000, 6'b111010,
                        6'b111011, 6'b111100, 6'b111101, 6'b111110, 6'b111111: begin
                            // widening: waddu wadd wsubu wsub wmulu wmulsu wmul wmaccu wmacc wmaccus wmaccsu
                            kind = VK_WIDEN;
                            eew_d = sew_l2 + 2'd1;
                            if (sew_l2 == 2'd2) legal = 1'b0;
                            if (funct6 == 6'b111110 && funct3 == F3_OPMVV) legal = 1'b0; // wmaccus .vx only
                        end
                        6'b110100, 6'b110101, 6'b110110, 6'b110111: begin
                            // waddu.w wadd.w wsubu.w wsub.w: vs2 already 2*SEW
                            kind = VK_WIDEN;
                            eew_d = sew_l2 + 2'd1;
                            eew_s2 = sew_l2 + 2'd1;
                            if (sew_l2 == 2'd2) legal = 1'b0;
                        end
                        default: legal = 1'b0;
                    endcase
                end
                F3_OPFVV, F3_OPFVF: begin
                    // FP32 only (Zve32f): SEW must be 32 except the int16<->f32 conversions.
                    if (!fp_enabled || !(frm <= 3'd4)) legal = 1'b0;
                    case (funct6)
                        6'b000000, 6'b000010, 6'b000100, 6'b000110, 6'b001000, 6'b001001,
                        6'b001010, 6'b100000, 6'b100001, 6'b100100, 6'b100111,
                        6'b101000, 6'b101001, 6'b101010, 6'b101011, 6'b101100, 6'b101101,
                        6'b101110, 6'b101111: begin
                            // fadd fsub fmin fmax fsgnj fsgnjn fsgnjx fdiv frdiv fmul frsub
                            // fmadd fnmadd fmsub fnmsub fmacc fnmacc fmsac fnmsac
                            if (sew_l2 != 2'd2) legal = 1'b0;
                            if ((funct6 == 6'b100001 || funct6 == 6'b100111) && funct3 == F3_OPFVV) legal = 1'b0;
                        end
                        6'b000001, 6'b000011, 6'b000101, 6'b000111: begin
                            // fredusum fredosum fredmin fredmax
                            kind = VK_RED;
                            if (sew_l2 != 2'd2 || funct3 != F3_OPFVV) legal = 1'b0;
                            chk_s1 = 1'b0;
                            chk_d = 1'b0;
                        end
                        6'b001110: begin kind = VK_SLIDEUP; if (funct3 != F3_OPFVF || sew_l2 != 2'd2) legal = 1'b0; end
                        6'b001111: begin kind = VK_SLIDEDOWN; if (funct3 != F3_OPFVF || sew_l2 != 2'd2) legal = 1'b0; end
                        6'b010000: begin
                            if (sew_l2 != 2'd2) legal = 1'b0;
                            if (funct3 == F3_OPFVV) begin
                                // vfmv.f.s: writes f[rd] inside the coprocessor
                                kind = VK_XUNARY;
                                chk_d = 1'b0; chk_s2 = 1'b0; chk_s1 = 1'b0;
                                if (vs1 != 5'd0 || !vm || !fp_enabled) legal = 1'b0;
                            end else begin
                                kind = VK_SUNARY;  // vfmv.s.f
                                chk_d = 1'b0; chk_s2 = 1'b0;
                                if (vs2 != 5'd0 || !vm) legal = 1'b0;
                            end
                        end
                        6'b010111: begin                         // vfmerge.vfm / vfmv.v.f
                            if (funct3 != F3_OPFVF || sew_l2 != 2'd2) legal = 1'b0;
                            if (vm && vs2 != 5'd0) legal = 1'b0;
                            if (!vm && vd == 5'd0) legal = 1'b0;
                        end
                        6'b011000, 6'b011001, 6'b011011, 6'b011100, 6'b011101, 6'b011111: begin
                            // mfeq mfle mflt mfne mfgt mfge -> mask
                            kind = VK_MASKDEST;
                            mask_dest = 1'b1;
                            if (sew_l2 != 2'd2) legal = 1'b0;
                            if ((funct6 == 6'b011101 || funct6 == 6'b011111) && funct3 == F3_OPFVV) legal = 1'b0;
                        end
                        6'b010010: begin
                            // VFUNARY0 conversions
                            chk_s1 = 1'b0;
                            if (funct3 != F3_OPFVV) legal = 1'b0;
                            case (vs1)
                                5'd0, 5'd1, 5'd2, 5'd3, 5'd6, 5'd7: if (sew_l2 != 2'd2) legal = 1'b0; // vfcvt.*
                                5'd10, 5'd11: begin                // vfwcvt.f.xu.v / vfwcvt.f.x.v: int16 -> f32
                                    kind = VK_WIDEN;
                                    eew_d = sew_l2 + 2'd1;
                                    if (sew_l2 != 2'd1) legal = 1'b0;
                                end
                                5'd16, 5'd17, 5'd22, 5'd23: begin  // vfncvt.xu/x.f.w, rtz: f32 -> int16
                                    kind = VK_NARROW;
                                    eew_s2 = sew_l2 + 2'd1;
                                    if (sew_l2 != 2'd1) legal = 1'b0;
                                end
                                default: legal = 1'b0;
                            endcase
                        end
                        6'b010011: begin
                            // VFUNARY1: vfsqrt(0) vfrsqrt7(4) vfrec7(5) vfclass(16)
                            chk_s1 = 1'b0;
                            if (funct3 != F3_OPFVV || sew_l2 != 2'd2) legal = 1'b0;
                            if (vs1 != 5'd0 && vs1 != 5'd4 && vs1 != 5'd5 && vs1 != 5'd16) legal = 1'b0;
                        end
                        default: legal = 1'b0;
                    endcase
                end
                default: legal = 1'b0;
            endcase
        end

        // ---------------- Register group checks ----------------
        l2_d = emul_l2(eew_d);
        seg_regs = ({5'b0, nf} + 8'd1) * {4'b0, nregs(l2_d)};
        l2_s2 = emul_l2(eew_s2);
        l2_s1 = emul_l2(eew_s1);
        if (is_vector && legal && kind != VK_VMVNR && kind != VK_MEM_WHOLE && kind < VK_SFP) begin
            if (kind == VK_MEM_UNIT || kind == VK_MEM_STRIDED || kind == VK_MEM_INDEXED) begin
                // Segment group: (nf+1) * EMUL <= 8 and fits in v0..v31
                if (!emul_ok(l2_d) || !aligned(vd, l2_d)) legal = 1'b0;
                if (seg_regs > 8'd8) legal = 1'b0;
                if (({3'b0, vd} + seg_regs) > 8'd32) legal = 1'b0;
                if (!vm && vd == 5'd0 && is_load) legal = 1'b0;
                if (kind == VK_MEM_INDEXED) begin
                    if (!emul_ok(l2_s2) || !aligned(vs2, l2_s2)) legal = 1'b0;
                    // A load's data group may not overlap its index group when EEW differs
                    if (is_load && eew_s2 != eew_d && overlap(vd, seg_regs[3:0], vs2, nregs(l2_s2)))
                        legal = 1'b0;
                end
            end else begin
                if (chk_d && !mask_dest && (!emul_ok(l2_d) || !aligned(vd, l2_d))) legal = 1'b0;
                if (chk_s2 && kind != VK_MASKLOGIC && (!emul_ok(l2_s2) || !aligned(vs2, l2_s2))) legal = 1'b0;
                if (chk_s1 && (!emul_ok(l2_s1) || !aligned(vs1, l2_s1))) legal = 1'b0;
                // A masked op may not write v0 unless the result is a mask (or reduction scalar)
                if (!vm && !mask_dest && chk_d && vd == 5'd0 &&
                    kind != VK_RED && kind != VK_WRED) legal = 1'b0;
                // Widening dest may not overlap a narrower source; narrowing dest may not overlap vs2
                if (kind == VK_WIDEN) begin
                    if (eew_s2 != eew_d && overlap(vd, nregs(l2_d), vs2, nregs(l2_s2))) legal = 1'b0;
                    if (chk_s1 && overlap(vd, nregs(l2_d), vs1, nregs(l2_s1))) legal = 1'b0;
                end
                if (kind == VK_NARROW && vd != vs2 && overlap(vd, nregs(l2_d), vs2, nregs(l2_s2))) legal = 1'b0;
                // Slide-up / gather / compress dest may not overlap sources
                if (kind == VK_SLIDEUP || kind == VK_GATHER || kind == VK_GATHER16 || kind == VK_COMPRESS ||
                    kind == VK_IOTA) begin
                    if (overlap(vd, nregs(l2_d), vs2, (kind == VK_IOTA) ? 4'd1 : nregs(l2_s2))) legal = 1'b0;
                    if ((kind == VK_GATHER || kind == VK_GATHER16) && chk_s1 &&
                        overlap(vd, nregs(l2_d), vs1, nregs(l2_s1))) legal = 1'b0;
                    if (kind == VK_COMPRESS && overlap(vd, nregs(l2_d), vs1, 4'd1)) legal = 1'b0;
                end
                // Mask-producing compares with LMUL>1: dest may only overlap a source at its lowest register
                if (mask_dest && kind == VK_MASKDEST && nregs(l2_s2) > 4'd1) begin
                    if (overlap(vd, 4'd1, vs2, nregs(l2_s2)) && vd != vs2) legal = 1'b0;
                    if (chk_s1 && overlap(vd, 4'd1, vs1, nregs(l2_s1)) && vd != vs1) legal = 1'b0;
                end
            end
        end
        if (!is_vector) legal = 1'b0;
    end

endmodule
