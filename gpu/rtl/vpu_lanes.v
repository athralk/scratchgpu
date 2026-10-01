`default_nettype none
// TinyGPU v2 SIMD lanes: the fast path for SEW=32 elementwise integer and FP32 operations.
//
// LANES words of a 512-bit register per cycle (LANES = 8 by default: a register takes two
// beats; 16 does it in one at roughly twice the LUTs). Each lane has an integer ALU, a
// 33x33 multiplier (DSPs) and a compact FP32 fused multiply-add. Two register stages:
// operands in, results out, so a beat issued in cycle t is written back in cycle t+2.
//
// Operation encoding is the instruction's own funct3/funct6 (see vpu_top fast_op).
module vpu_lanes #(
    parameter LANES = 8
) (
    input  wire                  clk,
    input  wire                  rst,
    // Beat in
    input  wire                  in_valid,
    input  wire [2:0]            funct3,
    input  wire [5:0]            funct6,
    input  wire [2:0]            frm,
    input  wire [32*LANES-1:0]   a,         // vs2 words
    input  wire [32*LANES-1:0]   b,         // vs1 words (scalar already broadcast)
    input  wire [32*LANES-1:0]   d,         // old vd words
    input  wire [LANES-1:0]      active,    // element is written
    input  wire [LANES-1:0]      sel,       // vmerge: take b (mask bit)
    input  wire [4:0]            wreg_in,
    input  wire [15:0]           wword_in,  // first word index of this beat (bank offset)
    // Beat out (two cycles later)
    output reg                   out_valid,
    output reg  [32*LANES-1:0]   result,
    output reg  [LANES-1:0]      out_active,
    output reg  [4:0]            wreg_out,
    output reg  [15:0]           wword_out,
    output reg  [4:0]            fflags     // OR over active lanes
);
    localparam [2:0] F3_OPIVV = 3'b000, F3_OPFVV = 3'b001, F3_OPMVV = 3'b010, F3_OPIVI = 3'b011,
                     F3_OPIVX = 3'b100, F3_OPFVF = 3'b101, F3_OPMVX = 3'b110;

    // ---------------- Stage 1: operand registers ----------------
    reg                  s1_valid;
    reg [2:0]            s1_f3;
    reg [5:0]            s1_f6;
    reg [2:0]            s1_rm;
    reg [32*LANES-1:0]   s1_a, s1_b, s1_d;
    reg [LANES-1:0]      s1_act, s1_sel;
    reg [4:0]            s1_wreg;
    reg [15:0]           s1_wword;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            s1_valid <= 1'b0;
        end else begin
            s1_valid <= in_valid;
            if (in_valid) begin
                s1_f3 <= funct3;
                s1_f6 <= funct6;
                s1_rm <= frm;
                s1_a <= a;
                s1_b <= b;
                s1_d <= d;
                s1_act <= active;
                s1_sel <= sel;
                s1_wreg <= wreg_in;
                s1_wword <= wword_in;
            end
        end
    end

    wire is_fp = (s1_f3 == F3_OPFVV) || (s1_f3 == F3_OPFVF);
    wire is_mv = (s1_f3 == F3_OPMVV) || (s1_f3 == F3_OPMVX);

    // ---------------- Lanes ----------------
    wire [32*LANES-1:0] lane_res;
    wire [5*LANES-1:0]  lane_flags;

    genvar l;
    generate
        for (l = 0; l < LANES; l = l + 1) begin : lane
            wire [31:0] va = s1_a[32*l +: 32];
            wire [31:0] vb = s1_b[32*l +: 32];
            wire [31:0] vd = s1_d[32*l +: 32];

            // Integer: one 33x33 signed multiplier covers mul/mulh/mulhu/mulhsu and the
            // multiply-adds (low 32 bits of a signed and an unsigned product are the same).
            reg  [32:0] mx, my;
            always @(*) begin
                mx = {va[31], va};
                my = {vb[31], vb};
                case (s1_f6)
                    6'b100100: begin mx = {1'b0, va}; my = {1'b0, vb}; end   // vmulhu
                    6'b100110: begin mx = {va[31], va}; my = {1'b0, vb}; end // vmulhsu (vs2 signed)
                    6'b101001, 6'b101011: begin mx = {vb[31], vb}; my = {vd[31], vd}; end  // vmadd/vnmsub: vs1*vd
                    default: ;
                endcase
            end
            wire signed [65:0] prod = $signed(mx) * $signed(my);

            reg [31:0] ires;
            always @(*) begin
                ires = 32'b0;
                if (!is_mv) begin
                    case (s1_f6)
                        6'b000000: ires = va + vb;
                        6'b000010: ires = va - vb;
                        6'b000011: ires = vb - va;
                        6'b000100: ires = (va < vb) ? va : vb;
                        6'b000101: ires = ($signed(va) < $signed(vb)) ? va : vb;
                        6'b000110: ires = (va > vb) ? va : vb;
                        6'b000111: ires = ($signed(va) > $signed(vb)) ? va : vb;
                        6'b001001: ires = va & vb;
                        6'b001010: ires = va | vb;
                        6'b001011: ires = va ^ vb;
                        6'b100101: ires = va << vb[4:0];
                        6'b101000: ires = va >> vb[4:0];
                        6'b101001: ires = $signed(va) >>> vb[4:0];
                        6'b010111: ires = s1_sel[l] ? vb : va;            // vmerge / vmv.v
                        default: ires = 32'b0;
                    endcase
                end else begin
                    case (s1_f6)
                        6'b100101: ires = prod[31:0];                    // vmul
                        6'b100100, 6'b100110, 6'b100111: ires = prod[63:32];  // vmulhu / vmulhsu / vmulh
                        6'b101001: ires = prod[31:0] + va;               // vmadd:  vs1*vd + vs2
                        6'b101011: ires = va - prod[31:0];               // vnmsub: -(vs1*vd) + vs2
                        6'b101101: ires = prod[31:0] + vd;               // vmacc:  vs1*vs2 + vd
                        6'b101111: ires = vd - prod[31:0];               // vnmsac: -(vs1*vs2) + vd
                        default: ires = 32'b0;
                    endcase
                end
            end

            // FP32: fused multiply-add for add/sub/mul/fma forms; min/max/sgnj directly
            reg [31:0] fx, fy, fz;
            always @(*) begin
                fx = va; fy = 32'h3F800000; fz = vb;
                case (s1_f6)
                    6'b000010: begin fx = va; fy = 32'h3F800000; fz = {~vb[31], vb[30:0]}; end  // vfsub
                    6'b100111: begin fx = vb; fy = 32'h3F800000; fz = {~va[31], va[30:0]}; end  // vfrsub
                    6'b100100: begin fx = va; fy = vb; fz = 32'h80000000; end                   // vfmul
                    6'b101000: begin fx = vb; fy = vd; fz = va; end                             // vfmadd
                    6'b101001: begin fx = {~vb[31], vb[30:0]}; fy = vd; fz = {~va[31], va[30:0]}; end
                    6'b101010: begin fx = vb; fy = vd; fz = {~va[31], va[30:0]}; end
                    6'b101011: begin fx = {~vb[31], vb[30:0]}; fy = vd; fz = va; end
                    6'b101100: begin fx = vb; fy = va; fz = vd; end                             // vfmacc
                    6'b101101: begin fx = {~vb[31], vb[30:0]}; fy = va; fz = {~vd[31], vd[30:0]}; end
                    6'b101110: begin fx = vb; fy = va; fz = {~vd[31], vd[30:0]}; end
                    6'b101111: begin fx = {~vb[31], vb[30:0]}; fy = va; fz = vd; end
                    default: ;
                endcase
            end
            wire [31:0] fma_res;
            wire [4:0]  fma_fl;
            fp32_fma fma (.a(fx), .b(fy), .c(fz), .rm(s1_rm), .mul(s1_f6 == 6'b100100),
                         .result(fma_res), .flags(fma_fl));

            // min/max (IEEE 754-2019 minimumNumber/maximumNumber) and sign injection
            wire a_nan = (va[30:23] == 8'hFF) && (va[22:0] != 0);
            wire b_nan = (vb[30:23] == 8'hFF) && (vb[22:0] != 0);
            wire mm_nv = (a_nan && !va[22]) || (b_nan && !vb[22]);
            wire a_lt = (va[31] != vb[31]) ? va[31] :
                        va[31] ? (va[30:0] > vb[30:0]) : (va[30:0] < vb[30:0]);
            wire [31:0] fmin = (a_nan && b_nan) ? 32'h7FC00000 : a_nan ? vb : b_nan ? va : (a_lt ? va : vb);
            wire [31:0] fmax = (a_nan && b_nan) ? 32'h7FC00000 : a_nan ? vb : b_nan ? va : (a_lt ? vb : va);

            reg [31:0] fres;
            reg [4:0]  ffl;
            always @(*) begin
                ffl = 5'b0;
                case (s1_f6)
                    6'b000100: begin fres = fmin; ffl = {mm_nv, 4'b0}; end
                    6'b000110: begin fres = fmax; ffl = {mm_nv, 4'b0}; end
                    6'b001000: fres = {vb[31], va[30:0]};
                    6'b001001: fres = {~vb[31], va[30:0]};
                    6'b001010: fres = {va[31] ^ vb[31], va[30:0]};
                    default: begin fres = fma_res; ffl = fma_fl; end
                endcase
            end

            assign lane_res[32*l +: 32] = is_fp ? fres : ires;
            assign lane_flags[5*l +: 5] = (is_fp && s1_act[l]) ? ffl : 5'b0;
        end
    endgenerate

    // ---------------- Stage 2: result registers ----------------
    integer k;
    reg [4:0] fl_or;
    always @(*) begin
        fl_or = 5'b0;
        for (k = 0; k < LANES; k = k + 1) fl_or = fl_or | lane_flags[5*k +: 5];
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            out_valid <= 1'b0;
            fflags <= 5'b0;
        end else begin
            out_valid <= s1_valid;
            fflags <= s1_valid ? fl_or : 5'b0;
            if (s1_valid) begin
                result <= lane_res;
                out_active <= s1_act;
                wreg_out <= s1_wreg;
                wword_out <= s1_wword;
            end
        end
    end

endmodule
