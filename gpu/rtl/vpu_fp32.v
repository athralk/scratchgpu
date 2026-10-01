`default_nettype none
// FP32 arithmetic for the vector unit (RVV OPFVV/OPFVF), IEEE 754 with RISC-V NaN rules.
//
// Everything is built on one exact fused multiply-add (fadd = a*1+b, fmul = a*b+(-0)), with a
// single round/pack stage shared by FMA, int->float conversion and divide/sqrt results.
// Bit-exact against Spike/softfloat, including flags (tininess detected after rounding).
// Divide and square root are iterative (fp32_divsqrt below); this module only packs them.
//
// Operand roles follow the instruction: a = vs2, b = vs1 / f[rs1] (or the reduction
// accumulator), d = old vd (for multiply-add forms).
module vpu_fp32 (
    input  wire [5:0]  funct6,
    input  wire [4:0]  vs1_field,     // sub-opcode for VFUNARY0/1
    input  wire        is_vf,
    input  wire [1:0]  sew,
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire [31:0] d,
    input  wire [2:0]  rm,
    output reg  [31:0] result,
    output reg         result_bit,    // compares
    output reg  [4:0]  flags          // NV DZ OF UF NX
);
    localparam [31:0] QNAN = 32'h7FC00000;
    localparam [31:0] ONE = 32'h3F800000;
    localparam [31:0] NEG_ZERO = 32'h80000000;

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    function is_nan;  input [31:0] x; begin is_nan = (x[30:23] == 8'hFF) && (x[22:0] != 0); end endfunction
    function is_snan; input [31:0] x; begin is_snan = is_nan(x) && !x[22]; end endfunction
    function is_inf;  input [31:0] x; begin is_inf = (x[30:23] == 8'hFF) && (x[22:0] == 0); end endfunction
    function is_zero; input [31:0] x; begin is_zero = (x[30:0] == 0); end endfunction

    function [9:0] fclass;
        input [31:0] x;
        begin
            fclass = 10'b0;
            if (is_inf(x))                fclass[x[31] ? 0 : 7] = 1'b1;
            else if (is_nan(x))           fclass[x[22] ? 9 : 8] = 1'b1;
            else if (is_zero(x))          fclass[x[31] ? 3 : 4] = 1'b1;
            else if (x[30:23] == 8'h00)   fclass[x[31] ? 2 : 5] = 1'b1;
            else                          fclass[x[31] ? 1 : 6] = 1'b1;
        end
    endfunction

    // Round-up decision for the magnitude, given guard/sticky and the result sign
    function round_up;
        input [2:0] mode;
        input s, lsb, g, st;
        begin
            case (mode)
                3'd0: round_up = g && (st || lsb);   // RNE
                3'd1: round_up = 1'b0;               // RTZ
                3'd2: round_up = (g || st) && s;     // RDN
                3'd3: round_up = (g || st) && !s;    // RUP
                3'd4: round_up = g;                  // RMM
                default: round_up = 1'b0;
            endcase
        end
    endfunction

    // float -> integer (32 or 16 bits), signed or unsigned. Returns {flags, value}.
    // |x| >= 2^32 (exponent >= 159) is always out of range; otherwise the integer part fits
    // 32 bits: X = m * 2^32 shifted right by (182 - e) leaves it, with guard and sticky below.
    function [36:0] f2i;
        input [31:0] x;
        input is_signed;
        input narrow;
        input [2:0] mode;
        reg [7:0] e;
        reg [23:0] m;
        reg [63:0] X;
        reg [32:0] I;
        reg g, st, inc, s, inv;
        reg [8:0] r;
        reg [32:0] maxpos, maxneg;
        reg [31:0] v;
        begin
            s = x[31];
            e = x[30:23];
            m = {(e != 0), x[22:0]};
            maxpos = is_signed ? (narrow ? 33'h7FFF : 33'h7FFFFFFF) : (narrow ? 33'hFFFF : 33'hFFFFFFFF);
            maxneg = is_signed ? (narrow ? 33'h8000 : 33'h80000000) : 33'h0;
            X = {8'b0, m, 32'b0};
            r = 9'd182 - {1'b0, ((e == 0) ? 8'd1 : e)};
            if (e >= 8'd159) begin I = 33'h100000000; g = 0; st = 0; end
            else if (r >= 9'd64) begin I = 33'd0; g = 0; st = (m != 0); end
            else begin
                I = {1'b0, X[63:0] >> r};
                g = X[r - 1];
                st = (r >= 9'd2) ? ((X & ((64'd1 << (r - 1)) - 64'd1)) != 0) : 1'b0;
            end
            inc = round_up(mode, s, I[0], g, st);
            I = I + {32'b0, inc};
            inv = 1'b0;
            if (!s && I > maxpos) inv = 1'b1;
            if (s && is_signed && I > maxneg) inv = 1'b1;
            if (s && !is_signed && I != 0) inv = 1'b1;
            if (is_nan(x)) begin
                f2i = {1'b1, 4'b0, maxpos[31:0]};
            end else if (inv || is_inf(x)) begin
                v = s ? (narrow ? {16'b0, maxneg[15:0]} : maxneg[31:0]) : maxpos[31:0];
                f2i = {1'b1, 4'b0, v};
            end else begin
                v = s ? (32'd0 - I[31:0]) : I[31:0];
                if (narrow) v = {16'b0, v[15:0]};
                f2i = {4'b0, (g || st), v};
            end
        end
    endfunction

    // integer -> float: normalize a 32-bit magnitude, round to 24 bits (never tiny/overflow)
    function [36:0] i2f;
        input [31:0] x;
        input is_signed;
        input [2:0] mode;
        reg s;
        reg [31:0] mag, nrm;
        reg [4:0] lz;
        reg [24:0] mr;
        reg g, st, inc;
        reg [7:0] ex;
        integer k;
        begin
            s = is_signed && x[31];
            mag = s ? (32'd0 - x) : x;
            lz = 5'd0;
            for (k = 0; k < 32; k = k + 1) if (mag[k]) lz = 5'd31 - k[4:0];
            nrm = mag << lz;
            g = nrm[7];
            st = (nrm[6:0] != 0);
            inc = round_up(mode, s, nrm[8], g, st);
            mr = {1'b0, nrm[31:8]} + {24'b0, inc};
            ex = 8'd158 - {3'b0, lz} + (mr[24] ? 8'd1 : 8'd0);
            if (mag == 0) i2f = {5'b0, 32'b0};
            else i2f = {4'b0, (g || st), s, ex, mr[24] ? mr[23:1] : mr[22:0]};
        end
    endfunction

    // min / max (IEEE 754-2019 minimumNumber / maximumNumber)
    function [36:0] fminmax;
        input [31:0] x, y;
        input want_max;
        reg nv, lt;
        begin
            nv = is_snan(x) || is_snan(y);
            if (is_nan(x) && is_nan(y)) fminmax = {nv, 4'b0, QNAN};
            else if (is_nan(x)) fminmax = {nv, 4'b0, y};
            else if (is_nan(y)) fminmax = {nv, 4'b0, x};
            else begin
                // x < y, with -0 < +0
                if (x[31] != y[31]) lt = x[31];
                else lt = x[31] ? (x[30:0] > y[30:0]) : (x[30:0] < y[30:0]);
                fminmax = {nv, 4'b0, (want_max ? !lt : lt) ? x : y};
            end
        end
    endfunction

    // x < y / x <= y / x == y on non-NaN operands
    function flt;
        input [31:0] x, y;
        begin
            if (is_zero(x) && is_zero(y)) flt = 1'b0;
            else if (x[31] != y[31]) flt = x[31];
            else flt = x[31] ? (x[30:0] > y[30:0]) : (x[30:0] < y[30:0]);
        end
    endfunction
    function feq;
        input [31:0] x, y;
        begin
            feq = (x == y) || (is_zero(x) && is_zero(y));
        end
    endfunction

    // vfrec7 / vfrsqrt7 (tables and edge cases as in Spike's fall_reciprocal.c)
    function [6:0] rec7_tab;
        input [6:0] i;
        begin
            case (i)
                7'd0: rec7_tab = 7'd127;
                7'd1: rec7_tab = 7'd125;
                7'd2: rec7_tab = 7'd123;
                7'd3: rec7_tab = 7'd121;
                7'd4: rec7_tab = 7'd119;
                7'd5: rec7_tab = 7'd117;
                7'd6: rec7_tab = 7'd116;
                7'd7: rec7_tab = 7'd114;
                7'd8: rec7_tab = 7'd112;
                7'd9: rec7_tab = 7'd110;
                7'd10: rec7_tab = 7'd109;
                7'd11: rec7_tab = 7'd107;
                7'd12: rec7_tab = 7'd105;
                7'd13: rec7_tab = 7'd104;
                7'd14: rec7_tab = 7'd102;
                7'd15: rec7_tab = 7'd100;
                7'd16: rec7_tab = 7'd99;
                7'd17: rec7_tab = 7'd97;
                7'd18: rec7_tab = 7'd96;
                7'd19: rec7_tab = 7'd94;
                7'd20: rec7_tab = 7'd93;
                7'd21: rec7_tab = 7'd91;
                7'd22: rec7_tab = 7'd90;
                7'd23: rec7_tab = 7'd88;
                7'd24: rec7_tab = 7'd87;
                7'd25: rec7_tab = 7'd85;
                7'd26: rec7_tab = 7'd84;
                7'd27: rec7_tab = 7'd83;
                7'd28: rec7_tab = 7'd81;
                7'd29: rec7_tab = 7'd80;
                7'd30: rec7_tab = 7'd79;
                7'd31: rec7_tab = 7'd77;
                7'd32: rec7_tab = 7'd76;
                7'd33: rec7_tab = 7'd75;
                7'd34: rec7_tab = 7'd74;
                7'd35: rec7_tab = 7'd72;
                7'd36: rec7_tab = 7'd71;
                7'd37: rec7_tab = 7'd70;
                7'd38: rec7_tab = 7'd69;
                7'd39: rec7_tab = 7'd68;
                7'd40: rec7_tab = 7'd66;
                7'd41: rec7_tab = 7'd65;
                7'd42: rec7_tab = 7'd64;
                7'd43: rec7_tab = 7'd63;
                7'd44: rec7_tab = 7'd62;
                7'd45: rec7_tab = 7'd61;
                7'd46: rec7_tab = 7'd60;
                7'd47: rec7_tab = 7'd59;
                7'd48: rec7_tab = 7'd58;
                7'd49: rec7_tab = 7'd57;
                7'd50: rec7_tab = 7'd56;
                7'd51: rec7_tab = 7'd55;
                7'd52: rec7_tab = 7'd54;
                7'd53: rec7_tab = 7'd53;
                7'd54: rec7_tab = 7'd52;
                7'd55: rec7_tab = 7'd51;
                7'd56: rec7_tab = 7'd50;
                7'd57: rec7_tab = 7'd49;
                7'd58: rec7_tab = 7'd48;
                7'd59: rec7_tab = 7'd47;
                7'd60: rec7_tab = 7'd46;
                7'd61: rec7_tab = 7'd45;
                7'd62: rec7_tab = 7'd44;
                7'd63: rec7_tab = 7'd43;
                7'd64: rec7_tab = 7'd42;
                7'd65: rec7_tab = 7'd41;
                7'd66: rec7_tab = 7'd40;
                7'd67: rec7_tab = 7'd40;
                7'd68: rec7_tab = 7'd39;
                7'd69: rec7_tab = 7'd38;
                7'd70: rec7_tab = 7'd37;
                7'd71: rec7_tab = 7'd36;
                7'd72: rec7_tab = 7'd35;
                7'd73: rec7_tab = 7'd35;
                7'd74: rec7_tab = 7'd34;
                7'd75: rec7_tab = 7'd33;
                7'd76: rec7_tab = 7'd32;
                7'd77: rec7_tab = 7'd31;
                7'd78: rec7_tab = 7'd31;
                7'd79: rec7_tab = 7'd30;
                7'd80: rec7_tab = 7'd29;
                7'd81: rec7_tab = 7'd28;
                7'd82: rec7_tab = 7'd28;
                7'd83: rec7_tab = 7'd27;
                7'd84: rec7_tab = 7'd26;
                7'd85: rec7_tab = 7'd25;
                7'd86: rec7_tab = 7'd25;
                7'd87: rec7_tab = 7'd24;
                7'd88: rec7_tab = 7'd23;
                7'd89: rec7_tab = 7'd23;
                7'd90: rec7_tab = 7'd22;
                7'd91: rec7_tab = 7'd21;
                7'd92: rec7_tab = 7'd21;
                7'd93: rec7_tab = 7'd20;
                7'd94: rec7_tab = 7'd19;
                7'd95: rec7_tab = 7'd19;
                7'd96: rec7_tab = 7'd18;
                7'd97: rec7_tab = 7'd17;
                7'd98: rec7_tab = 7'd17;
                7'd99: rec7_tab = 7'd16;
                7'd100: rec7_tab = 7'd15;
                7'd101: rec7_tab = 7'd15;
                7'd102: rec7_tab = 7'd14;
                7'd103: rec7_tab = 7'd14;
                7'd104: rec7_tab = 7'd13;
                7'd105: rec7_tab = 7'd12;
                7'd106: rec7_tab = 7'd12;
                7'd107: rec7_tab = 7'd11;
                7'd108: rec7_tab = 7'd11;
                7'd109: rec7_tab = 7'd10;
                7'd110: rec7_tab = 7'd9;
                7'd111: rec7_tab = 7'd9;
                7'd112: rec7_tab = 7'd8;
                7'd113: rec7_tab = 7'd8;
                7'd114: rec7_tab = 7'd7;
                7'd115: rec7_tab = 7'd7;
                7'd116: rec7_tab = 7'd6;
                7'd117: rec7_tab = 7'd5;
                7'd118: rec7_tab = 7'd5;
                7'd119: rec7_tab = 7'd4;
                7'd120: rec7_tab = 7'd4;
                7'd121: rec7_tab = 7'd3;
                7'd122: rec7_tab = 7'd3;
                7'd123: rec7_tab = 7'd2;
                7'd124: rec7_tab = 7'd2;
                7'd125: rec7_tab = 7'd1;
                7'd126: rec7_tab = 7'd1;
                7'd127: rec7_tab = 7'd0;
                default: rec7_tab = 7'd0;
            endcase
        end
    endfunction
    function [6:0] rsqrt7_tab;
        input [6:0] i;
        begin
            case (i)
                7'd0: rsqrt7_tab = 7'd52;
                7'd1: rsqrt7_tab = 7'd51;
                7'd2: rsqrt7_tab = 7'd50;
                7'd3: rsqrt7_tab = 7'd48;
                7'd4: rsqrt7_tab = 7'd47;
                7'd5: rsqrt7_tab = 7'd46;
                7'd6: rsqrt7_tab = 7'd44;
                7'd7: rsqrt7_tab = 7'd43;
                7'd8: rsqrt7_tab = 7'd42;
                7'd9: rsqrt7_tab = 7'd41;
                7'd10: rsqrt7_tab = 7'd40;
                7'd11: rsqrt7_tab = 7'd39;
                7'd12: rsqrt7_tab = 7'd38;
                7'd13: rsqrt7_tab = 7'd36;
                7'd14: rsqrt7_tab = 7'd35;
                7'd15: rsqrt7_tab = 7'd34;
                7'd16: rsqrt7_tab = 7'd33;
                7'd17: rsqrt7_tab = 7'd32;
                7'd18: rsqrt7_tab = 7'd31;
                7'd19: rsqrt7_tab = 7'd30;
                7'd20: rsqrt7_tab = 7'd30;
                7'd21: rsqrt7_tab = 7'd29;
                7'd22: rsqrt7_tab = 7'd28;
                7'd23: rsqrt7_tab = 7'd27;
                7'd24: rsqrt7_tab = 7'd26;
                7'd25: rsqrt7_tab = 7'd25;
                7'd26: rsqrt7_tab = 7'd24;
                7'd27: rsqrt7_tab = 7'd23;
                7'd28: rsqrt7_tab = 7'd23;
                7'd29: rsqrt7_tab = 7'd22;
                7'd30: rsqrt7_tab = 7'd21;
                7'd31: rsqrt7_tab = 7'd20;
                7'd32: rsqrt7_tab = 7'd19;
                7'd33: rsqrt7_tab = 7'd19;
                7'd34: rsqrt7_tab = 7'd18;
                7'd35: rsqrt7_tab = 7'd17;
                7'd36: rsqrt7_tab = 7'd16;
                7'd37: rsqrt7_tab = 7'd16;
                7'd38: rsqrt7_tab = 7'd15;
                7'd39: rsqrt7_tab = 7'd14;
                7'd40: rsqrt7_tab = 7'd14;
                7'd41: rsqrt7_tab = 7'd13;
                7'd42: rsqrt7_tab = 7'd12;
                7'd43: rsqrt7_tab = 7'd12;
                7'd44: rsqrt7_tab = 7'd11;
                7'd45: rsqrt7_tab = 7'd10;
                7'd46: rsqrt7_tab = 7'd10;
                7'd47: rsqrt7_tab = 7'd9;
                7'd48: rsqrt7_tab = 7'd9;
                7'd49: rsqrt7_tab = 7'd8;
                7'd50: rsqrt7_tab = 7'd7;
                7'd51: rsqrt7_tab = 7'd7;
                7'd52: rsqrt7_tab = 7'd6;
                7'd53: rsqrt7_tab = 7'd6;
                7'd54: rsqrt7_tab = 7'd5;
                7'd55: rsqrt7_tab = 7'd4;
                7'd56: rsqrt7_tab = 7'd4;
                7'd57: rsqrt7_tab = 7'd3;
                7'd58: rsqrt7_tab = 7'd3;
                7'd59: rsqrt7_tab = 7'd2;
                7'd60: rsqrt7_tab = 7'd2;
                7'd61: rsqrt7_tab = 7'd1;
                7'd62: rsqrt7_tab = 7'd1;
                7'd63: rsqrt7_tab = 7'd0;
                7'd64: rsqrt7_tab = 7'd127;
                7'd65: rsqrt7_tab = 7'd125;
                7'd66: rsqrt7_tab = 7'd123;
                7'd67: rsqrt7_tab = 7'd121;
                7'd68: rsqrt7_tab = 7'd119;
                7'd69: rsqrt7_tab = 7'd118;
                7'd70: rsqrt7_tab = 7'd116;
                7'd71: rsqrt7_tab = 7'd114;
                7'd72: rsqrt7_tab = 7'd113;
                7'd73: rsqrt7_tab = 7'd111;
                7'd74: rsqrt7_tab = 7'd109;
                7'd75: rsqrt7_tab = 7'd108;
                7'd76: rsqrt7_tab = 7'd106;
                7'd77: rsqrt7_tab = 7'd105;
                7'd78: rsqrt7_tab = 7'd103;
                7'd79: rsqrt7_tab = 7'd102;
                7'd80: rsqrt7_tab = 7'd100;
                7'd81: rsqrt7_tab = 7'd99;
                7'd82: rsqrt7_tab = 7'd97;
                7'd83: rsqrt7_tab = 7'd96;
                7'd84: rsqrt7_tab = 7'd95;
                7'd85: rsqrt7_tab = 7'd93;
                7'd86: rsqrt7_tab = 7'd92;
                7'd87: rsqrt7_tab = 7'd91;
                7'd88: rsqrt7_tab = 7'd90;
                7'd89: rsqrt7_tab = 7'd88;
                7'd90: rsqrt7_tab = 7'd87;
                7'd91: rsqrt7_tab = 7'd86;
                7'd92: rsqrt7_tab = 7'd85;
                7'd93: rsqrt7_tab = 7'd84;
                7'd94: rsqrt7_tab = 7'd83;
                7'd95: rsqrt7_tab = 7'd82;
                7'd96: rsqrt7_tab = 7'd80;
                7'd97: rsqrt7_tab = 7'd79;
                7'd98: rsqrt7_tab = 7'd78;
                7'd99: rsqrt7_tab = 7'd77;
                7'd100: rsqrt7_tab = 7'd76;
                7'd101: rsqrt7_tab = 7'd75;
                7'd102: rsqrt7_tab = 7'd74;
                7'd103: rsqrt7_tab = 7'd73;
                7'd104: rsqrt7_tab = 7'd72;
                7'd105: rsqrt7_tab = 7'd71;
                7'd106: rsqrt7_tab = 7'd70;
                7'd107: rsqrt7_tab = 7'd70;
                7'd108: rsqrt7_tab = 7'd69;
                7'd109: rsqrt7_tab = 7'd68;
                7'd110: rsqrt7_tab = 7'd67;
                7'd111: rsqrt7_tab = 7'd66;
                7'd112: rsqrt7_tab = 7'd65;
                7'd113: rsqrt7_tab = 7'd64;
                7'd114: rsqrt7_tab = 7'd63;
                7'd115: rsqrt7_tab = 7'd63;
                7'd116: rsqrt7_tab = 7'd62;
                7'd117: rsqrt7_tab = 7'd61;
                7'd118: rsqrt7_tab = 7'd60;
                7'd119: rsqrt7_tab = 7'd59;
                7'd120: rsqrt7_tab = 7'd59;
                7'd121: rsqrt7_tab = 7'd58;
                7'd122: rsqrt7_tab = 7'd57;
                7'd123: rsqrt7_tab = 7'd56;
                7'd124: rsqrt7_tab = 7'd56;
                7'd125: rsqrt7_tab = 7'd55;
                7'd126: rsqrt7_tab = 7'd54;
                7'd127: rsqrt7_tab = 7'd53;
                default: rsqrt7_tab = 7'd0;
            endcase
        end
    endfunction

    function [36:0] frec7;
        input [31:0] x;
        input [2:0] mode;
        reg s;
        reg signed [15:0] e, oe;
        reg [22:0] sig;
        reg [22:0] osig;
        integer k;
        begin
            s = x[31];
            if (is_inf(x)) frec7 = {5'b0, s, 31'b0};
            else if (is_zero(x)) frec7 = {1'b0, 1'b1, 3'b0, s, 8'hFF, 23'b0};
            else if (is_nan(x)) frec7 = {is_snan(x), 4'b0, QNAN};
            else begin
                e = $signed({8'b0, x[30:23]});
                sig = x[22:0];
                if (x[30:23] == 0) begin
                    for (k = 0; k < 23; k = k + 1)
                        if (!sig[22]) begin e = e - 1; sig = sig << 1; end
                    sig = sig << 1;
                end
                if (x[30:23] == 0 && e != 0 && e != -1) begin
                    // Too small: the reciprocal overflows
                    if (mode == 3'd1 || (mode == 3'd2 && !s) || (mode == 3'd3 && s))
                        frec7 = {2'b0, 1'b1, 1'b0, 1'b1, s, 8'hFE, 23'h7FFFFF};
                    else
                        frec7 = {2'b0, 1'b1, 1'b0, 1'b1, s, 8'hFF, 23'h0};
                end else begin
                    osig = {rec7_tab(sig[22:16]), 16'b0};
                    oe = 16'sd253 - e;
                    if (oe == 0 || oe == -1) begin
                        osig = (osig >> 1) | 23'h400000;
                        if (oe == -1) begin osig = osig >> 1; oe = 0; end
                    end
                    frec7 = {5'b0, s, oe[7:0], osig};
                end
            end
        end
    endfunction

    function [36:0] frsqrt7;
        input [31:0] x;
        reg signed [15:0] e, oe;
        reg [22:0] sig;
        integer k;
        begin
            if (is_nan(x)) frsqrt7 = {is_snan(x), 4'b0, QNAN};
            else if (is_zero(x)) frsqrt7 = {1'b0, 1'b1, 3'b0, x[31], 8'hFF, 23'b0};
            else if (x[31]) frsqrt7 = {1'b1, 4'b0, QNAN};
            else if (is_inf(x)) frsqrt7 = {5'b0, 32'b0};
            else begin
                e = $signed({8'b0, x[30:23]});
                sig = x[22:0];
                if (x[30:23] == 0) begin
                    for (k = 0; k < 23; k = k + 1)
                        if (!sig[22]) begin e = e - 1; sig = sig << 1; end
                    sig = sig << 1;
                end
                oe = (16'sd380 - e) >>> 1;
                frsqrt7 = {5'b0, 1'b0, oe[7:0], rsqrt7_tab({e[0], sig[22:17]}), 16'b0};
            end
        end
    endfunction

    // ------------------------------------------------------------------
    // Operation select
    // ------------------------------------------------------------------
    // All add/sub/mul/multiply-add forms go through one compact fused multiply-add.
    reg  [31:0] fx, fy, fz;
    reg         use_fma;
    wire [31:0] fma_res;
    wire [4:0]  fma_flags;
    fp32_fma fma_u (.a(fx), .b(fy), .c(fz), .rm(rm), .mul(funct6 == 6'b100100),
                     .result(fma_res), .flags(fma_flags));
    always @(*) begin
        use_fma = 1'b1;
        fx = a; fy = ONE; fz = b;
        case (funct6)
            6'b000000, 6'b000001, 6'b000011: begin fx = a; fy = ONE; fz = b; end           // fadd/fredsum
            6'b000010: begin fx = a; fy = ONE; fz = {~b[31], b[30:0]}; end                 // vfsub
            6'b100111: begin fx = b; fy = ONE; fz = {~a[31], a[30:0]}; end                 // vfrsub
            6'b100100: begin fx = a; fy = b; fz = NEG_ZERO; end                            // vfmul
            6'b101000: begin fx = b; fy = d; fz = a; end                                   // vfmadd
            6'b101001: begin fx = {~b[31], b[30:0]}; fy = d; fz = {~a[31], a[30:0]}; end   // vfnmadd
            6'b101010: begin fx = b; fy = d; fz = {~a[31], a[30:0]}; end                   // vfmsub
            6'b101011: begin fx = {~b[31], b[30:0]}; fy = d; fz = a; end                   // vfnmsub
            6'b101100: begin fx = b; fy = a; fz = d; end                                   // vfmacc
            6'b101101: begin fx = {~b[31], b[30:0]}; fy = a; fz = {~d[31], d[30:0]}; end   // vfnmacc
            6'b101110: begin fx = b; fy = a; fz = {~d[31], d[30:0]}; end                   // vfmsac
            6'b101111: begin fx = {~b[31], b[30:0]}; fy = a; fz = d; end                   // vfnmsac
            default: use_fma = 1'b0;
        endcase
    end

    reg [36:0] r;
    reg nv;
    always @(*) begin
        r = 37'b0;
        result_bit = 1'b0;
        nv = 1'b0;
        case (funct6)
            6'b000100, 6'b000101: r = fminmax(a, b, 1'b0);
            6'b000110, 6'b000111: r = fminmax(a, b, 1'b1);
            6'b001000: r = {5'b0, b[31], a[30:0]};                                      // vfsgnj
            6'b001001: r = {5'b0, ~b[31], a[30:0]};                                     // vfsgnjn
            6'b001010: r = {5'b0, a[31] ^ b[31], a[30:0]};                              // vfsgnjx
            6'b010010: begin                                                            // VFUNARY0
                case (vs1_field)
                    5'd0:  r = f2i(a, 1'b0, 1'b0, rm);          // vfcvt.xu.f.v
                    5'd1:  r = f2i(a, 1'b1, 1'b0, rm);          // vfcvt.x.f.v
                    5'd2:  r = i2f(a, 1'b0, rm);                // vfcvt.f.xu.v
                    5'd3:  r = i2f(a, 1'b1, rm);                // vfcvt.f.x.v
                    5'd6:  r = f2i(a, 1'b0, 1'b0, 3'd1);        // vfcvt.rtz.xu.f.v
                    5'd7:  r = f2i(a, 1'b1, 1'b0, 3'd1);        // vfcvt.rtz.x.f.v
                    5'd10: r = i2f({16'b0, a[15:0]}, 1'b0, rm); // vfwcvt.f.xu.v (int16)
                    5'd11: r = i2f({{16{a[15]}}, a[15:0]}, 1'b1, rm);
                    5'd16: r = f2i(a, 1'b0, 1'b1, rm);          // vfncvt.xu.f.w
                    5'd17: r = f2i(a, 1'b1, 1'b1, rm);          // vfncvt.x.f.w
                    5'd22: r = f2i(a, 1'b0, 1'b1, 3'd1);        // vfncvt.rtz.xu.f.w
                    5'd23: r = f2i(a, 1'b1, 1'b1, 3'd1);
                    default: r = 37'b0;
                endcase
            end
            6'b010011: begin                                                            // VFUNARY1
                case (vs1_field)
                    5'd4:  r = frsqrt7(a);
                    5'd5:  r = frec7(a, rm);
                    5'd16: r = {5'b0, 22'b0, fclass(a)};
                    default: r = 37'b0;   // vfsqrt: fp32_divsqrt
                endcase
            end
            // compares (vs2 op vs1/f): eq/ne quiet, lt/le/gt/ge signaling
            6'b011000: begin result_bit = !is_nan(a) && !is_nan(b) && feq(a, b); nv = is_snan(a) || is_snan(b); end
            6'b011100: begin result_bit = is_nan(a) || is_nan(b) || !feq(a, b);  nv = is_snan(a) || is_snan(b); end
            6'b011011: begin result_bit = !is_nan(a) && !is_nan(b) && flt(a, b);  nv = is_nan(a) || is_nan(b); end
            6'b011001: begin result_bit = !is_nan(a) && !is_nan(b) && (flt(a, b) || feq(a, b)); nv = is_nan(a) || is_nan(b); end
            6'b011101: begin result_bit = !is_nan(a) && !is_nan(b) && flt(b, a);  nv = is_nan(a) || is_nan(b); end
            6'b011111: begin result_bit = !is_nan(a) && !is_nan(b) && (flt(b, a) || feq(a, b)); nv = is_nan(a) || is_nan(b); end
            default: r = 37'b0;
        endcase
        if (use_fma) r = {fma_flags, fma_res};
        result = r[31:0];
        flags = r[36:32] | {nv, 4'b0};
    end

endmodule

// ----------------------------------------------------------------------------
// Iterative FP32 divide / square root: one quotient/root bit per cycle (27 cycles).
// ----------------------------------------------------------------------------
module fp32_divsqrt (
    input  wire        clk,
    input  wire        rst,
    input  wire        start,     // pulse with operands valid
    input  wire        is_sqrt,
    input  wire [31:0] x,         // dividend / radicand
    input  wire [31:0] y,         // divisor
    input  wire [2:0]  rm,
    output reg         done,      // one-cycle pulse
    output reg  [31:0] result,
    output reg  [4:0]  flags
);
    localparam [31:0] QNAN = 32'h7FC00000;
    function is_nan;  input [31:0] v; begin is_nan = (v[30:23] == 8'hFF) && (v[22:0] != 0); end endfunction
    function is_snan; input [31:0] v; begin is_snan = is_nan(v) && !v[22]; end endfunction
    function is_inf;  input [31:0] v; begin is_inf = (v[30:23] == 8'hFF) && (v[22:0] == 0); end endfunction
    function is_zero; input [31:0] v; begin is_zero = (v[30:0] == 0); end endfunction

    // Reuse the shared round/pack through a vpu_fp32-style function copy
    function round_up;
        input [2:0] mode;
        input s, lsb, g, st;
        begin
            case (mode)
                3'd0: round_up = g && (st || lsb);
                3'd1: round_up = 1'b0;
                3'd2: round_up = (g || st) && s;
                3'd3: round_up = (g || st) && !s;
                3'd4: round_up = g;
                default: round_up = 1'b0;
            endcase
        end
    endfunction
    function [36:0] round_pack;
        input s;
        input [63:0] M;
        input signed [15:0] eL;
        input [2:0] mode;
        integer p, lsb_pos, i;
        reg [63:0] mant, mant2;
        reg g, st, g2, st2, inc, inexact, tiny;
        reg signed [15:0] Le, E, Eu;
        begin
            p = 0;
            for (i = 0; i < 64; i = i + 1) if (M[i]) p = i;
            lsb_pos = p - 23;
            if (lsb_pos < -149 - eL) lsb_pos = -149 - eL;
            if (lsb_pos <= 0) begin mant = M << (-lsb_pos); g = 0; st = 0; end
            else if (lsb_pos > p + 1) begin mant = 64'd0; g = 0; st = 1; end
            else begin
                mant = M >> lsb_pos;
                g = M[lsb_pos - 1];
                st = (lsb_pos >= 2) ? ((M & ((64'd1 << (lsb_pos - 1)) - 64'd1)) != 0) : 1'b0;
            end
            inexact = g || st;
            inc = round_up(mode, s, mant[0], g, st);
            mant = mant + {63'b0, inc};
            Le = lsb_pos + eL;
            if (mant[24]) begin mant = mant >> 1; Le = Le + 1; end
            E = mant[23] ? (Le + 16'sd150) : 16'sd0;
            if (p - 23 <= 0) begin mant2 = M << (23 - p); g2 = 0; st2 = 0; end
            else begin
                mant2 = M >> (p - 23);
                g2 = M[p - 24];
                st2 = (p - 23 >= 2) ? ((M & ((64'd1 << (p - 24)) - 64'd1)) != 0) : 1'b0;
            end
            mant2 = mant2 + {63'b0, round_up(mode, s, mant2[0], g2, st2)};
            Eu = p + eL + 16'sd127 + (mant2[24] ? 16'sd1 : 16'sd0);
            tiny = (Eu < 16'sd1);
            if (E >= 16'sd255) begin
                if (mode == 3'd1 || (mode == 3'd2 && !s) || (mode == 3'd3 && s))
                    round_pack = {5'b00101, s, 8'hFE, 23'h7FFFFF};
                else
                    round_pack = {5'b00101, s, 8'hFF, 23'h0};
            end else
                round_pack = {3'b0, tiny && inexact, inexact, s, E[7:0], mant[22:0]};
        end
    endfunction

    // Normalize a finite nonzero operand: mantissa with the leading one at bit 23
    function [39:0] norm;   // {exp(16, signed, unbiased-ish: value = m * 2^(e-150)), m(24)}
        input [31:0] v;
        reg [23:0] m;
        reg signed [15:0] e;
        integer k;
        begin
            m = {(v[30:23] != 0), v[22:0]};
            e = (v[30:23] == 0) ? 16'sd1 : $signed({8'b0, v[30:23]});
            for (k = 0; k < 24; k = k + 1)
                if (!m[23]) begin m = m << 1; e = e - 1; end
            norm = {e, m};
        end
    endfunction

    reg busy;
    reg [4:0] count;
    reg op_sqrt, s_r;
    reg [2:0] rm_r;
    reg signed [15:0] eL_r;
    reg [52:0] rem;       // partial remainder
    reg [52:0] divisor;
    reg [27:0] q;         // quotient / root bits
    reg [55:0] rad;       // radicand bits still to bring down (sqrt)
    reg special;
    reg [31:0] special_val;
    reg [4:0] special_flags;

    reg [39:0] nx, ny;
    reg signed [15:0] ex_, ey_, t;
    reg [23:0] mx_, my_;
    reg [52:0] trial;
    reg [55:0] m_rad;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy <= 1'b0;
            done <= 1'b0;
            result <= 32'b0;
            flags <= 5'b0;
            count <= 5'd0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                op_sqrt <= is_sqrt;
                rm_r <= rm;
                special <= 1'b1;
                special_flags <= 5'b0;
                if (is_sqrt) begin
                    s_r <= 1'b0;
                    if (is_nan(x)) begin special_val <= QNAN; special_flags <= {is_snan(x), 4'b0}; end
                    else if (is_zero(x)) special_val <= x;
                    else if (x[31]) begin special_val <= QNAN; special_flags <= 5'b10000; end
                    else if (is_inf(x)) special_val <= x;
                    else begin
                        special <= 1'b0;
                        nx = norm(x);
                        ex_ = nx[39:24];
                        mx_ = nx[23:0];
                        // value = m * 2^(e-150); make (e-150) even, then root of m * 2^30
                        t = ex_ - 16'sd150;
                        if (t[0]) begin m_rad = {31'b0, mx_, 1'b0}; t = t - 16'sd1; end
                        else m_rad = {32'b0, mx_};
                        // 27 bit-pairs of (m_rad << 30) give root = sqrt(m_rad << 28):
                        // value = root * 2^((t - 28)/2)
                        rad <= m_rad << 30;
                        eL_r <= ((t - 16'sd28) >>> 1) - 16'sd1;   // one extra bit for sticky
                        rem <= 53'b0;
                        q <= 28'b0;
                        count <= 5'd0;
                    end
                end else begin
                    s_r <= x[31] ^ y[31];
                    if (is_nan(x) || is_nan(y)) begin
                        special_val <= QNAN; special_flags <= {is_snan(x) || is_snan(y), 4'b0};
                    end else if ((is_inf(x) && is_inf(y)) || (is_zero(x) && is_zero(y))) begin
                        special_val <= QNAN; special_flags <= 5'b10000;
                    end else if (is_inf(x) || is_zero(y)) begin
                        special_val <= {x[31] ^ y[31], 8'hFF, 23'b0};
                        special_flags <= (is_zero(y) && !is_inf(x)) ? 5'b01000 : 5'b0;
                    end else if (is_zero(x) || is_inf(y)) begin
                        special_val <= {x[31] ^ y[31], 31'b0};
                    end else begin
                        special <= 1'b0;
                        nx = norm(x);
                        ny = norm(y);
                        ex_ = nx[39:24];
                        ey_ = ny[39:24];
                        // quotient bits: Q = floor(mx * 2^26 / my), 27 bits; value = Q * 2^(ex-ey-26)
                        rem <= {29'b0, nx[23:0]};
                        divisor <= {29'b0, ny[23:0]};
                        eL_r <= ex_ - ey_ - 16'sd26 - 16'sd1;     // extra bit for sticky
                        q <= 28'b0;
                        count <= 5'd0;
                    end
                end
                busy <= 1'b1;
            end else if (busy) begin
                if (special) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    result <= special_val;
                    flags <= special_flags;
                end else if (count != 5'd27) begin
                    count <= count + 5'd1;
                    if (op_sqrt) begin
                        // bring down two radicand bits, try (4*root + 1)
                        trial = {rem[50:0], rad[55:54]} - {23'b0, q[27:0], 2'b01};
                        if (!trial[52]) begin
                            rem <= trial;
                            q <= {q[26:0], 1'b1};
                        end else begin
                            rem <= {rem[50:0], rad[55:54]};
                            q <= {q[26:0], 1'b0};
                        end
                        rad <= rad << 2;
                    end else begin
                        if (rem >= divisor) begin
                            rem <= (rem - divisor) << 1;
                            q <= {q[26:0], 1'b1};
                        end else begin
                            rem <= rem << 1;
                            q <= {q[26:0], 1'b0};
                        end
                    end
                end else begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    {flags, result} <= round_pack(s_r, {35'b0, q[26:0], 1'b0} | {63'b0, (rem != 0)},
                                                  eL_r, rm_r);
                end
            end
        end
    end

endmodule
