`default_nettype none
// Scalar FP64 (RV32D) arithmetic for the FP/vector coprocessor, plus the conversions
// between double and single. Operand roles follow the scalar instruction: x = rs1, y = rs2,
// z = rs3. Divide/sqrt run in fp_divsqrt (iterative); everything else is combinational.
//
// op encoding (set by vpu_top from the instruction):
//   0 add  1 sub  2 mul  3 fmadd  4 fmsub  5 fnmsub  6 fnmadd
//   7 sgnj 8 sgnjn 9 sgnjx 10 min 11 max 12 eq 13 lt 14 le 15 class
//   16 cvt.w.d 17 cvt.wu.d 18 cvt.d.w 19 cvt.d.wu 20 cvt.s.d 21 cvt.d.s
module vpu_fp64 (
    input  wire [4:0]  op,
    input  wire [63:0] x,
    input  wire [63:0] y,
    input  wire [63:0] z,
    input  wire [31:0] xi,          // integer source (cvt.d.w) / single source (cvt.d.s)
    input  wire [2:0]  rm,
    output reg  [63:0] result,      // double, or {32'b0, int/single/compare/class}
    output reg  [4:0]  flags
);
    localparam [63:0] QNAN = 64'h7FF8000000000000;

    function is_nan;  input [63:0] v; begin is_nan = (v[62:52] == 11'h7FF) && (v[51:0] != 0); end endfunction
    function is_snan; input [63:0] v; begin is_snan = is_nan(v) && !v[51]; end endfunction
    function is_inf;  input [63:0] v; begin is_inf = (v[62:52] == 11'h7FF) && (v[51:0] == 0); end endfunction
    function is_zero; input [63:0] v; begin is_zero = (v[62:0] == 0); end endfunction

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

    // ---------------- FMA-based ops ----------------
    reg [63:0] fx, fy, fz;
    wire [63:0] fma_res;
    wire [4:0] fma_fl;
    localparam [63:0] ONE = 64'h3FF0000000000000;
    localparam [63:0] NZERO = 64'h8000000000000000;
    always @(*) begin
        fx = x; fy = ONE; fz = y;
        case (op)
            5'd1: begin fx = x; fy = ONE; fz = {~y[63], y[62:0]}; end           // sub
            5'd2: begin fx = x; fy = y; fz = NZERO; end                          // mul
            5'd3: begin fx = x; fy = y; fz = z; end                              // fmadd
            5'd4: begin fx = x; fy = y; fz = {~z[63], z[62:0]}; end              // fmsub
            5'd5: begin fx = {~x[63], x[62:0]}; fy = y; fz = z; end              // fnmsub
            5'd6: begin fx = {~x[63], x[62:0]}; fy = y; fz = {~z[63], z[62:0]}; end // fnmadd
            default: ;
        endcase
    end
    fp_fma #(.EW(11), .MW(52)) fma64 (.a(fx), .b(fy), .c(fz), .rm(rm), .mul(op == 5'd2),
                                     .result(fma_res), .flags(fma_fl));

    // ---------------- Comparisons ----------------
    function flt;
        input [63:0] p, q;
        begin
            if (is_zero(p) && is_zero(q)) flt = 1'b0;
            else if (p[63] != q[63]) flt = p[63];
            else flt = p[63] ? (p[62:0] > q[62:0]) : (p[62:0] < q[62:0]);
        end
    endfunction
    function feq;
        input [63:0] p, q;
        begin
            feq = (p == q) || (is_zero(p) && is_zero(q));
        end
    endfunction

    // ---------------- double -> int32 ----------------
    function [36:0] d2i;
        input [63:0] v;
        input is_signed;
        input [2:0] mode;
        reg [10:0] e;
        reg [52:0] m;
        reg [33:0] I;
        reg g, st, s, inv;
        integer r;
        reg [33:0] maxpos, maxneg;
        reg [31:0] o;
        begin
            s = v[63];
            e = v[62:52];
            m = {(e != 0), v[51:0]};
            maxpos = is_signed ? 34'h7FFFFFFF : 34'hFFFFFFFF;
            maxneg = is_signed ? 34'h80000000 : 34'h0;
            r = 1075 - ((e == 0) ? 1 : e);
            if (e >= 11'd1055) begin I = 34'h100000000; g = 0; st = 0; end   // |v| >= 2^32
            else if (r >= 54) begin I = 0; g = 0; st = (m != 0); end
            else begin
                I = {1'b0, m >> r};
                g = m[r - 1];
                st = (r >= 2) ? ((m & ((53'd1 << (r - 1)) - 53'd1)) != 0) : 1'b0;
            end
            I = I + round_up(mode, s, I[0], g, st);
            inv = (!s && I > maxpos) || (s && is_signed && I > maxneg) || (s && !is_signed && I != 0);
            if (is_nan(v)) d2i = {1'b1, 4'b0, maxpos[31:0]};
            else if (inv || is_inf(v)) d2i = {1'b1, 4'b0, s ? maxneg[31:0] : maxpos[31:0]};
            else begin
                o = s ? (32'd0 - I[31:0]) : I[31:0];
                d2i = {4'b0, g || st, o};
            end
        end
    endfunction

    // ---------------- int32 -> double (exact) ----------------
    function [63:0] i2d;
        input [31:0] v;
        input is_signed;
        reg s;
        reg [31:0] mag;
        reg [4:0] lz;
        reg [31:0] nrm;
        integer k;
        begin
            s = is_signed && v[31];
            mag = s ? (32'd0 - v) : v;
            lz = 0;
            for (k = 0; k < 32; k = k + 1) if (mag[k]) lz = 31 - k;
            nrm = mag << lz;
            if (mag == 0) i2d = 64'b0;
            else i2d = {s, 11'd1054 - {6'b0, lz}, nrm[30:0], 21'b0};
        end
    endfunction

    // ---------------- double -> single (rounded) ----------------
    function [36:0] d2s;
        input [63:0] v;
        input [2:0] mode;
        reg s;
        reg [52:0] m;
        reg signed [12:0] es;       // biased single exponent of the leading bit
        reg [80:0] W;
        reg [23:0] mant;
        reg [24:0] mr;
        reg g, st, tiny, g2, st2;
        reg [24:0] mr2;
        integer sh;
        begin
            s = v[63];
            m = {(v[62:52] != 0), v[51:0]};
            if (is_nan(v)) d2s = {is_snan(v), 4'b0, 32'h7FC00000};
            else if (is_inf(v)) d2s = {5'b0, s, 8'hFF, 23'b0};
            else if (is_zero(v)) d2s = {5'b0, s, 31'b0};
            else if (v[62:52] == 0) begin
                // double subnormal: far below the smallest single subnormal
                if ((mode == 3'd3 && !s) || (mode == 3'd2 && s)) d2s = {5'b00011, s, 31'd1};
                else d2s = {5'b00011, s, 31'd0};
            end else begin
                es = $signed({2'b0, v[62:52]}) - 13'sd1023 + 13'sd127;
                // Significand bits below the kept ones: 29 for a normal result, more if subnormal
                sh = 29 + ((es < 1) ? (1 - es) : 0);
                if (sh > 80) sh = 80;
                W = {m, 28'b0};                          // m at bits [80:28]
                mant = (W >> (sh + 28)) & 24'hFFFFFF;
                g = W[sh + 27];
                st = ((W & ((81'd1 << (sh + 27)) - 81'd1)) != 0);
                mr = {1'b0, mant} + round_up(mode, s, mant[0], g, st);
                // tininess after rounding: round at 24 bits with unbounded exponent
                mr2 = {1'b0, m[52:29]} + round_up(mode, s, m[29], m[28], m[27:0] != 0);
                tiny = (es < 0) || (es == 0 && !mr2[24]);
                if (es >= 1) begin
                    if (mr[24]) begin
                        if (es + 1 >= 255) d2s = {5'b00101, s, (mode == 3'd1 || (mode == 3'd2 && !s) || (mode == 3'd3 && s)) ? 31'h7F7FFFFF : 31'h7F800000};
                        else d2s = {4'b0, g || st, s, es[7:0] + 8'd1, mr[23:1]};
                    end else if (es >= 255) begin
                        d2s = {5'b00101, s, (mode == 3'd1 || (mode == 3'd2 && !s) || (mode == 3'd3 && s)) ? 31'h7F7FFFFF : 31'h7F800000};
                    end else d2s = {4'b0, g || st, s, es[7:0], mr[22:0]};
                end else begin
                    // subnormal single (or rounds up into the smallest normal)
                    d2s = {3'b0, tiny && (g || st), g || st, s, mr[23] ? 8'd1 : 8'd0, mr[22:0]};
                end
            end
        end
    endfunction

    // ---------------- single -> double (exact) ----------------
    reg [63:0] sd;
    always @(*) begin : s2d_full
        reg [22:0] f;
        reg signed [12:0] e;
        integer k;
        sd = 64'b0;
        if ((xi[30:23] == 8'hFF) && (xi[22:0] != 0)) sd = QNAN;
        else if (xi[30:23] == 8'hFF) sd = {xi[31], 11'h7FF, 52'b0};
        else if (xi[30:0] == 0) sd = {xi[31], 63'b0};
        else begin
            f = xi[22:0];
            e = (xi[30:23] == 0) ? 13'sd1 : $signed({5'b0, xi[30:23]});
            if (xi[30:23] == 0) begin
                for (k = 0; k < 23; k = k + 1)
                    if (!f[22]) begin f = f << 1; e = e - 1; end
                f = f << 1;   // drop the leading one
                e = e - 1;
            end
            sd = {xi[31], e[10:0] + 11'd896, f, 29'b0};
        end
    end

    // ---------------- Select ----------------
    reg nv;
    reg a_lt;
    always @(*) begin
        flags = 5'b0;
        result = 64'b0;
        nv = is_snan(x) || is_snan(y);
        a_lt = (x[63] != y[63]) ? x[63] : x[63] ? (x[62:0] > y[62:0]) : (x[62:0] < y[62:0]);
        case (op)
            5'd0, 5'd1, 5'd2, 5'd3, 5'd4, 5'd5, 5'd6: begin result = fma_res; flags = fma_fl; end
            5'd7:  result = {y[63], x[62:0]};
            5'd8:  result = {~y[63], x[62:0]};
            5'd9:  result = {x[63] ^ y[63], x[62:0]};
            5'd10, 5'd11: begin
                flags[4] = nv;
                if (is_nan(x) && is_nan(y)) result = QNAN;
                else if (is_nan(x)) result = y;
                else if (is_nan(y)) result = x;
                else result = ((op == 5'd10) ? a_lt : !a_lt) ? x : y;
            end
            5'd12: begin result = {63'b0, !is_nan(x) && !is_nan(y) && feq(x, y)}; flags[4] = nv; end
            5'd13: begin result = {63'b0, !is_nan(x) && !is_nan(y) && flt(x, y)}; flags[4] = is_nan(x) || is_nan(y); end
            5'd14: begin result = {63'b0, !is_nan(x) && !is_nan(y) && (flt(x, y) || feq(x, y))}; flags[4] = is_nan(x) || is_nan(y); end
            5'd15: begin
                result = 64'b0;
                if (is_inf(x))                 result[x[63] ? 0 : 7] = 1'b1;
                else if (is_nan(x))            result[x[51] ? 9 : 8] = 1'b1;
                else if (is_zero(x))           result[x[63] ? 3 : 4] = 1'b1;
                else if (x[62:52] == 11'd0)    result[x[63] ? 2 : 5] = 1'b1;
                else                           result[x[63] ? 1 : 6] = 1'b1;
            end
            5'd16: begin {flags, result[31:0]} = d2i(x, 1'b1, rm); end
            5'd17: begin {flags, result[31:0]} = d2i(x, 1'b0, rm); end
            5'd18: result = i2d(xi, 1'b1);
            5'd19: result = i2d(xi, 1'b0);
            5'd20: begin {flags, result[31:0]} = d2s(x, rm); end
            5'd21: begin result = sd; flags[4] = (xi[30:23] == 8'hFF) && (xi[22:0] != 0) && !xi[22]; end
            default: ;
        endcase
    end

endmodule

// ----------------------------------------------------------------------------
// Iterative divide / square root for any binary format (P+3 cycles).
// ----------------------------------------------------------------------------
module fp_divsqrt #(
    parameter EW = 11,
    parameter MW = 52
) (
    input  wire          clk,
    input  wire          rst,
    input  wire          start,
    input  wire          is_sqrt,
    input  wire [EW+MW:0] x,
    input  wire [EW+MW:0] y,
    input  wire [2:0]    rm,
    output reg           done,
    output reg  [EW+MW:0] result,
    output reg  [4:0]    flags
);
    localparam P = MW + 1;
    localparam N = EW + MW + 1;
    localparam IT = P + 3;                 // quotient / root bits
    localparam RW = 2 * IT + 2;            // radicand window
    localparam SE = (P + 4) - ((P + 4) % 2);   // even radicand shift (the root exponent halves it)
    localparam integer BIAS = (1 << (EW - 1)) - 1;
    localparam integer EMAX = (1 << EW) - 1;
    localparam [N-1:0] QNAN = {1'b0, {EW{1'b1}}, 1'b1, {(MW-1){1'b0}}};
    localparam [IT:0] ONE_W = 1;

    function is_nan;  input [N-1:0] v; begin is_nan = (&v[N-2:MW]) && (v[MW-1:0] != 0); end endfunction
    function is_snan; input [N-1:0] v; begin is_snan = is_nan(v) && !v[MW-1]; end endfunction
    function is_inf;  input [N-1:0] v; begin is_inf = (&v[N-2:MW]) && (v[MW-1:0] == 0); end endfunction
    function is_zero; input [N-1:0] v; begin is_zero = (v[N-2:0] == 0); end endfunction
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

    // Round and pack s * M * 2^eL, M has its leading one at or below bit IT (one extra
    // sticky bit below the quotient), never zero.
    function [N+4:0] round_pack;
        input s;
        input [IT:0] M;
        input signed [15:0] eL;
        input [2:0] mode;
        integer p, lsb_pos, i;
        reg [IT:0] mant, mant2;
        reg g, st, g2, st2, inexact, tiny;
        reg signed [15:0] Le, E, Eu;
        begin
            p = 0;
            for (i = 0; i <= IT; i = i + 1) if (M[i]) p = i;
            lsb_pos = p - MW;
            if (lsb_pos < (1 - BIAS - MW) - eL) lsb_pos = (1 - BIAS - MW) - eL;
            if (lsb_pos <= 0) begin mant = M << (-lsb_pos); g = 0; st = 0; end
            else if (lsb_pos > p + 1) begin mant = 0; g = 0; st = 1; end
            else begin
                mant = M >> lsb_pos;
                g = M[lsb_pos - 1];
                st = (lsb_pos >= 2) ? ((M & ((ONE_W << (lsb_pos - 1)) - ONE_W)) != 0) : 1'b0;
            end
            inexact = g || st;
            mant = mant + round_up(mode, s, mant[0], g, st);
            Le = lsb_pos + eL;
            if (mant[P]) begin mant = mant >> 1; Le = Le + 1; end
            E = mant[P-1] ? (Le + BIAS + MW) : 0;
            if (p - MW <= 0) begin mant2 = M << (MW - p); g2 = 0; st2 = 0; end
            else begin
                mant2 = M >> (p - MW);
                g2 = M[p - MW - 1];
                st2 = (p - MW >= 2) ? ((M & ((ONE_W << (p - MW - 1)) - ONE_W)) != 0) : 1'b0;
            end
            mant2 = mant2 + round_up(mode, s, mant2[0], g2, st2);
            Eu = p + eL + BIAS + (mant2[P] ? 1 : 0);
            tiny = (Eu < 1);
            if (E >= EMAX) begin
                if (mode == 3'd1 || (mode == 3'd2 && !s) || (mode == 3'd3 && s))
                    round_pack = {5'b00101, s, {(EW-1){1'b1}}, 1'b0, {MW{1'b1}}};
                else
                    round_pack = {5'b00101, s, {EW{1'b1}}, {MW{1'b0}}};
            end else
                round_pack = {3'b0, tiny && inexact, inexact, s, E[EW-1:0], mant[MW-1:0]};
        end
    endfunction

    // Normalize a finite nonzero operand: significand with the leading one at bit P-1,
    // value = m * 2^(e - BIAS - MW)
    function [P+15:0] norm;
        input [N-1:0] v;
        reg [P-1:0] m;
        reg signed [15:0] e;
        integer k;
        begin
            m = {(v[N-2:MW] != 0), v[MW-1:0]};
            e = (v[N-2:MW] == 0) ? 16'sd1 : $signed({{(16-EW){1'b0}}, v[N-2:MW]});
            for (k = 0; k < P; k = k + 1)
                if (!m[P-1]) begin m = m << 1; e = e - 1; end
            norm = {e, m};
        end
    endfunction

    reg busy, op_sqrt, s_r, special;
    reg [7:0] count;
    reg [2:0] rm_r;
    reg signed [15:0] eL_r;
    reg [2*P+4:0] rem, divisor;
    reg [IT:0] q;
    reg [RW-1:0] rad;
    reg [N-1:0] special_val;
    reg [4:0] special_flags;
    reg [P+15:0] nx, ny;
    reg signed [15:0] t;
    reg [P:0] m_rad;
    reg [2*P+4:0] trial;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            busy <= 1'b0;
            done <= 1'b0;
            count <= 0;
        end else begin
            done <= 1'b0;
            if (start && !busy) begin
                op_sqrt <= is_sqrt;
                rm_r <= rm;
                special <= 1'b1;
                special_flags <= 5'b0;
                busy <= 1'b1;
                if (is_sqrt) begin
                    s_r <= 1'b0;
                    if (is_nan(x)) begin special_val <= QNAN; special_flags <= {is_snan(x), 4'b0}; end
                    else if (is_zero(x)) special_val <= x;
                    else if (x[N-1]) begin special_val <= QNAN; special_flags <= 5'b10000; end
                    else if (is_inf(x)) special_val <= x;
                    else begin
                        special <= 1'b0;
                        nx = norm(x);
                        t = nx[P+15:P] - (BIAS + MW);
                        if (t[0]) begin m_rad = {nx[P-1:0], 1'b0}; t = t - 1; end
                        else m_rad = {1'b0, nx[P-1:0]};
                        // root = floor(sqrt(m_rad << SE)), SE the even shift that fits (P+4 or P+3)
                        rad <= {{(RW-P-1){1'b0}}, m_rad} << (SE + 2);
                        eL_r <= ((t - SE) >>> 1) - 1;
                        rem <= 0;
                        q <= 0;
                        count <= 0;
                    end
                end else begin
                    s_r <= x[N-1] ^ y[N-1];
                    if (is_nan(x) || is_nan(y)) begin
                        special_val <= QNAN; special_flags <= {is_snan(x) || is_snan(y), 4'b0};
                    end else if ((is_inf(x) && is_inf(y)) || (is_zero(x) && is_zero(y))) begin
                        special_val <= QNAN; special_flags <= 5'b10000;
                    end else if (is_inf(x) || is_zero(y)) begin
                        special_val <= {x[N-1] ^ y[N-1], {EW{1'b1}}, {MW{1'b0}}};
                        special_flags <= (is_zero(y) && !is_inf(x)) ? 5'b01000 : 5'b0;
                    end else if (is_zero(x) || is_inf(y)) begin
                        special_val <= {x[N-1] ^ y[N-1], {(N-1){1'b0}}};
                    end else begin
                        special <= 1'b0;
                        nx = norm(x);
                        ny = norm(y);
                        rem <= nx[P-1:0];
                        divisor <= ny[P-1:0];
                        eL_r <= nx[P+15:P] - ny[P+15:P] - (P + 2) - 1;
                        q <= 0;
                        count <= 0;
                    end
                end
            end else if (busy) begin
                if (special) begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    result <= special_val;
                    flags <= special_flags;
                end else if (count != IT) begin
                    count <= count + 1;
                    if (op_sqrt) begin
                        trial = {rem[2*P+2:0], rad[RW-1:RW-2]} - {q, 2'b01};
                        if (!trial[2*P+4]) begin
                            rem <= trial;
                            q <= {q[IT-1:0], 1'b1};
                        end else begin
                            rem <= {rem[2*P+2:0], rad[RW-1:RW-2]};
                            q <= {q[IT-1:0], 1'b0};
                        end
                        rad <= rad << 2;
                    end else begin
                        if (rem >= divisor) begin
                            rem <= (rem - divisor) << 1;
                            q <= {q[IT-1:0], 1'b1};
                        end else begin
                            rem <= rem << 1;
                            q <= {q[IT-1:0], 1'b0};
                        end
                    end
                end else begin
                    busy <= 1'b0;
                    done <= 1'b1;
                    {flags, result} <= round_pack(s_r, {q[IT-1:0], (rem != 0)}, eL_r, rm_r);
                end
            end
        end
    end

endmodule
