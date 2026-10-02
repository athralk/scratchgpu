`default_nettype none
// ----------------------------------------------------------------------------
// Fast approximate divide / square root (the CUDA __fdividef approach): a/b = a * (1/b).
// Same ports and handshake as fp_divsqrt, so either can sit behind vpu_top.
//
//   1/b      : seed from a 2048-entry table (2^-12), then Goldschmidt steps
//              q = a*r, d = b*r; repeat { f = 2 - d; q = q*f; d = d*f }
//   sqrt(u)  : 1/sqrt seed from a 2048-entry table, then Goldschmidt steps
//              g = u*y, h = y/2; repeat { r = 1/2 - g*h; g = g + g*r; h = h + h*r }
// Each step squares the relative error: 2^-12 -> 2^-24 -> 2^-48 -> 2^-96.
//   FP32 with 1 step: within ~2 ulp (what __fdividef / -use_fast_math promise).
//   FP64 with 3 steps: within ~1 ulp.
// Results are NOT always correctly rounded, and the inexact flag is set whenever the
// approximation leaves any bits below the result (also for exact quotients like 6/3).
// Special operands (NaN, infinity, zero, x/0, sqrt of a negative) and their flags are exact,
// and so is division by a power of two (x/1, x/2, x/0.25 ...), which skips the approximation.
//
// Every stage is registered, so no path is longer than one shift, one table read or one
// pipelined multiply: start -> classify -> normalize -> table -> multiply rounds ->
// pack -> (shared with fp_divsqrt) normalize/denormalize/round.
// ----------------------------------------------------------------------------
module fp_rcpdiv #(
    parameter EW = 11,
    parameter MW = 52,
    parameter DIV_STEPS = 3,
    parameter SQRT_STEPS = 3
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
    localparam IT = P + 3;                 // result bits handed to the rounding stages
    localparam F = IT + 4;                 // fraction bits of the fixed-point datapath
    localparam W = F + 3;                  // signed, values below 4
    localparam integer BIAS = (1 << (EW - 1)) - 1;
    localparam integer EMAX = (1 << EW) - 1;
    localparam [N-1:0] QNAN = {1'b0, {EW{1'b1}}, 1'b1, {(MW-1){1'b0}}};
    localparam signed [W-1:0] TWO  = {{(W-F-2){1'b0}}, 2'b10, {F{1'b0}}};
    localparam signed [W-1:0] HALF = {{(W-F){1'b0}}, 1'b1, {(F-1){1'b0}}};

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
    // Leading zeros of a P-bit significand (P when zero)
    function [7:0] lzc;
        input [P-1:0] m;
        integer k;
        reg found;
        begin
            lzc = P;
            found = 1'b0;
            for (k = P - 1; k >= 0; k = k - 1)
                if (!found && m[k]) begin lzc = P - 1 - k; found = 1'b1; end
        end
    endfunction

    // ---------------- Seed table (block RAM) ----------------
    reg [17:0] seed_rom [0:4095];
    `include "fp_seed_rom.vh"
    reg [11:0] rom_addr;
    reg [17:0] seed_q;
    always @(posedge clk) seed_q <= seed_rom[rom_addr];
    wire signed [W-1:0] seed_fx = $signed({{(W-18){1'b0}}, seed_q}) <<< (F - 18);

    // ---------------- Pipelined multiplier (DSP: input, product and output registers) ----------------
    reg  signed [W-1:0]   mul_a, mul_b;
    reg  signed [2*W-1:0] p1, p2;
    always @(posedge clk) begin
        p1 <= mul_a * mul_b;
        p2 <= p1;
    end
    wire signed [W-1:0] prod = p2[F+W-1:F];   // product of two F-fraction numbers, F fraction bits

    localparam S_IDLE = 4'd0, S_CLASS = 4'd1, S_NORMIN = 4'd2, S_TAB = 4'd3, S_MUL = 4'd4,
               S_PACK = 4'd5, S_NORM = 4'd6, S_DENORM = 4'd7, S_ROUND = 4'd8, S_SPECIAL = 4'd9;
    reg [3:0] st_q;
    reg op_sqrt, s_r;
    reg b_pow2;                            // divisor is a power of two: the quotient is exact
    reg [2:0] rm_r;
    reg [N-1:0] xr, yr;
    reg [P-1:0] mx, my;
    reg signed [15:0] ex, ey;
    reg [7:0] lzx, lzy;
    reg signed [15:0] eL_r;
    reg [N-1:0] special_val;
    reg [4:0] special_flags;
    reg [7:0] count;

    // Goldschmidt state: va = q / g, vb = d / h, vt = g*h; ma/mb = operands, r0 = seed
    reg signed [W-1:0] ma, mb, r0, va, vb, vt;
    reg [3:0] rnd;                         // round 0 = seed products, then the steps
    reg [2:0] ph;                          // 0 issue X, 1 issue Y, 3 X result, 4 Y result
    localparam [3:0] LAST_DIV = DIV_STEPS;
    localparam [3:0] LAST_SQRT = 2 * SQRT_STEPS;   // a sqrt step is two rounds (g*h, then update)
    wire last_rnd = (rnd == (op_sqrt ? LAST_SQRT : LAST_DIV));
    wire sq_gh = op_sqrt && rnd[0];       // sqrt round computing g*h
    wire signed [W-1:0] f_div = TWO - vb;
    wire signed [W-1:0] r_sq = HALF - vt;
    // Second product of the round, skipped when its result is not needed
    wire two_ops = (rnd == 4'd0) ? !op_sqrt : (!sq_gh && !last_rnd);

    // Normalized significands (leading one at P-1) and the fixed-point operands
    wire [P-1:0] mxn = mx << lzx;
    wire [P-1:0] myn = my << lzy;
    wire signed [15:0] exn = ex - $signed({8'b0, lzx});
    wire signed [15:0] eyn = ey - $signed({8'b0, lzy});
    wire signed [15:0] tq = exn - BIAS;   // x = 1.m * 2^tq
    wire signed [15:0] tq_even = tq[0] ? tq - 16'sd1 : tq;
    wire signed [W-1:0] mx_fx = $signed({{(W-P){1'b0}}, mxn}) <<< (F - MW);
    wire signed [W-1:0] my_fx = $signed({{(W-P){1'b0}}, myn}) <<< (F - MW);

    // Rounding state: M = {result bits, sticky}; value = M * 2^eL_r
    reg [IT:0] M;
    reg msticky;
    reg [1:0] lead;
    reg signed [15:0] E;
    reg tiny_r;

    wire [IT:0] M_sh = (lead == 2'd0) ? (M >> (IT - MW)) : (lead == 2'd1) ? (M >> (IT - 1 - MW)) : (M >> (IT - 2 - MW));
    wire [P-1:0] mant = M_sh[P-1:0];
    wire g_bit = (lead == 2'd0) ? M[IT - MW - 1] : (lead == 2'd1) ? M[IT - MW - 2] : M[IT - MW - 3];
    wire [IT:0] below_mask = (lead == 2'd0) ? ((1 << (IT - MW - 1)) - 1) : (lead == 2'd1) ? ((1 << (IT - MW - 2)) - 1) : ((1 << (IT - MW - 3)) - 1);
    wire st_bit = ((M & below_mask) != 0) || msticky;
    wire inc = round_up(rm_r, s_r, mant[0], g_bit, st_bit);
    wire [P:0] mant_r = {1'b0, mant} + inc;

    // Result of the multiply rounds, clamped below 2 (approximation error at the top end)
    wire signed [W-1:0] vfin = va[W-1] ? {W{1'b0}} :
                                (va[W-2:F+1] != 0) ? ({{(W-F-1){1'b0}}, {(F+1){1'b1}}}) : va;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            st_q <= S_IDLE;
            done <= 1'b0;
            count <= 0;
            result <= 0;
            flags <= 0;
            rnd <= 0;
            ph <= 0;
        end else begin
            done <= 1'b0;
            case (st_q)
                S_IDLE: if (start) begin
                    xr <= x;
                    yr <= y;
                    op_sqrt <= is_sqrt;
                    rm_r <= rm;
                    st_q <= S_CLASS;
                end

                S_CLASS: begin
                    special_flags <= 5'b0;
                    st_q <= S_SPECIAL;
                    mx <= {(xr[N-2:MW] != 0), xr[MW-1:0]};
                    my <= {(yr[N-2:MW] != 0), yr[MW-1:0]};
                    ex <= (xr[N-2:MW] == 0) ? 16'sd1 : $signed({{(16-EW){1'b0}}, xr[N-2:MW]});
                    ey <= (yr[N-2:MW] == 0) ? 16'sd1 : $signed({{(16-EW){1'b0}}, yr[N-2:MW]});
                    lzx <= lzc({(xr[N-2:MW] != 0), xr[MW-1:0]});
                    lzy <= lzc({(yr[N-2:MW] != 0), yr[MW-1:0]});
                    if (op_sqrt) begin
                        s_r <= 1'b0;
                        if (is_nan(xr)) begin special_val <= QNAN; special_flags <= {is_snan(xr), 4'b0}; end
                        else if (is_zero(xr)) special_val <= xr;
                        else if (xr[N-1]) begin special_val <= QNAN; special_flags <= 5'b10000; end
                        else if (is_inf(xr)) special_val <= xr;
                        else st_q <= S_NORMIN;
                    end else begin
                        s_r <= xr[N-1] ^ yr[N-1];
                        if (is_nan(xr) || is_nan(yr)) begin
                            special_val <= QNAN; special_flags <= {is_snan(xr) || is_snan(yr), 4'b0};
                        end else if ((is_inf(xr) && is_inf(yr)) || (is_zero(xr) && is_zero(yr))) begin
                            special_val <= QNAN; special_flags <= 5'b10000;
                        end else if (is_inf(xr) || is_zero(yr)) begin
                            special_val <= {xr[N-1] ^ yr[N-1], {EW{1'b1}}, {MW{1'b0}}};
                            special_flags <= (is_zero(yr) && !is_inf(xr)) ? 5'b01000 : 5'b0;
                        end else if (is_zero(xr) || is_inf(yr)) begin
                            special_val <= {xr[N-1] ^ yr[N-1], {(N-1){1'b0}}};
                        end else st_q <= S_NORMIN;
                    end
                end

                S_NORMIN: begin
                    st_q <= S_TAB;
                    if (op_sqrt) begin
                        // u = 1.m (tq even) or 2 * 1.m (tq odd), so sqrt(x) = sqrt(u) * 2^(tq/2)
                        ma <= tq[0] ? (mx_fx <<< 1) : mx_fx;
                        eL_r <= (tq_even >>> 1) - IT;
                        rom_addr <= {1'b1, tq[0], mxn[MW-1 -: 10]};
                    end else begin
                        ma <= mx_fx;
                        mb <= my_fx;
                        b_pow2 <= (myn[MW-1:0] == 0);
                        eL_r <= exn - eyn - IT;
                        rom_addr <= {1'b0, myn[MW-1 -: 11]};
                    end
                end

                S_TAB: begin                    // seed_q is read during this cycle
                    st_q <= S_MUL;
                    rnd <= 0;
                    ph <= 0;
                    if (!op_sqrt && b_pow2) begin
                        va <= ma;               // a / 2^k: just the exponent changes
                        st_q <= S_PACK;
                    end
                end

                S_MUL: begin
                    ph <= ph + 3'd1;
                    if (ph == 3'd0) begin
                        if (rnd == 4'd0) begin
                            r0 <= seed_fx;
                            mul_a <= ma; mul_b <= seed_fx;               // q = a*r  / g = u*y
                            if (op_sqrt) vb <= seed_fx >>> 1;           // h = y/2
                        end else if (op_sqrt) begin
                            if (sq_gh) begin mul_a <= va; mul_b <= vb; end   // g*h
                            else begin mul_a <= va; mul_b <= r_sq; end       // g*r
                        end else begin
                            mul_a <= va; mul_b <= f_div;                 // q*f
                        end
                    end else if (ph == 3'd1) begin
                        if (rnd == 4'd0) begin mul_a <= mb; mul_b <= r0; end  // d = b*r
                        else if (op_sqrt) begin mul_a <= vb; mul_b <= r_sq; end  // h*r
                        else begin mul_a <= vb; mul_b <= f_div; end      // d*f
                    end else if (ph == 3'd3) begin
                        if (rnd == 4'd0) va <= prod;
                        else if (sq_gh) vt <= prod;
                        else if (op_sqrt) va <= va + prod;
                        else va <= prod;
                        if (!two_ops) begin
                            ph <= 3'd0;
                            rnd <= rnd + 4'd1;
                            if (last_rnd) st_q <= S_PACK;
                        end
                    end else if (ph == 3'd4) begin
                        if (rnd == 4'd0) vb <= prod;
                        else if (op_sqrt) vb <= vb + prod;
                        else vb <= prod;
                        ph <= 3'd0;
                        rnd <= rnd + 4'd1;
                        if (last_rnd) st_q <= S_PACK;
                    end
                end

                S_PACK: begin
                    // Bit F of the result (weight 1) goes to M[IT]; everything below IT is sticky
                    M <= {vfin[F -: IT], (vfin[F-IT:0] != 0)};
                    msticky <= 1'b0;
                    count <= 0;
                    st_q <= S_NORM;
                end

                S_SPECIAL: begin
                    st_q <= S_IDLE;
                    done <= 1'b1;
                    result <= special_val;
                    flags <= special_flags;
                end

                // ---------------- Rounding (as in fp_divsqrt) ----------------
                S_NORM: begin
                    lead <= M[IT] ? 2'd0 : M[IT-1] ? 2'd1 : 2'd2;
                    E <= eL_r + BIAS + (M[IT] ? IT : M[IT-1] ? IT - 1 : IT - 2);
                    st_q <= S_DENORM;
                end

                S_DENORM: begin
                    if (E == 16'sd0 && !count[7]) begin
                        tiny_r <= !mant_r[P];
                        count[7] <= 1'b1;
                    end else if (E < 16'sd0 && !count[7]) begin
                        tiny_r <= 1'b1;
                        count[7] <= 1'b1;
                    end else if (E < 16'sd1) begin
                        M <= M >> 1;
                        msticky <= msticky | M[0];
                        E <= E + 1;
                        if (M == 0) E <= 16'sd1;
                    end else begin
                        if (!count[7]) tiny_r <= 1'b0;
                        st_q <= S_ROUND;
                    end
                end

                S_ROUND: begin
                    st_q <= S_IDLE;
                    done <= 1'b1;
                    if (E + (mant_r[P] ? 1 : 0) >= EMAX) begin
                        flags <= 5'b00101;
                        if (rm_r == 3'd1 || (rm_r == 3'd2 && !s_r) || (rm_r == 3'd3 && s_r))
                            result <= {s_r, {(EW-1){1'b1}}, 1'b0, {MW{1'b1}}};
                        else
                            result <= {s_r, {EW{1'b1}}, {MW{1'b0}}};
                    end else begin
                        flags <= {3'b0, tiny_r && (g_bit || st_bit), g_bit || st_bit};
                        if (mant_r[P])
                            result <= {s_r, E[EW-1:0] + 1'b1, {MW{1'b0}}};
                        else if (mant_r[P-1])
                            result <= {s_r, E[EW-1:0], mant_r[MW-1:0]};
                        else
                            result <= {s_r, {EW{1'b0}}, mant_r[MW-1:0]};
                    end
                end

                default: st_q <= S_IDLE;
            endcase
        end
    end

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
