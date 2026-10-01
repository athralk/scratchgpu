`default_nettype none
// Compact IEEE fused multiply-add for any binary format: result = a*b + c, one rounding.
//   fp_fma #(.EW(8),  .MW(23))  FP32    (same datapath as fp32_fma, which is its proven twin)
//   fp_fma #(.EW(11), .MW(52))  FP64
// Signs are applied by the caller. RISC-V NaN rules; flags match softfloat (tininess after
// rounding). P = MW+1 significand bits; a (3P+5)-bit window holds the product at [2P+1:2],
// the addend's leading bit starts at 3P+3 and shifts right; bit 0 is a sticky slot.
module fp_fma #(
    parameter EW = 11,
    parameter MW = 52
) (
    input  wire [EW+MW:0] a,
    input  wire [EW+MW:0] b,
    input  wire [EW+MW:0] c,
    input  wire [2:0]     rm,
    input  wire           mul,         // plain multiply (c ignored): a zero product keeps its own sign
    output reg  [EW+MW:0] result,
    output reg  [4:0]     flags        // NV DZ OF UF NX
);
    localparam P = MW + 1;
    localparam W = 3 * P + 5;
    localparam N = EW + MW + 1;
    localparam SW = EW + 4;                           // signed exponent arithmetic width
    localparam integer BIAS = (1 << (EW - 1)) - 1;
    localparam integer EMAX = (1 << EW) - 1;
    localparam [N-1:0] QNAN = {1'b0, {EW{1'b1}}, 1'b1, {(MW-1){1'b0}}};

    wire [EW-1:0] ea_f = a[N-2:MW], eb_f = b[N-2:MW], ec_f = c[N-2:MW];
    wire a_nan = (&ea_f) && (a[MW-1:0] != 0);
    wire b_nan = (&eb_f) && (b[MW-1:0] != 0);
    wire c_nan = (&ec_f) && (c[MW-1:0] != 0);
    wire a_snan = a_nan && !a[MW-1];
    wire b_snan = b_nan && !b[MW-1];
    wire c_snan = c_nan && !c[MW-1];
    wire a_inf = (&ea_f) && (a[MW-1:0] == 0);
    wire b_inf = (&eb_f) && (b[MW-1:0] == 0);
    wire c_inf = (&ec_f) && (c[MW-1:0] == 0);
    wire a_zero = (a[N-2:0] == 0);
    wire b_zero = (b[N-2:0] == 0);
    wire c_zero = (c[N-2:0] == 0);

    wire sp = a[N-1] ^ b[N-1];
    wire sc = c[N-1];

    wire [P-1:0] ma = {(ea_f != 0), a[MW-1:0]};
    wire [P-1:0] mb = {(eb_f != 0), b[MW-1:0]};
    wire [P-1:0] mc = {(ec_f != 0), c[MW-1:0]};
    wire signed [SW-1:0] ea = (ea_f == 0) ? 1 : $signed({4'b0, ea_f});
    wire signed [SW-1:0] eb = (eb_f == 0) ? 1 : $signed({4'b0, eb_f});
    wire signed [SW-1:0] ec = (ec_f == 0) ? 1 : $signed({4'b0, ec_f});

    // Product: bit 2P-2 has biased exponent ep
    wire [2*P-1:0] prod = ma * mb;
    wire signed [SW-1:0] ep = ea + eb - BIAS;

    // Addend alignment (pinned at the top when far above the product)
    wire signed [SW-1:0] sh_s = ep + (P + 3) - ec;
    wire [SW-1:0] sh = (sh_s < 0) ? 0 : (sh_s > W) ? W : sh_s;
    wire [W-1:0] c_top = {1'b0, mc, {(2*P+4){1'b0}}};
    wire [W-1:0] c_shift = c_top >> sh;
    wire c_lost = ((c_top & ~({W{1'b1}} << sh)) != 0) || (sh_s > W && mc != 0);
    wire [W-1:0] cw = {c_shift[W-1:1], c_shift[0] | c_lost};
    wire [W-1:0] pw = {{(P+3){1'b0}}, prod, 2'b00};
    wire signed [SW-1:0] ew = (sh_s < 0) ? (ec - (P + 3)) : ep;

    // Add / subtract
    wire eff_sub = sp ^ sc;
    wire [W:0] sum_add = {1'b0, pw} + {1'b0, cw};
    wire [W:0] diff = {1'b0, pw} - {1'b0, cw};
    wire diff_neg = diff[W];
    wire [W-1:0] mag = eff_sub ? (diff_neg ? (~diff[W-1:0] + 1'b1) : diff[W-1:0]) : sum_add[W-1:0];
    wire s_res = eff_sub ? (diff_neg ? sc : sp) : sp;

    // Normalize (top window bit has exponent ew + P + 4)
    reg [SW-1:0] lzc;
    integer i;
    always @(*) begin
        lzc = W;
        for (i = 0; i < W; i = i + 1) if (mag[i]) lzc = W - 1 - i;
    end
    wire signed [SW-1:0] limit = ew + (P + 3);
    reg  [W-1:0] norm;
    reg  signed [SW-1:0] e_res;
    reg  rsh_sticky;
    always @(*) begin
        rsh_sticky = 1'b0;
        if (limit < 0) begin
            norm = (-limit > W - 1) ? {W{1'b0}} : (mag >> (-limit));
            rsh_sticky = (-limit > W - 1) ? (mag != 0) : ((mag & ~({W{1'b1}} << (-limit))) != 0);
            e_res = 1;
        end else if ($signed(lzc) <= limit) begin
            norm = mag << lzc;
            e_res = ew + (P + 4) - $signed(lzc);
        end else begin
            norm = mag << limit;
            e_res = 1;
        end
    end

    wire [P-1:0] mant = norm[W-1 -: P];
    wire guard = norm[W-1-P];
    wire sticky = (norm[W-2-P:0] != 0) || rsh_sticky;

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

    wire inc = round_up(rm, s_res, mant[0], guard, sticky);
    wire [P:0] mant_r = {1'b0, mant} + inc;
    wire carry = mant_r[P];
    wire signed [SW-1:0] e_fin = e_res + (carry ? 1 : 0);
    wire [MW-1:0] frac = carry ? mant_r[P-1:1] : mant_r[MW-1:0];
    wire is_norm = carry || mant_r[P-1];
    wire inexact = guard || sticky;

    // Tininess after rounding (see fp32_fma)
    wire [P-1:0] mant_n = norm[W-2 -: P];
    wire guard_n = norm[W-2-P];
    wire sticky_n = (norm[W-3-P:0] != 0) || rsh_sticky;
    wire tiny = (e_res == 1) && !mant[P-1] &&
                !(mant[P-2] && (&mant_n) && round_up(rm, s_res, mant_n[0], guard_n, sticky_n));

    always @(*) begin
        flags = 5'b0;
        if (a_nan || b_nan || c_nan) begin
            result = QNAN;
            flags[4] = a_snan || b_snan || c_snan || ((a_inf && b_zero) || (a_zero && b_inf));
        end else if ((a_inf && b_zero) || (a_zero && b_inf)) begin
            result = QNAN;
            flags[4] = 1'b1;
        end else if (a_inf || b_inf) begin
            if (c_inf && (sc != sp)) begin result = QNAN; flags[4] = 1'b1; end
            else result = {sp, {EW{1'b1}}, {MW{1'b0}}};
        end else if (c_inf) begin
            result = c;
        end else if (a_zero || b_zero) begin
            if (mul) result = {sp, {(N-1){1'b0}}};
            else if (c_zero) result = {(rm == 3'd2) ? (sp | sc) : (sp & sc), {(N-1){1'b0}}};
            else result = c;
        end else if (mag == 0) begin
            result = {(rm == 3'd2), {(N-1){1'b0}}};
        end else if (e_fin >= EMAX) begin
            flags[2] = 1'b1;
            flags[0] = 1'b1;
            if (rm == 3'd1 || (rm == 3'd2 && !s_res) || (rm == 3'd3 && s_res))
                result = {s_res, {(EW-1){1'b1}}, 1'b0, {MW{1'b1}}};
            else
                result = {s_res, {EW{1'b1}}, {MW{1'b0}}};
        end else begin
            result = {s_res, is_norm ? e_fin[EW-1:0] : {EW{1'b0}}, frac};
            flags[0] = inexact;
            flags[1] = tiny && inexact;
        end
    end

endmodule
