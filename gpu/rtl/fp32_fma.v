`default_nettype none
// Compact FP32 fused multiply-add: result = a*b + c, one rounding (IEEE 754, RISC-V NaNs).
// Signs are applied by the caller (negate an operand by flipping its sign bit).
//
// Datapath: 24x24 product (DSP), addend aligned into a 76-bit window against the product
// with a sticky bit, one add/subtract, leading-zero count, normalizing shift limited at the
// subnormal boundary, then rounding in any of the five modes. Flags match softfloat
// (tininess detected after rounding). Combinational; the lanes register around it.
module fp32_fma (
    input  wire [31:0] a,
    input  wire [31:0] b,
    input  wire [31:0] c,
    input  wire [2:0]  rm,
    input  wire        mul,         // plain multiply (c ignored): a zero product keeps its own sign
    output reg  [31:0] result,
    output reg  [4:0]  flags        // NV DZ OF UF NX
);
    localparam [31:0] QNAN = 32'h7FC00000;

    wire [7:0] ea_f = a[30:23], eb_f = b[30:23], ec_f = c[30:23];
    wire a_nan = (ea_f == 8'hFF) && (a[22:0] != 0);
    wire b_nan = (eb_f == 8'hFF) && (b[22:0] != 0);
    wire c_nan = (ec_f == 8'hFF) && (c[22:0] != 0);
    wire a_snan = a_nan && !a[22];
    wire b_snan = b_nan && !b[22];
    wire c_snan = c_nan && !c[22];
    wire a_inf = (ea_f == 8'hFF) && (a[22:0] == 0);
    wire b_inf = (eb_f == 8'hFF) && (b[22:0] == 0);
    wire c_inf = (ec_f == 8'hFF) && (c[22:0] == 0);
    wire a_zero = (a[30:0] == 0);
    wire b_zero = (b[30:0] == 0);
    wire c_zero = (c[30:0] == 0);

    wire sp = a[31] ^ b[31];
    wire sc = c[31];

    // Significands with hidden bit; subnormals use exponent 1
    wire [23:0] ma = {(ea_f != 0), a[22:0]};
    wire [23:0] mb = {(eb_f != 0), b[22:0]};
    wire [23:0] mc = {(ec_f != 0), c[22:0]};
    wire signed [10:0] ea = (ea_f == 0) ? 11'sd1 : $signed({3'b0, ea_f});
    wire signed [10:0] eb = (eb_f == 0) ? 11'sd1 : $signed({3'b0, eb_f});
    wire signed [10:0] ec = (ec_f == 0) ? 11'sd1 : $signed({3'b0, ec_f});

    // ---------------- Product ----------------
    // Product bit 46 has (biased) exponent ep = ea + eb - 127.
    wire [47:0] prod = ma * mb;
    wire signed [10:0] ep = ea + eb - 11'sd127;

    // ---------------- Window ----------------
    // 77-bit window, bit 0 is a sticky slot. Vector bit k has exponent ep + (k - 48):
    // the product sits at [49:2] (its bit 46 at 48); the addend's leading bit starts at 75
    // (exponent ep + 27) and is shifted right by sh = ep + 27 - ec. If the addend is even
    // higher (sh < 0) it stays at 75: the product is then below the addend's guard bits and
    // only its nonzero-ness matters, which the fixed placement preserves.
    wire signed [10:0] sh_s = ep + 11'sd27 - ec;
    wire [6:0] sh = (sh_s < 0) ? 7'd0 : (sh_s > 11'sd77) ? 7'd77 : sh_s[6:0];
    wire [76:0] c_top = {1'b0, mc, 52'b0};
    wire [76:0] c_shift = c_top >> sh;
    wire c_lost = ((c_top & ~({77{1'b1}} << sh)) != 0) || (sh_s > 11'sd77 && mc != 0);
    wire [76:0] cw = {c_shift[76:1], c_shift[0] | c_lost};
    wire [76:0] pw = {27'b0, prod, 2'b0};
    // Exponent of window bit 48: the product's, unless the addend was pinned at the top.
    wire signed [10:0] ew = (sh_s < 0) ? (ec - 11'sd27) : ep;

    // ---------------- Add / subtract ----------------
    wire eff_sub = sp ^ sc;
    wire [77:0] sum_add = {1'b0, pw} + {1'b0, cw};
    wire [77:0] diff = {1'b0, pw} - {1'b0, cw};
    wire diff_neg = diff[77];
    wire [76:0] mag = eff_sub ? (diff_neg ? (~diff[76:0] + 77'd1) : diff[76:0]) : sum_add[76:0];
    wire s_res = eff_sub ? (diff_neg ? sc : sp) : sp;

    // ---------------- Normalize ----------------
    // Vector bit 76 has exponent ep + 28. Shift left by the leading-zero count, but keep the
    // exponent of bit 76 >= 1 (subnormal results); a tiny product shifts right instead.
    reg [6:0] lzc;
    integer i;
    always @(*) begin
        lzc = 7'd77;
        for (i = 0; i < 77; i = i + 1) if (mag[i]) lzc = 7'd76 - i[6:0];
    end
    wire signed [10:0] limit = ew + 11'sd27;
    reg  [76:0] norm;
    reg  signed [10:0] e_res;
    reg  rsh_sticky;
    always @(*) begin
        rsh_sticky = 1'b0;
        if (limit < 0) begin
            norm = (-limit > 11'sd76) ? 77'd0 : (mag >> (-limit));
            rsh_sticky = (-limit > 11'sd76) ? (mag != 0) : ((mag & ~({77{1'b1}} << (-limit))) != 0);
            e_res = 11'sd1;
        end else if ($signed({4'b0, lzc}) <= limit) begin
            norm = mag << lzc;
            e_res = ew + 11'sd28 - $signed({4'b0, lzc});
        end else begin
            norm = mag << limit[6:0];
            e_res = 11'sd1;
        end
    end

    // Significand: bits [76:53], guard 52, sticky below
    wire [23:0] mant = norm[76:53];
    wire guard = norm[52];
    wire sticky = (norm[51:0] != 0) || rsh_sticky;

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
    wire [24:0] mant_r = {1'b0, mant} + {24'b0, inc};
    wire carry = mant_r[24];
    wire signed [10:0] e_fin = e_res + (carry ? 11'sd1 : 11'sd0);
    wire [22:0] frac = carry ? mant_r[23:1] : mant_r[22:0];
    wire is_norm = carry || mant_r[23];
    wire inexact = guard || sticky;

    // Tininess after rounding: the value rounded to 24 bits with unbounded exponent < 2^-126.
    // Only results with e_res == 1 and no leading bit (subnormal range) can be tiny; such a
    // result is not tiny if rounding the fully normalized 24-bit significand reaches 2^-126.
    wire [23:0] mant_n = norm[75:52];
    wire guard_n = norm[51];
    wire sticky_n = (norm[50:0] != 0) || rsh_sticky;
    wire tiny = (e_res == 11'sd1) && !mant[23] &&
                !(mant[22] && (&mant_n) && round_up(rm, s_res, mant_n[0], guard_n, sticky_n));

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
            else result = {sp, 8'hFF, 23'b0};
        end else if (c_inf) begin
            result = c;
        end else if (a_zero || b_zero) begin
            if (mul) result = {sp, 31'b0};
            else if (c_zero) result = {(rm == 3'd2) ? (sp | sc) : (sp & sc), 31'b0};
            else result = c;
        end else if (mag == 0) begin
            result = {(rm == 3'd2), 31'b0};       // exact cancellation
        end else if (e_fin >= 11'sd255) begin
            flags[2] = 1'b1;
            flags[0] = 1'b1;
            if (rm == 3'd1 || (rm == 3'd2 && !s_res) || (rm == 3'd3 && s_res))
                result = {s_res, 8'hFE, 23'h7FFFFF};
            else
                result = {s_res, 8'hFF, 23'h0};
        end else begin
            result = {s_res, is_norm ? e_fin[7:0] : 8'h00, frac};
            flags[0] = inexact;
            flags[1] = tiny && inexact;
        end
    end

endmodule
