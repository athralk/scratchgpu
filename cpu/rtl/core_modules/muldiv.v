`default_nettype none
// Multi-cycle RV32M unit for the EX stage.
//
// One shared 33x33 signed multiplier (two register stages so it maps onto DSP48E1
// pipeline registers) and one radix-2 restoring divider. EX holds the instruction
// until `ready`; the core stalls the front end and bubbles EX/MEM meanwhile.
//   MUL*      : 3 cycles in EX (latch, multiply, product register)
//   DIV*/REM* : 34 cycles (setup, 32 iterations, sign fix-up); /0 and overflow take 2
module muldiv (
    input wire clk,
    input wire rst,
    input wire req,            // a valid M instruction is in EX
    input wire kill,           // EX is squashed this cycle (trap, redirect)
    input wire advance,        // EX hands its result to EX/MEM this cycle
    input wire [6:0] instr_id,
    input wire [31:0] a,       // forwarded rs1
    input wire [31:0] b,       // forwarded rs2
    output wire ready,
    output wire [31:0] result
);
`include "instr_defines.vh"

    localparam [2:0] S_IDLE = 3'd0,
                     S_MUL1 = 3'd1,
                     S_MUL2 = 3'd2,
                     S_DIV  = 3'd3,
                     S_FIX  = 3'd4,
                     S_DONE = 3'd5;

    reg [2:0] state;
    reg [31:0] result_q;

    // ---------------- Multiplier ----------------
    wire op_is_mul = (instr_id == INSTR_MUL) || (instr_id == INSTR_MULH) ||
                     (instr_id == INSTR_MULHSU) || (instr_id == INSTR_MULHU);
    wire a_signed = (instr_id == INSTR_MUL) || (instr_id == INSTR_MULH) || (instr_id == INSTR_MULHSU);
    wire b_signed = (instr_id == INSTR_MUL) || (instr_id == INSTR_MULH);

    reg signed [32:0] mul_a_q;
    reg signed [32:0] mul_b_q;
    reg signed [65:0] mul_p_q;
    reg mul_high_q;

    // ---------------- Divider ----------------
    wire op_is_signed_div = (instr_id == INSTR_DIV) || (instr_id == INSTR_REM);
    wire op_is_rem = (instr_id == INSTR_REM) || (instr_id == INSTR_REMU);
    wire a_neg = op_is_signed_div && a[31];
    wire b_neg = op_is_signed_div && b[31];
    wire [31:0] a_mag = a_neg ? (~a + 32'd1) : a;
    wire [31:0] b_mag = b_neg ? (~b + 32'd1) : b;
    wire div_by_zero = (b == 32'd0);
    wire div_overflow = op_is_signed_div && (a == 32'h80000000) && (b == 32'hFFFFFFFF);

    reg [31:0] quo_q;      // dividend shifts out, quotient shifts in
    reg [31:0] rem_q;
    reg [31:0] divisor_q;
    reg [4:0] count_q;
    reg want_rem_q;
    reg negate_q;

    wire [32:0] rem_shift = {rem_q, quo_q[31]};
    wire [32:0] rem_diff = rem_shift - {1'b0, divisor_q};
    wire rem_ge = !rem_diff[32];
    wire [31:0] div_raw = want_rem_q ? rem_q : quo_q;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            result_q <= 32'b0;
            mul_a_q <= 33'sd0;
            mul_b_q <= 33'sd0;
            mul_p_q <= 66'sd0;
            mul_high_q <= 1'b0;
            quo_q <= 32'b0;
            rem_q <= 32'b0;
            divisor_q <= 32'b0;
            count_q <= 5'd0;
            want_rem_q <= 1'b0;
            negate_q <= 1'b0;
        end else if (kill) begin
            state <= S_IDLE;
        end else begin
            case (state)
                S_IDLE: if (req) begin
                    if (op_is_mul) begin
                        mul_a_q <= {a_signed && a[31], a};
                        mul_b_q <= {b_signed && b[31], b};
                        mul_high_q <= (instr_id != INSTR_MUL);
                        state <= S_MUL1;
                    end else if (div_by_zero) begin
                        result_q <= op_is_rem ? a : 32'hFFFFFFFF;
                        state <= S_DONE;
                    end else if (div_overflow) begin
                        result_q <= op_is_rem ? 32'h0 : 32'h80000000;
                        state <= S_DONE;
                    end else begin
                        quo_q <= a_mag;
                        rem_q <= 32'b0;
                        divisor_q <= b_mag;
                        count_q <= 5'd0;
                        want_rem_q <= op_is_rem;
                        // Quotient is negative when signs differ; remainder takes the dividend's sign.
                        negate_q <= op_is_rem ? a_neg : (a_neg ^ b_neg);
                        state <= S_DIV;
                    end
                end
                S_MUL1: begin
                    mul_p_q <= mul_a_q * mul_b_q;
                    state <= S_MUL2;
                end
                S_MUL2: begin
                    result_q <= mul_high_q ? mul_p_q[63:32] : mul_p_q[31:0];
                    state <= S_DONE;
                end
                S_DIV: begin
                    rem_q <= rem_ge ? rem_diff[31:0] : rem_shift[31:0];
                    quo_q <= {quo_q[30:0], rem_ge};
                    count_q <= count_q + 5'd1;
                    if (count_q == 5'd31) state <= S_FIX;
                end
                S_FIX: begin
                    result_q <= negate_q ? (~div_raw + 32'd1) : div_raw;
                    state <= S_DONE;
                end
                S_DONE: if (advance) state <= S_IDLE;
                default: state <= S_IDLE;
            endcase
        end
    end

    assign ready = (state == S_DONE);
    assign result = result_q;

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
