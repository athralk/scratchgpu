`default_nettype none

module atomic_lsu (
    input wire clk,
    input wire rst,
    input wire [6:0] instr_id_mem,
    input wire [31:0] mem_addr_mem,
    input wire [31:0] rs2_value_mem,
    input wire [31:0] mem_read_data,
    // High while MEM waits for memory: the access has not happened, so the reservation holds.
    input wire mem_hold,
    input wire non_atomic_store_write_enable,
    input wire [31:0] non_atomic_store_write_addr,

    output wire is_lr_w,
    output wire is_sc_w,
    output wire is_amo_w,
    output wire atomic_read_enable,
    output wire atomic_write_enable,
    output wire sc_success,
    output wire sc_wait,                // SC spends its first MEM cycle checking the reservation
    output wire [31:0] sc_result,
    output reg [31:0] atomic_new_word
);
`include "instr_defines.vh"

    reg lr_valid;
    reg [31:0] lr_addr;

    wire is_amo_sw;
    wire is_amo_add;
    wire is_amo_and;
    wire is_amo_or;
    wire is_amo_xor;
    wire is_amo_max;
    wire is_amo_min;
    wire is_amo_maxu;
    wire is_amo_minu;
    wire atomic_word_aligned;
    wire signed [31:0] old_word_signed;
    wire signed [31:0] rs2_signed;

    assign is_lr_w = (instr_id_mem == INSTR_LR_W);
    assign is_sc_w = (instr_id_mem == INSTR_SC_W);
    assign is_amo_sw = (instr_id_mem == INSTR_AMOSWAP_W);
    assign is_amo_add = (instr_id_mem == INSTR_AMOADD_W);
    assign is_amo_and = (instr_id_mem == INSTR_AMOAND_W);
    assign is_amo_or = (instr_id_mem == INSTR_AMOOR_W);
    assign is_amo_xor = (instr_id_mem == INSTR_AMOXOR_W);
    assign is_amo_max = (instr_id_mem == INSTR_AMOMAX_W);
    assign is_amo_min = (instr_id_mem == INSTR_AMOMIN_W);
    assign is_amo_maxu = (instr_id_mem == INSTR_AMOMAXU_W);
    assign is_amo_minu = (instr_id_mem == INSTR_AMOMINU_W);
    assign is_amo_w = is_amo_sw || is_amo_add || is_amo_and || is_amo_or ||
                      is_amo_xor || is_amo_max || is_amo_min ||
                      is_amo_maxu || is_amo_minu;

    assign atomic_read_enable = is_lr_w || is_amo_w;
    assign atomic_word_aligned = (mem_addr_mem[1:0] == 2'b00);
    // SC checks its reservation (a 30-bit address compare) in its first MEM cycle and writes in
    // the second, so the write enable comes from a register: the compare is not in front of the
    // D-cache request. The core holds MEM while sc_wait is high.
    reg sc_checked, sc_ok;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sc_checked <= 1'b0;
            sc_ok <= 1'b0;
        end else if (!mem_hold) begin
            sc_checked <= 1'b0;                 // the instruction leaves MEM
        end else if (is_sc_w && !sc_checked) begin
            sc_checked <= 1'b1;
            sc_ok <= atomic_word_aligned && lr_valid && (lr_addr == mem_addr_mem);
        end
    end
    assign sc_wait = is_sc_w && !sc_checked;
    assign sc_success = is_sc_w && sc_checked && sc_ok;
    assign atomic_write_enable = sc_success || is_amo_w;
    assign sc_result = sc_success ? 32'h0 : 32'h1;

    assign old_word_signed = mem_read_data;
    assign rs2_signed = rs2_value_mem;

    always @(*) begin
        if (is_amo_sw) begin
            atomic_new_word = rs2_value_mem;
        end else if (is_amo_add) begin
            atomic_new_word = mem_read_data + rs2_value_mem;
        end else if (is_amo_and) begin
            atomic_new_word = mem_read_data & rs2_value_mem;
        end else if (is_amo_or) begin
            atomic_new_word = mem_read_data | rs2_value_mem;
        end else if (is_amo_xor) begin
            atomic_new_word = mem_read_data ^ rs2_value_mem;
        end else if (is_amo_max) begin
            atomic_new_word = ($signed(old_word_signed) >= $signed(rs2_signed)) ?
                              mem_read_data : rs2_value_mem;
        end else if (is_amo_min) begin
            atomic_new_word = ($signed(old_word_signed) <= $signed(rs2_signed)) ?
                              mem_read_data : rs2_value_mem;
        end else if (is_amo_maxu) begin
            atomic_new_word = (mem_read_data >= rs2_value_mem) ?
                              mem_read_data : rs2_value_mem;
        end else if (is_amo_minu) begin
            atomic_new_word = (mem_read_data <= rs2_value_mem) ?
                              mem_read_data : rs2_value_mem;
        end else begin
            atomic_new_word = 32'h0;
        end
    end

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            lr_valid <= 1'b0;
            lr_addr <= 32'b0;
        end else if (!mem_hold) begin
            if (is_lr_w && atomic_word_aligned) begin
                lr_valid <= 1'b1;
                lr_addr <= mem_addr_mem;
            end else if (is_sc_w || is_amo_w ||
                         (non_atomic_store_write_enable &&
                          (non_atomic_store_write_addr[31:2] == lr_addr[31:2]))) begin
                lr_valid <= 1'b0;
            end
        end
    end

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
