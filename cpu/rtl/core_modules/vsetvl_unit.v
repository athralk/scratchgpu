`default_nettype none
// vsetvli / vsetivli / vsetvl for VLEN=512, ELEN=32 (combinational, EX stage).
//   vl = min(AVL, VLMAX), as Spike does. An unsupported vtype sets vill and vl = 0.
module vsetvl_unit (
    input  wire [31:0] instr,
    input  wire [31:0] rs1_value,
    input  wire [31:0] rs2_value,
    input  wire [31:0] cur_vl,
    output reg  [31:0] new_vl,
    output reg  [31:0] new_vtype
);
    wire is_vsetivli = (instr[31:30] == 2'b11);
    wire is_vsetvl   = (instr[31:25] == 7'b1000000);
    wire [4:0] rd  = instr[11:7];
    wire [4:0] rs1 = instr[19:15];

    // Requested vtype; any reserved bit set makes it invalid.
    wire [31:0] req = is_vsetvl   ? rs2_value :
                      is_vsetivli ? {22'b0, instr[29:20]} :
                                    {21'b0, instr[30:20]};
    wire [2:0] vsew  = req[5:3];
    wire [2:0] vlmul = req[2:0];

    // Fractional LMUL needs SEW <= LMUL * ELEN: mf2 -> e8/e16, mf4 -> e8, mf8 -> none.
    wire vtype_ok = (req[30:8] == 23'b0) && !req[31] && (vsew <= 3'd2) && (vlmul != 3'b100) &&
                    !(vlmul == 3'b111 && vsew > 3'd1) &&
                    !(vlmul == 3'b110 && vsew > 3'd0) &&
                    (vlmul != 3'b101);

    // VLMAX = VLEN/SEW * LMUL = (64 >> log2(SEW/8)) scaled by LMUL; up to 512 (e8, m8)
    wire [9:0] per_reg = 10'd64 >> vsew;
    reg [9:0] vlmax_full;
    always @(*) begin
        case (vlmul)
            3'b000: vlmax_full = per_reg;
            3'b001: vlmax_full = per_reg << 1;
            3'b010: vlmax_full = per_reg << 2;
            3'b011: vlmax_full = per_reg << 3;
            3'b111: vlmax_full = per_reg >> 1;
            3'b110: vlmax_full = per_reg >> 2;
            default: vlmax_full = 10'd0;
        endcase
    end

    wire [31:0] avl = is_vsetivli ? {27'b0, rs1} :
                      (rs1 != 5'd0) ? rs1_value :
                      (rd != 5'd0)  ? 32'hFFFFFFFF :
                                      cur_vl;

    always @(*) begin
        if (!vtype_ok) begin
            new_vtype = 32'h80000000;
            new_vl = 32'd0;
        end else begin
            new_vtype = {24'b0, req[7:0]};
            new_vl = (avl > {22'b0, vlmax_full}) ? {22'b0, vlmax_full} : avl;
        end
    end

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
