`default_nettype none
// Vector register file: 32 registers x 512 bits, as 16 banks (one per lane) of 32 x 32-bit.
//
// Three read ports, each returning a whole register (512 bits); the 16-lane datapath uses
// every bank, the element sequencer muxes the word it needs. One write port with a
// per-bank enable. Each bank is 32 deep with 3 asynchronous reads + 1 write: one RAM32M
// column per 2 bits on 7-series, so the whole file costs about 1k LUTs.
// v0 (the mask register) is also kept in flops so every lane can see its mask bits.
module vpu_vrf (
    input  wire         clk,
    input  wire [4:0]   ra,
    input  wire [4:0]   rb,
    input  wire [4:0]   rd,
    output wire [511:0] va,
    output wire [511:0] vb,
    output wire [511:0] vd,
    input  wire [4:0]   wreg,
    input  wire [15:0]  wen,      // per 32-bit word (bank)
    input  wire [511:0] wdata,
    output reg  [511:0] v0
);
    genvar l;
    generate
        for (l = 0; l < 16; l = l + 1) begin : bank
            reg [31:0] mem [0:31];
            integer i;
            initial for (i = 0; i < 32; i = i + 1) mem[i] = 32'h0;
            always @(posedge clk) if (wen[l]) mem[wreg] <= wdata[32*l +: 32];
            assign va[32*l +: 32] = mem[ra];
            assign vb[32*l +: 32] = mem[rb];
            assign vd[32*l +: 32] = mem[rd];
        end
    endgenerate

    integer k;
    initial v0 = 512'b0;
    always @(posedge clk) begin
        if (wreg == 5'd0)
            for (k = 0; k < 16; k = k + 1)
                if (wen[k]) v0[32*k +: 32] <= wdata[32*k +: 32];
    end
endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
