// Accuracy test wrapper for fp_rcpdiv: the FP32 and FP64 configurations used by vpu_top.
// Run: make -C gpu/tb   (Verilator; compares against the host's IEEE divide/sqrt)
module fp_rcpdiv_tb (
    input  wire clk, rst,
    input  wire start32, start64, is_sqrt,
    input  wire [31:0] x32, y32,
    input  wire [63:0] x64, y64,
    input  wire [2:0] rm,
    output wire done32, done64,
    output wire [31:0] res32,
    output wire [63:0] res64,
    output wire [4:0] fl32, fl64
);
    fp_rcpdiv #(.EW(8), .MW(23), .DIV_STEPS(1), .SQRT_STEPS(1)) u32 (
        .clk(clk), .rst(rst), .start(start32), .is_sqrt(is_sqrt), .x(x32), .y(y32), .rm(rm),
        .done(done32), .result(res32), .flags(fl32));
    fp_rcpdiv #(.EW(11), .MW(52), .DIV_STEPS(3), .SQRT_STEPS(3)) u64 (
        .clk(clk), .rst(rst), .start(start64), .is_sqrt(is_sqrt), .x(x64), .y(y64), .rm(rm),
        .done(done64), .result(res64), .flags(fl64));
endmodule
