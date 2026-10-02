`include "memory_map.vh"
module pc(
   input wire clk,
   input wire rst,
   input wire j_signal,
   input wire stall,         // Added stall input
   input wire [31:0] jump,
   output wire [31:0] out,
   output wire [31:0] next_out  // PC for the next cycle, so a synchronous I-cache can index early
);
   reg [31:0] next_pc = `INSTR_MEM_BASE;

    always @ (posedge clk) begin
        if(rst)
            next_pc <= `INSTR_MEM_BASE;
        else if(j_signal) begin
            next_pc <= jump;
        end
        else if(stall) begin
            // If stalling, don't update PC
            next_pc <= next_pc;
        end
        else begin
            next_pc <= next_pc + 32'h4;
        end
    end

    assign out = next_pc;
    assign next_out = rst      ? `INSTR_MEM_BASE :
                      j_signal ? jump :
                      stall    ? next_pc :
                                 next_pc + 32'h4;
endmodule
