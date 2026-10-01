`default_nettype none
// Behavioral AXI3 slave standing in for the Zynq PS DDR behind S_AXI_HP0 (simulation only).
// Word layout matches unified_mem, so the same hex images load: DRAM window at word 0,
// the 1 MB data region after INSTR_MEM_SIZE. Image path: +hex=<file>.
module axi_ddr_model #(
    parameter [31:0] DDR_BASE = 32'h20000000,
    parameter MEM_WORDS = 17039360,       // 64 MB + 1 MB (the Linux build overrides: 256 MB)
    parameter READ_LATENCY = 12,          // cycles from AR to first R beat
    parameter STALL_EVERY = 0             // >0: drop ready/valid every Nth cycle
) (
    input  wire        clk,
    input  wire        rst,
    input  wire [31:0] awaddr,
    input  wire [3:0]  awlen,
    input  wire [2:0]  awsize,
    input  wire        awvalid,
    output wire        awready,
    input  wire [63:0] wdata,
    input  wire [7:0]  wstrb,
    input  wire        wlast,
    input  wire        wvalid,
    output wire        wready,
    output reg         bvalid,
    input  wire        bready,
    input  wire [31:0] araddr,
    input  wire [3:0]  arlen,
    input  wire [2:0]  arsize,
    input  wire        arvalid,
    output wire        arready,
    output reg  [63:0] rdata,
    output reg         rlast,
    output reg         rvalid,
    input  wire        rready
);
    localparam [31:0] DATA_OFFSET = 32'h10000000;
    localparam [31:0] INSTR_WORDS = 32'h04000000 / 4;

    reg [31:0] mem [0:MEM_WORDS-1];
    reg [1023:0] hex_path;
    integer i;
    // Runtime overrides: +ddr_latency=N +ddr_stall=N (stall every Nth cycle)
    integer read_latency, stall_every;
    initial begin
        read_latency = READ_LATENCY;
        stall_every = STALL_EVERY;
        if ($value$plusargs("ddr_latency=%d", read_latency)) ;
        if ($value$plusargs("ddr_stall=%d", stall_every)) ;
        for (i = 0; i < MEM_WORDS; i = i + 1) mem[i] = 32'h0;
        if ($value$plusargs("hex=%s", hex_path)) begin
            $display("axi_ddr_model: loading %0s", hex_path);
            $readmemh(hex_path, mem);
        end
    end

    function [31:0] word_index;
        input [31:0] a;
        begin
            if (a >= DDR_BASE + DATA_OFFSET) word_index = INSTR_WORDS + ((a - DDR_BASE - DATA_OFFSET) >> 2);
            else                             word_index = (a - DDR_BASE) >> 2;
        end
    endfunction

    reg [31:0] cycle;
    always @(posedge clk) cycle <= rst ? 32'd0 : cycle + 32'd1;
    wire stall = (stall_every > 0) && (cycle % stall_every == 0);

    // ---------------- Writes ----------------
    reg w_active;
    reg [31:0] w_addr;
    reg [2:0] w_size;
    assign awready = !w_active && !bvalid && !stall;
    assign wready = w_active && !stall;

    always @(posedge clk) begin
        if (rst) begin
            w_active <= 1'b0;
            bvalid <= 1'b0;
        end else begin
            if (bvalid && bready) bvalid <= 1'b0;
            if (awvalid && awready) begin
                w_active <= 1'b1;
                w_addr <= awaddr;
                w_size <= awsize;
            end
            if (wvalid && wready) begin : write_beat
                reg [31:0] lo, hi;
                lo = word_index({w_addr[31:3], 3'b000});
                hi = lo + 1;
                if (lo < MEM_WORDS) begin
                    if (wstrb[0]) mem[lo][7:0]   <= wdata[7:0];
                    if (wstrb[1]) mem[lo][15:8]  <= wdata[15:8];
                    if (wstrb[2]) mem[lo][23:16] <= wdata[23:16];
                    if (wstrb[3]) mem[lo][31:24] <= wdata[31:24];
                end
                if (hi < MEM_WORDS) begin
                    if (wstrb[4]) mem[hi][7:0]   <= wdata[39:32];
                    if (wstrb[5]) mem[hi][15:8]  <= wdata[47:40];
                    if (wstrb[6]) mem[hi][23:16] <= wdata[55:48];
                    if (wstrb[7]) mem[hi][31:24] <= wdata[63:56];
                end
                w_addr <= w_addr + (32'd1 << w_size);
                if (wlast) begin
                    w_active <= 1'b0;
                    bvalid <= 1'b1;
                end
            end
        end
    end

    // ---------------- Reads ----------------
    reg r_active;
    reg [31:0] r_addr;
    reg [2:0] r_size;
    reg [3:0] r_left;
    reg [7:0] r_wait;
    assign arready = !r_active && !stall;

    function [63:0] read_pair;
        input [31:0] a;
        reg [31:0] lo;
        begin
            lo = word_index({a[31:3], 3'b000});
            read_pair = {(lo + 1 < MEM_WORDS) ? mem[lo + 1] : 32'h0,
                         (lo < MEM_WORDS) ? mem[lo] : 32'h0};
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            r_active <= 1'b0;
            rvalid <= 1'b0;
            rlast <= 1'b0;
        end else begin
            if (arvalid && arready) begin
                r_active <= 1'b1;
                r_addr <= araddr;
                r_size <= arsize;
                r_left <= arlen;
                r_wait <= read_latency[7:0];
            end else if (r_active) begin
                if (rvalid && rready) begin
                    if (rlast) begin
                        rvalid <= 1'b0;
                        r_active <= 1'b0;
                    end else begin
                        rvalid <= 1'b0;
                    end
                end
                if (r_wait != 0) begin
                    r_wait <= r_wait - 8'd1;
                end else if ((!rvalid || rready) && !(rvalid && rlast) && !stall) begin
                    rdata <= read_pair(r_addr);
                    rvalid <= 1'b1;
                    rlast <= (r_left == 4'd0);
                    r_addr <= r_addr + (32'd1 << r_size);
                    r_left <= r_left - 4'd1;
                end
            end
        end
    end

endmodule
