`default_nettype none
// Instruction cache: 8 KB, 2-way, 32-byte lines, VIPT (4 KB per way = one page, no aliasing).
//
// The arrays are synchronous (BRAM) and indexed with the core's next-cycle PC, so a hit
// answers in the same cycle the PC is presented: the core sees zero fetch latency on hits.
// Data is stored as 64-bit words so a refill writes one AXI beat per cycle.
module icache (
    input  wire        clk,
    input  wire        rst,
    input  wire        invalidate,      // FENCE.I

    // Core side
    input  wire [31:0] pc_next,         // indexes the arrays
    input  wire [31:0] pc,              // the fetch being answered this cycle
    input  wire        lookup_valid,    // translation ready (TLB hit or bare)
    input  wire        cacheable,       // physical address is DRAM
    input  wire [19:0] ppn,             // physical page of pc
    output wire        hit,
    output wire [31:0] instr,

    // Refill port (line read, 4 x 64-bit beats)
    output reg         rf_req,
    output reg  [31:0] rf_addr,
    output reg         rf_order,        // read must wait for buffered stores (after FENCE.I)
    input  wire        rf_gnt,
    input  wire        rf_beat,
    input  wire [63:0] rf_data,
    input  wire        rf_last
);

    localparam SETS = 128;

    // ---------------- Arrays ----------------
    reg [63:0] data0 [0:SETS*4-1];
    reg [63:0] data1 [0:SETS*4-1];
    reg [19:0] tag0 [0:SETS-1];
    reg [19:0] tag1 [0:SETS-1];
    reg [SETS-1:0] valid0;
    reg [SETS-1:0] valid1;
    reg [SETS-1:0] lru;              // way to replace next

    wire [6:0] rd_set = pc_next[11:5];
    wire [8:0] rd_word = pc_next[11:3];

    reg [63:0] data0_q, data1_q;
    reg [19:0] tag0_q, tag1_q;
    reg valid0_q, valid1_q;

    // Refill write port: each beat is written in the cycle it arrives.
    wire       fill_we;
    wire       fill_way;
    wire [8:0] fill_word;
    wire [63:0] fill_data;

    always @(posedge clk) begin
        data0_q <= data0[rd_word];
        data1_q <= data1[rd_word];
        tag0_q <= tag0[rd_set];
        tag1_q <= tag1[rd_set];
        if (fill_we && !fill_way) data0[fill_word] <= fill_data;
        if (fill_we &&  fill_way) data1[fill_word] <= fill_data;
    end

    // ---------------- Lookup ----------------
    wire hit0 = valid0_q && (tag0_q == ppn);
    wire hit1 = valid1_q && (tag1_q == ppn);
    wire [63:0] hit_data = hit1 ? data1_q : data0_q;
    assign hit = lookup_valid && cacheable && (hit0 || hit1);
    // Fetches from non-DRAM space are answered with an all-zero (illegal) instruction.
    assign instr = !cacheable ? 32'h0 : (pc[2] ? hit_data[63:32] : hit_data[31:0]);

    // ---------------- Refill FSM ----------------
    // S_SETTLE: one cycle after the last beat, while the registered valid read catches up;
    // without it the just-filled line still looks like a miss and is fetched again.
    localparam S_IDLE = 2'd0, S_REQ = 2'd1, S_FILL = 2'd2, S_SETTLE = 2'd3;
    reg [1:0] state;
    reg [1:0] beat;
    reg [6:0] miss_set;
    reg [19:0] miss_tag;
    reg miss_way;
    reg drain_needed;
    reg fill_cancelled;   // FENCE.I arrived mid-refill: the line lands but stays invalid

    assign fill_we = (state == S_FILL) && rf_beat;
    assign fill_way = miss_way;
    assign fill_word = {miss_set, beat};
    assign fill_data = rf_data;

    wire miss = lookup_valid && cacheable && !(hit0 || hit1);
    wire victim = !valid0_q ? 1'b0 : !valid1_q ? 1'b1 : lru[pc[11:5]];

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            rf_req <= 1'b0;
            rf_addr <= 32'b0;
            rf_order <= 1'b0;
            beat <= 2'b0;
            miss_set <= 7'b0;
            miss_tag <= 20'b0;
            miss_way <= 1'b0;
            drain_needed <= 1'b0;
            fill_cancelled <= 1'b0;
            valid0 <= {SETS{1'b0}};
            valid1 <= {SETS{1'b0}};
            lru <= {SETS{1'b0}};
        end else begin
            if (hit) lru[pc[11:5]] <= hit0;  // replace the other way next time

            case (state)
                S_IDLE: if (miss && !invalidate) begin
                    miss_set <= pc[11:5];
                    miss_tag <= ppn;
                    miss_way <= victim;
                    // The victim's old contents die now: its tag is rewritten below.
                    if (!victim) valid0[pc[11:5]] <= 1'b0;
                    else         valid1[pc[11:5]] <= 1'b0;
                    rf_addr <= {ppn, pc[11:5], 5'b0};
                    rf_order <= drain_needed;
                    rf_req <= 1'b1;
                    beat <= 2'b0;
                    fill_cancelled <= 1'b0;
                    state <= S_REQ;
                end
                S_REQ: if (rf_gnt) begin
                    rf_req <= 1'b0;
                    drain_needed <= 1'b0;
                    state <= S_FILL;
                end
                S_FILL: if (rf_beat) begin
                    beat <= beat + 2'd1;
                    // Valid rises at the same edge as the last data write; the registered
                    // valid read lags one cycle, so no hit sees the line before its data.
                    if (rf_last) begin
                        if (!fill_cancelled && !invalidate) begin
                            if (!miss_way) valid0[miss_set] <= 1'b1;
                            else           valid1[miss_set] <= 1'b1;
                        end
                        state <= S_SETTLE;
                    end
                end
                S_SETTLE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase

            if (invalidate) begin
                valid0 <= {SETS{1'b0}};
                valid1 <= {SETS{1'b0}};
                drain_needed <= 1'b1;
                if (state != S_IDLE) fill_cancelled <= 1'b1;
            end
        end
    end

    // Tag write when the refill starts; the way's valid bit is already low.
    always @(posedge clk) begin
        if (state == S_IDLE && miss && !invalidate) begin
            if (!victim) tag0[pc[11:5]] <= ppn;
            else         tag1[pc[11:5]] <= ppn;
        end
    end

    // Registered valid read at the next-cycle index, like the BRAM arrays.
    always @(posedge clk) begin
        valid0_q <= valid0[rd_set];
        valid1_q <= valid1[rd_set];
    end

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
