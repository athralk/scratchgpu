`default_nettype none
// Data cache: 8 KB, 2-way, 32-byte lines, VIPT, write-through / no-write-allocate,
// with a 4-entry store buffer in front of memory.
//
// Like the I-cache, the arrays are indexed with the MEM address of the next cycle,
// so a load hit answers in the cycle it is presented. Stores complete as soon as the
// store buffer takes them (updating the line too if it hits). Write-through keeps DRAM
// current, which is what lets the vector unit and DMA readers see CPU stores.
//
// Ordering: a load miss, an uncached (MMIO) access and the page walker all wait until
// every buffered store has been acknowledged by memory (`write_idle`).
module dcache (
    input  wire        clk,
    input  wire        rst,

    // Core side (MEM stage). The request is held until rvalid.
    input  wire        req_rd,
    input  wire        req_wr,
    input  wire [31:0] vaddr,
    input  wire [31:0] addr_next,       // indexes the arrays
    input  wire [31:0] wdata,           // low-aligned, as the core presents it
    input  wire [3:0]  be,              // relative to vaddr
    input  wire [2:0]  load_type,
    input  wire        lookup_valid,    // translation ready, no fault
    input  wire [19:0] ppn,
    input  wire        cacheable,
    output wire        rvalid,
    output reg  [31:0] rdata,

    // Line invalidate (CBO.INVAL/FLUSH, vector stores): physical line address
    input  wire        inval_req,
    input  wire [31:0] inval_addr,

    // Refill port (line read)
    output reg         rf_req,
    output reg  [31:0] rf_addr,
    input  wire        rf_gnt,
    input  wire        rf_beat,
    input  wire [63:0] rf_data,
    input  wire        rf_last,

    // Store buffer drain port (one 32-bit word)
    output wire        wb_valid,
    output wire [31:0] wb_addr,
    output wire [31:0] wb_data,         // lane-aligned to wb_addr[1:0] = 0
    output wire [3:0]  wb_be,
    input  wire        wb_pop,
    output wire        wb_empty,
    input  wire        write_idle,      // buffer empty and every write acknowledged

    // Uncached (MMIO) port: one-cycle access, read data combinational
    output reg         io_rd,
    output reg         io_wr,
    output reg  [31:0] io_addr,
    output reg  [31:0] io_wdata,        // low-aligned, as the peripherals expect
    input  wire [31:0] io_rdata
);

    localparam SETS = 128;

    // ---------------- Arrays ----------------
    reg [63:0] data0 [0:SETS*4-1];
    reg [63:0] data1 [0:SETS*4-1];
    reg [19:0] tag0 [0:SETS-1];
    reg [19:0] tag1 [0:SETS-1];
    reg [SETS-1:0] valid0;
    reg [SETS-1:0] valid1;
    reg [SETS-1:0] lru;

    wire [6:0] rd_set = addr_next[11:5];
    wire [8:0] rd_word = addr_next[11:3];

    reg [63:0] data0_q, data1_q;
    reg [19:0] tag0_q, tag1_q;
    reg valid0_q, valid1_q;

    // Single write port: refill beats and store hits.
    wire       wr_en;
    wire       wr_way;
    wire [8:0] wr_word;
    wire [7:0] wr_be;
    wire [63:0] wr_data;

    integer b;
    always @(posedge clk) begin
        data0_q <= data0[rd_word];
        data1_q <= data1[rd_word];
        tag0_q <= tag0[rd_set];
        tag1_q <= tag1[rd_set];
        valid0_q <= valid0[rd_set];
        valid1_q <= valid1[rd_set];
        for (b = 0; b < 8; b = b + 1) begin
            if (wr_en && !wr_way && wr_be[b]) data0[wr_word][8*b +: 8] <= wr_data[8*b +: 8];
            if (wr_en &&  wr_way && wr_be[b]) data1[wr_word][8*b +: 8] <= wr_data[8*b +: 8];
        end
    end

    // ---------------- Lookup ----------------
    wire [6:0] cur_set = vaddr[11:5];
    wire hit0 = valid0_q && (tag0_q == ppn);
    wire hit1 = valid1_q && (tag1_q == ppn);
    wire line_hit = hit0 || hit1;
    wire hit_way = hit1;

    // A store written at the edge that also read this word: the read saw the old bytes.
    reg        byp_valid;
    reg        byp_way;
    reg [8:0]  byp_word;
    reg [7:0]  byp_be;
    reg [63:0] byp_data;
    wire [63:0] raw_line = hit_way ? data1_q : data0_q;
    reg [63:0] line_word;
    always @(*) begin
        line_word = raw_line;
        if (byp_valid && byp_way == hit_way && byp_word == vaddr[11:3]) begin
            for (b = 0; b < 8; b = b + 1)
                if (byp_be[b]) line_word[8*b +: 8] = byp_data[8*b +: 8];
        end
    end
    wire [31:0] hit_word = vaddr[2] ? line_word[63:32] : line_word[31:0];

    // ---------------- Store buffer ----------------
    reg [31:0] sb_addr [0:3];
    reg [31:0] sb_data [0:3];
    reg [3:0]  sb_be [0:3];
    reg [1:0]  sb_head, sb_tail;
    reg [2:0]  sb_count;
    wire sb_full = (sb_count == 3'd4);
    assign wb_empty = (sb_count == 3'd0);
    assign wb_valid = !wb_empty;
    assign wb_addr = sb_addr[sb_head];
    assign wb_data = sb_data[sb_head];
    assign wb_be = sb_be[sb_head];

    wire [31:0] paddr = {ppn, vaddr[11:0]};
    wire [3:0]  be_lane = be << vaddr[1:0];
    wire [31:0] wdata_lane = wdata << {vaddr[1:0], 3'b000};

    wire fsm_idle;
    wire cached_access = lookup_valid && cacheable && fsm_idle;
    wire load_hit = cached_access && req_rd && !req_wr && line_hit;
    wire store_accept = cached_access && req_wr && !sb_full;
    wire store_hit_write = store_accept && line_hit;

    // ---------------- Refill / uncached FSM ----------------
    localparam S_IDLE = 3'd0, S_RF_REQ = 3'd1, S_RF_FILL = 3'd2, S_IO_WAIT = 3'd3,
               S_IO_DONE = 3'd4, S_SETTLE = 3'd5;  // SETTLE: see icache
    reg [2:0] state;
    reg [1:0] beat;
    reg [6:0] miss_set;
    reg miss_way;
    reg [31:0] io_rdata_q;
    reg inval_hit_pending;

    assign fsm_idle = (state == S_IDLE);
    wire victim = !valid0_q ? 1'b0 : !valid1_q ? 1'b1 : lru[cur_set];
    wire load_miss = cached_access && req_rd && !req_wr && !line_hit;
    wire io_access = lookup_valid && !cacheable && (req_rd || req_wr);
    wire refill_beat = (state == S_RF_FILL) && rf_beat;

    assign wr_en   = refill_beat || store_hit_write;
    assign wr_way  = refill_beat ? miss_way : hit_way;
    assign wr_word = refill_beat ? {miss_set, beat} : vaddr[11:3];
    assign wr_be   = refill_beat ? 8'hFF : (vaddr[2] ? {be_lane, 4'b0} : {4'b0, be_lane});
    assign wr_data = refill_beat ? rf_data : {wdata_lane, wdata_lane};

    // ---------------- Response ----------------
    assign rvalid = load_hit || store_accept || (state == S_IO_DONE);

    // Format a load like the core expects: the addressed byte/half in the low bits.
    wire [31:0] load_word = (state == S_IO_DONE) ? io_rdata_q : hit_word;
    wire [31:0] shifted = load_word >> {vaddr[1:0], 3'b000};
    always @(*) begin
        case (load_type)
            3'b000:  rdata = {{24{shifted[7]}}, shifted[7:0]};
            3'b100:  rdata = {24'b0, shifted[7:0]};
            3'b001:  rdata = {{16{shifted[15]}}, shifted[15:0]};
            3'b101:  rdata = {16'b0, shifted[15:0]};
            default: rdata = (state == S_IO_DONE) ? io_rdata_q : hit_word;
        endcase
    end

    // Invalidate lookup: compare the target line against both ways (tags read combinationally
    // from the arrays at that set; a rare operation, so it takes the slow path).
    wire [6:0] inv_set = inval_addr[11:5];
    wire inv_hit0 = valid0[inv_set] && (tag0[inv_set] == inval_addr[31:12]);
    wire inv_hit1 = valid1[inv_set] && (tag1[inv_set] == inval_addr[31:12]);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            beat <= 2'b0;
            miss_set <= 7'b0;
            miss_way <= 1'b0;
            rf_req <= 1'b0;
            rf_addr <= 32'b0;
            io_rd <= 1'b0;
            io_wr <= 1'b0;
            io_addr <= 32'b0;
            io_wdata <= 32'b0;
            io_rdata_q <= 32'b0;
            valid0 <= {SETS{1'b0}};
            valid1 <= {SETS{1'b0}};
            lru <= {SETS{1'b0}};
            sb_head <= 2'b0;
            sb_tail <= 2'b0;
            sb_count <= 3'b0;
            byp_valid <= 1'b0;
            byp_way <= 1'b0;
            byp_word <= 9'b0;
            byp_be <= 8'b0;
            byp_data <= 64'b0;
            inval_hit_pending <= 1'b0;
        end else begin
            io_rd <= 1'b0;
            io_wr <= 1'b0;

            byp_valid <= wr_en;
            byp_way <= wr_way;
            byp_word <= wr_word;
            byp_be <= wr_be;
            byp_data <= wr_data;

            if (load_hit || store_hit_write) lru[cur_set] <= !hit_way;

            // Store buffer push / pop
            if (store_accept) begin
                sb_addr[sb_tail] <= {paddr[31:2], 2'b00};
                sb_data[sb_tail] <= wdata_lane;
                sb_be[sb_tail] <= be_lane;
                sb_tail <= sb_tail + 2'd1;
            end
            if (wb_pop) sb_head <= sb_head + 2'd1;
            sb_count <= sb_count + {2'b0, store_accept} - {2'b0, wb_pop};

            case (state)
                S_IDLE: begin
                    if (load_miss && write_idle) begin
                        miss_set <= cur_set;
                        miss_way <= victim;
                        if (!victim) valid0[cur_set] <= 1'b0;
                        else         valid1[cur_set] <= 1'b0;
                        rf_addr <= {paddr[31:5], 5'b0};
                        rf_req <= 1'b1;
                        beat <= 2'b0;
                        state <= S_RF_REQ;
                    end else if (io_access && write_idle) begin
                        io_rd <= req_rd && !req_wr;
                        io_wr <= req_wr;
                        io_addr <= paddr;
                        io_wdata <= wdata;
                        state <= S_IO_WAIT;
                    end
                end
                S_RF_REQ: if (rf_gnt) begin
                    rf_req <= 1'b0;
                    state <= S_RF_FILL;
                end
                S_RF_FILL: if (rf_beat) begin
                    beat <= beat + 2'd1;
                    if (rf_last) begin
                        if (!inval_hit_pending) begin
                            if (!miss_way) valid0[miss_set] <= 1'b1;
                            else           valid1[miss_set] <= 1'b1;
                        end
                        inval_hit_pending <= 1'b0;
                        state <= S_SETTLE;
                    end
                end
                S_IO_WAIT: begin
                    // io_rd/io_wr were high for exactly the previous cycle; capture the read.
                    io_rdata_q <= io_rdata;
                    state <= S_IO_DONE;
                end
                S_IO_DONE: state <= S_IDLE;
                S_SETTLE: state <= S_IDLE;
                default: state <= S_IDLE;
            endcase

            if (inval_req) begin
                if (inv_hit0) valid0[inv_set] <= 1'b0;
                if (inv_hit1) valid1[inv_set] <= 1'b0;
                // The line being refilled is the one invalidated: do not mark it valid.
                if ((state == S_RF_REQ || state == S_RF_FILL) &&
                    rf_addr[31:5] == inval_addr[31:5])
                    inval_hit_pending <= 1'b1;
            end
        end
    end

    // Tag write at refill start (the victim's valid bit drops at the same edge).
    always @(posedge clk) begin
        if (state == S_IDLE && load_miss && write_idle) begin
            if (!victim) tag0[cur_set] <= ppn;
            else         tag1[cur_set] <= ppn;
        end
    end

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
