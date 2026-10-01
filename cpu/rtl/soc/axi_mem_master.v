`default_nettype none
`include "memory_map.vh"
// Memory arbiter and AXI3 master (64-bit) to the Zynq PS DDR through S_AXI_HP0.
//
// Read clients (one outstanding read at a time): page walker (word), D-cache refill
// (line), I-cache refill (line). Writes come from the D-cache store buffer, one word per
// transaction, several in flight. `write_idle` reports that every buffered store has been
// acknowledged; ordered reads (walker, D refill, I refill after FENCE.I) wait for it,
// because AXI does not order a read after a write to the same address on its own.
//
// Address remap: SoC DRAM 0x8000_0000 + n -> DDR_BASE + n; data region 0x1000_0000 + n
// -> DDR_BASE + 0x1000_0000 + n. Anything else never reaches AXI.
module axi_mem_master #(
    parameter [31:0] DDR_BASE = 32'h20000000,
    parameter MAX_WRITES = 4
) (
    input  wire        clk,
    input  wire        rst,

    // Page walker (word read)
    input  wire        ptw_req,
    input  wire [31:0] ptw_addr,
    output wire        ptw_gnt,
    output wire        ptw_rvalid,
    output wire [31:0] ptw_rdata,

    // D-cache refill
    input  wire        drf_req,
    input  wire [31:0] drf_addr,
    output wire        drf_gnt,
    output wire        drf_beat,
    output wire        drf_last,

    // I-cache refill
    input  wire        irf_req,
    input  wire [31:0] irf_addr,
    input  wire        irf_order,
    output wire        irf_gnt,
    output wire        irf_beat,
    output wire        irf_last,

    output wire [63:0] rf_data,          // shared beat data for both refill clients

    // Store buffer
    input  wire        wb_valid,
    input  wire [31:0] wb_addr,
    input  wire [31:0] wb_data,
    input  wire [3:0]  wb_be,
    input  wire        wb_empty,
    output wire        wb_pop,
    output wire        write_idle,

    // Extra ordered-read clients can wait on this (e.g. the vector unit)
    // ---------------- AXI3 master ----------------
    output wire [5:0]  m_axi_awid,
    output wire [31:0] m_axi_awaddr,
    output wire [3:0]  m_axi_awlen,
    output wire [2:0]  m_axi_awsize,
    output wire [1:0]  m_axi_awburst,
    output wire [1:0]  m_axi_awlock,
    output wire [3:0]  m_axi_awcache,
    output wire [2:0]  m_axi_awprot,
    output wire [3:0]  m_axi_awqos,
    output reg         m_axi_awvalid,
    input  wire        m_axi_awready,
    output wire [5:0]  m_axi_wid,
    output reg  [63:0] m_axi_wdata,
    output reg  [7:0]  m_axi_wstrb,
    output wire        m_axi_wlast,
    output reg         m_axi_wvalid,
    input  wire        m_axi_wready,
    input  wire [5:0]  m_axi_bid,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,
    output wire [5:0]  m_axi_arid,
    output reg  [31:0] m_axi_araddr,
    output reg  [3:0]  m_axi_arlen,
    output reg  [2:0]  m_axi_arsize,
    output wire [1:0]  m_axi_arburst,
    output wire [1:0]  m_axi_arlock,
    output wire [3:0]  m_axi_arcache,
    output wire [2:0]  m_axi_arprot,
    output wire [3:0]  m_axi_arqos,
    output reg         m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [5:0]  m_axi_rid,
    input  wire [63:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rlast,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready
);

    function [31:0] remap;
        input [31:0] a;
        begin
            if (a >= `SOC_DRAM_BASE)
                remap = DDR_BASE + (a - `SOC_DRAM_BASE);
            else
                remap = DDR_BASE + 32'h10000000 + (a - `DATA_MEM_BASE);
        end
    endfunction

    // Fixed AXI attributes
    assign m_axi_awid = 6'd0;
    assign m_axi_awlen = 4'd0;
    assign m_axi_awsize = 3'd2;
    assign m_axi_awburst = 2'b01;
    assign m_axi_awlock = 2'b00;
    assign m_axi_awcache = 4'b0011;
    assign m_axi_awprot = 3'b000;
    assign m_axi_awqos = 4'd0;
    assign m_axi_wid = 6'd0;
    assign m_axi_wlast = 1'b1;
    assign m_axi_bready = 1'b1;
    assign m_axi_arid = 6'd0;
    assign m_axi_arburst = 2'b01;
    assign m_axi_arlock = 2'b00;
    assign m_axi_arcache = 4'b0011;
    assign m_axi_arprot = 3'b000;
    assign m_axi_arqos = 4'd0;

    // ---------------- Write side ----------------
    reg [3:0] wr_outstanding;
    reg aw_done, w_done;
    reg wr_busy;
    wire b_fire = m_axi_bvalid;
    wire wr_complete = wr_busy && (aw_done || (m_axi_awvalid && m_axi_awready)) &&
                                  (w_done  || (m_axi_wvalid  && m_axi_wready));

    assign wb_pop = wr_complete;
    assign write_idle = wb_empty && !wr_busy && (wr_outstanding == 4'd0);

    reg [31:0] m_axi_awaddr_q;
    assign m_axi_awaddr = m_axi_awaddr_q;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            wr_busy <= 1'b0;
            aw_done <= 1'b0;
            w_done <= 1'b0;
            m_axi_awvalid <= 1'b0;
            m_axi_wvalid <= 1'b0;
            m_axi_awaddr_q <= 32'b0;
            m_axi_wdata <= 64'b0;
            m_axi_wstrb <= 8'b0;
            wr_outstanding <= 4'd0;
        end else begin
            wr_outstanding <= wr_outstanding + {3'b0, wr_complete} - {3'b0, b_fire};
            if (!wr_busy) begin
                if (wb_valid && (wr_outstanding < MAX_WRITES)) begin
                    wr_busy <= 1'b1;
                    m_axi_awvalid <= 1'b1;
                    m_axi_wvalid <= 1'b1;
                    m_axi_awaddr_q <= remap(wb_addr);
                    m_axi_wdata <= {wb_data, wb_data};
                    m_axi_wstrb <= wb_addr[2] ? {wb_be, 4'b0} : {4'b0, wb_be};
                    aw_done <= 1'b0;
                    w_done <= 1'b0;
                end
            end else begin
                if (m_axi_awvalid && m_axi_awready) begin
                    m_axi_awvalid <= 1'b0;
                    aw_done <= 1'b1;
                end
                if (m_axi_wvalid && m_axi_wready) begin
                    m_axi_wvalid <= 1'b0;
                    w_done <= 1'b1;
                end
                if (wr_complete) wr_busy <= 1'b0;
            end
        end
    end

    // ---------------- Read side ----------------
    localparam C_PTW = 2'd0, C_D = 2'd1, C_I = 2'd2;
    localparam R_IDLE = 2'd0, R_ADDR = 2'd1, R_DATA = 2'd2, R_ZERO = 2'd3;
    reg [1:0] rstate;
    reg [1:0] client;
    reg word_hi;                 // which 32-bit lane a word read returns

    wire ptw_ok = ptw_req && write_idle;
    wire drf_ok = drf_req && write_idle;
    wire irf_ok = irf_req && (!irf_order || write_idle);
    wire grant_any = (rstate == R_IDLE) && (ptw_ok || drf_ok || irf_ok);
    wire [1:0] grant_client = ptw_ok ? C_PTW : drf_ok ? C_D : C_I;

    assign ptw_gnt = grant_any && (grant_client == C_PTW);
    assign drf_gnt = grant_any && (grant_client == C_D);
    assign irf_gnt = grant_any && (grant_client == C_I);

    wire r_fire = (rstate == R_DATA) && m_axi_rvalid;
    assign m_axi_rready = (rstate == R_DATA);

    assign rf_data = m_axi_rdata;
    assign drf_beat = r_fire && (client == C_D);
    assign drf_last = drf_beat && m_axi_rlast;
    assign irf_beat = r_fire && (client == C_I);
    assign irf_last = irf_beat && m_axi_rlast;
    // A page-table read outside DRAM returns zero (an invalid PTE) without touching AXI.
    assign ptw_rvalid = (r_fire && (client == C_PTW)) || (rstate == R_ZERO);
    assign ptw_rdata = (rstate == R_ZERO) ? 32'h0 :
                       (word_hi ? m_axi_rdata[63:32] : m_axi_rdata[31:0]);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rstate <= R_IDLE;
            client <= C_PTW;
            word_hi <= 1'b0;
            m_axi_arvalid <= 1'b0;
            m_axi_araddr <= 32'b0;
            m_axi_arlen <= 4'd0;
            m_axi_arsize <= 3'd0;
        end else begin
            case (rstate)
                R_IDLE: if (grant_any) begin
                    client <= grant_client;
                    if (grant_client == C_PTW) begin
                        word_hi <= ptw_addr[2];
                        if (`IS_SOC_DRAM(ptw_addr)) begin
                            m_axi_araddr <= remap({ptw_addr[31:2], 2'b00});
                            m_axi_arlen <= 4'd0;
                            m_axi_arsize <= 3'd2;
                            m_axi_arvalid <= 1'b1;
                            rstate <= R_ADDR;
                        end else begin
                            rstate <= R_ZERO;
                        end
                    end else begin
                        m_axi_araddr <= remap(grant_client == C_D ? drf_addr : irf_addr);
                        m_axi_arlen <= 4'd3;     // 4 x 8 bytes = one 32-byte line
                        m_axi_arsize <= 3'd3;
                        m_axi_arvalid <= 1'b1;
                        rstate <= R_ADDR;
                    end
                end
                R_ADDR: if (m_axi_arready) begin
                    m_axi_arvalid <= 1'b0;
                    rstate <= R_DATA;
                end
                R_DATA: if (m_axi_rvalid && m_axi_rlast) rstate <= R_IDLE;
                R_ZERO: rstate <= R_IDLE;
                default: rstate <= R_IDLE;
            endcase
        end
    end

endmodule
