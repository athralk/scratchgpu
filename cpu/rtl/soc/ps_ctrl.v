`default_nettype none
// Control registers the Zynq ARM reaches through M_AXI_GP0 (AXI3 slave, 32-bit).
// Synapse starts held in reset; the ARM loads its image into DDR and then releases it.
//
//   0x00 CTRL     [0] core reset (1 = held, reset value 1)
//   0x04 PC       current fetch PC (read-only, for bring-up)
//   0x08 ID       0x53594E35 "SYN5"
//   0x0C SCRATCH  read/write
//   0x10 DDRBASE  DDR address that Synapse physical 0x8000_0000 maps to (read-only)
module ps_ctrl #(
    parameter [31:0] DDR_BASE = 32'h20000000,
    parameter RESET_HELD = 1           // simulation releases the core at power-up
) (
    input  wire        clk,
    input  wire        rst,
    output wire        core_reset,
    input  wire [31:0] pc_debug,

    input  wire [11:0] s_axi_awid,
    input  wire [31:0] s_axi_awaddr,
    input  wire [3:0]  s_axi_awlen,
    input  wire        s_axi_awvalid,
    output wire        s_axi_awready,
    input  wire [11:0] s_axi_wid,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wlast,
    input  wire        s_axi_wvalid,
    output wire        s_axi_wready,
    output reg  [11:0] s_axi_bid,
    output wire [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [11:0] s_axi_arid,
    input  wire [31:0] s_axi_araddr,
    input  wire [3:0]  s_axi_arlen,
    input  wire        s_axi_arvalid,
    output wire        s_axi_arready,
    output reg  [11:0] s_axi_rid,
    output reg  [31:0] s_axi_rdata,
    output wire [1:0]  s_axi_rresp,
    output reg         s_axi_rlast,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready
);

    reg ctrl_reset;
    reg [31:0] scratch;
    assign core_reset = ctrl_reset;
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    function [31:0] read_reg;
        input [7:0] off;
        begin
            case (off[4:2])
                3'd0: read_reg = {31'b0, ctrl_reset};
                3'd1: read_reg = pc_debug;
                3'd2: read_reg = 32'h53594E35;
                3'd3: read_reg = scratch;
                3'd4: read_reg = DDR_BASE;
                default: read_reg = 32'h0;
            endcase
        end
    endfunction

    // ---------------- Write: take AW, then each W beat (bursts write successive registers) ----
    reg aw_active;
    reg [7:0] aw_off;
    assign s_axi_awready = !aw_active && !s_axi_bvalid;
    assign s_axi_wready = aw_active;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            aw_active <= 1'b0;
            aw_off <= 8'b0;
            s_axi_bid <= 12'b0;
            s_axi_bvalid <= 1'b0;
            ctrl_reset <= RESET_HELD[0];
            scratch <= 32'b0;
        end else begin
            if (s_axi_bvalid && s_axi_bready) s_axi_bvalid <= 1'b0;
            if (s_axi_awvalid && s_axi_awready) begin
                aw_active <= 1'b1;
                aw_off <= s_axi_awaddr[7:0];
                s_axi_bid <= s_axi_awid;
            end
            if (s_axi_wvalid && s_axi_wready) begin
                if (s_axi_wstrb[0]) begin
                    case (aw_off[4:2])
                        3'd0: ctrl_reset <= s_axi_wdata[0];
                        3'd3: scratch <= s_axi_wdata;
                        default: ;
                    endcase
                end
                aw_off <= aw_off + 8'd4;
                if (s_axi_wlast) begin
                    aw_active <= 1'b0;
                    s_axi_bvalid <= 1'b1;
                end
            end
        end
    end

    // ---------------- Read: one beat per cycle for len+1 beats ----------------
    reg ar_active;
    reg [7:0] ar_off;
    reg [3:0] ar_left;
    assign s_axi_arready = !ar_active;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            ar_active <= 1'b0;
            ar_off <= 8'b0;
            ar_left <= 4'b0;
            s_axi_rid <= 12'b0;
            s_axi_rdata <= 32'b0;
            s_axi_rlast <= 1'b0;
            s_axi_rvalid <= 1'b0;
        end else begin
            if (s_axi_arvalid && s_axi_arready) begin
                ar_active <= 1'b1;
                ar_off <= s_axi_araddr[7:0] + 8'd4;
                ar_left <= s_axi_arlen;
                s_axi_rid <= s_axi_arid;
                s_axi_rdata <= read_reg(s_axi_araddr[7:0]);
                s_axi_rlast <= (s_axi_arlen == 4'd0);
                s_axi_rvalid <= 1'b1;
            end else if (s_axi_rvalid && s_axi_rready) begin
                if (ar_left == 4'd0) begin
                    s_axi_rvalid <= 1'b0;
                    ar_active <= 1'b0;
                end else begin
                    ar_left <= ar_left - 4'd1;
                    ar_off <= ar_off + 8'd4;
                    s_axi_rdata <= read_reg(ar_off);
                    s_axi_rlast <= (ar_left == 4'd1);
                end
            end
        end
    end

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
