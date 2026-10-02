`default_nettype none
`include "memory_map.vh"
// Sequential Sv32 page-table walker shared by the I- and D-TLBs.
//
// Reads PTEs from memory through the arbiter (which orders the read behind every
// buffered store, so page-table writes are always visible). Svade: no A/D updates;
// those faults come from the permission checks on the filled entry. A walk that
// finds no valid leaf reports a one-cycle fault for that VPN and fills nothing.
module ptw (
    input  wire        clk,
    input  wire        rst,
    input  wire        flush,          // SFENCE.VMA: drop the result of a walk in flight
    input  wire [31:0] satp,

    // Requests (level: held until fill or fault for that VPN)
    input  wire        d_req,
    input  wire [19:0] d_vpn,
    input  wire        i_req,
    input  wire [19:0] i_vpn,

    // Results (one cycle)
    output reg         fill_d,
    output reg         fill_i,
    output reg         fault_d,
    output reg         fault_i,
    output reg  [19:0] res_vpn,
    output reg  [31:0] res_pte,
    output reg         res_mega,

    // Memory read port (one 32-bit word)
    output reg         mem_req,
    output reg  [31:0] mem_addr,
    input  wire        mem_gnt,
    input  wire        mem_rvalid,
    input  wire [31:0] mem_rdata
);

    localparam S_IDLE = 2'd0, S_L1 = 2'd1, S_L0 = 2'd2;

    reg [1:0] state;
    reg for_d;
    reg [19:0] vpn_q;
    reg waiting;      // request granted, data pending
    reg discard;

    // PTE checks (privileged spec 4.3.2). PTE[31:30] would make a >4 GB address: not modelled.
    wire pte_v = mem_rdata[0];
    wire pte_r = mem_rdata[1];
    wire pte_w = mem_rdata[2];
    wire pte_x = mem_rdata[3];
    wire pte_bad = !pte_v || (!pte_r && pte_w) || (mem_rdata[31:30] != 2'b00);
    wire pte_leaf = pte_r || pte_x;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= S_IDLE;
            for_d <= 1'b0;
            vpn_q <= 20'b0;
            waiting <= 1'b0;
            discard <= 1'b0;
            mem_req <= 1'b0;
            mem_addr <= 32'b0;
            fill_d <= 1'b0;
            fill_i <= 1'b0;
            fault_d <= 1'b0;
            fault_i <= 1'b0;
            res_vpn <= 20'b0;
            res_pte <= 32'b0;
            res_mega <= 1'b0;
        end else begin
            fill_d <= 1'b0;
            fill_i <= 1'b0;
            fault_d <= 1'b0;
            fault_i <= 1'b0;
            if (flush && state != S_IDLE) discard <= 1'b1;

            if (mem_req && mem_gnt) begin
                mem_req <= 1'b0;
                waiting <= 1'b1;
            end

            case (state)
                S_IDLE: begin
                    discard <= 1'b0;
                    // Data side first: MEM is older than IF.
                    if (d_req || i_req) begin
                        for_d <= d_req;
                        vpn_q <= d_req ? d_vpn : i_vpn;
                        mem_req <= 1'b1;
                        mem_addr <= {satp[19:0], 12'b0} + {20'b0, (d_req ? d_vpn[19:10] : i_vpn[19:10]), 2'b00};
                        state <= S_L1;
                    end
                end
                S_L1, S_L0: if (waiting && mem_rvalid) begin
                    waiting <= 1'b0;
                    res_vpn <= vpn_q;
                    res_pte <= mem_rdata;
                    res_mega <= (state == S_L1);
                    if (pte_bad || (state == S_L0 && !pte_leaf) ||
                        (state == S_L1 && pte_leaf && mem_rdata[19:10] != 10'b0)) begin
                        // Invalid, non-leaf at level 0, or misaligned superpage.
                        fault_d <= for_d && !discard && !flush;
                        fault_i <= !for_d && !discard && !flush;
                        state <= S_IDLE;
                    end else if (pte_leaf) begin
                        fill_d <= for_d && !discard && !flush;
                        fill_i <= !for_d && !discard && !flush;
                        state <= S_IDLE;
                    end else begin
                        mem_req <= 1'b1;
                        mem_addr <= {mem_rdata[29:10], 12'b0} + {20'b0, vpn_q[9:0], 2'b00};
                        state <= S_L0;
                    end
                end
                default: state <= S_IDLE;
            endcase
        end
    end

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
