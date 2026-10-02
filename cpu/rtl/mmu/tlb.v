`default_nettype none
// Fully associative Sv32 TLB. Holds leaf PTEs filled by the page walker; permission
// and A/D (Svade) checks run on the returned PTE bits. Not ASID-tagged: satp.ASID is
// read-only zero, so SFENCE.VMA (any form) flushes everything.
module tlb #(
    parameter ENTRIES = 8
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        flush,
    // Lookup (combinational)
    input  wire [19:0] lookup_vpn,
    output wire        hit,
    output wire [19:0] ppn,        // physical page number for lookup_vpn
    output wire [7:0]  pte_flags,  // D A G U X W R V
    // Fill from the page walker
    input  wire        fill,
    input  wire [19:0] fill_vpn,
    input  wire [31:0] fill_pte,
    input  wire        fill_mega   // 4 MB superpage (leaf at level 1)
);

    localparam IDX_W = (ENTRIES <= 2) ? 1 : (ENTRIES <= 4) ? 2 : (ENTRIES <= 8) ? 3 :
                       (ENTRIES <= 16) ? 4 : 5;

    localparam [IDX_W-1:0] LAST = ENTRIES - 1;
    reg [ENTRIES-1:0] valid;
    reg [ENTRIES-1:0] mega;
    reg [19:0] vpn   [0:ENTRIES-1];
    reg [19:0] pte_ppn [0:ENTRIES-1];
    reg [7:0]  flags [0:ENTRIES-1];
    reg [IDX_W-1:0] victim;

    // Match: superpages compare only VPN[1].
    reg [ENTRIES-1:0] match;
    reg [19:0] ppn_r;
    reg [7:0] flags_r;
    integer i;
    always @(*) begin
        ppn_r = 20'b0;
        flags_r = 8'b0;
        for (i = 0; i < ENTRIES; i = i + 1) begin
            match[i] = valid[i] && (vpn[i][19:10] == lookup_vpn[19:10]) &&
                       (mega[i] || (vpn[i][9:0] == lookup_vpn[9:0]));
            if (match[i]) begin
                ppn_r = mega[i] ? {pte_ppn[i][19:10], lookup_vpn[9:0]} : pte_ppn[i];
                flags_r = flags[i];
            end
        end
    end

    assign hit = |match;
    assign ppn = ppn_r;
    assign pte_flags = flags_r;

    always @(posedge clk) begin
        if (rst || flush) begin
            valid <= {ENTRIES{1'b0}};
            victim <= {IDX_W{1'b0}};
        end else if (fill) begin
            valid[victim] <= 1'b1;
            mega[victim] <= fill_mega;
            vpn[victim] <= fill_vpn;
            pte_ppn[victim] <= fill_pte[29:10];
            flags[victim] <= fill_pte[7:0];
            victim <= (victim == LAST) ? {IDX_W{1'b0}} : victim + 1'b1;
        end
    end

endmodule

// Restore the default so this file's setting cannot leak into the next one compiled.
`default_nettype wire
