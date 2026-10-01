`default_nettype none
// Simulation wrapper: soc_top (core released at power-up) + behavioral DDR.
// Watches stores for the riscv-tests `tohost` protocol: +tohost=<hex address>.
module soc_sim_top #(
    parameter READ_LATENCY = 12,
    parameter STALL_EVERY = 0,
    parameter DDR_WORDS = 17039360
) (
    input  wire clk,
    input  wire rst,
    input  wire uart_rx,
    output wire uart_tx,
    output reg  done,
    output reg  [31:0] result
);
    wire rst_n = !rst;

    wire [31:0] awaddr, araddr;
    wire [3:0] awlen, arlen;
    wire [2:0] awsize, arsize;
    wire awvalid, awready, wvalid, wready, wlast, bvalid, bready;
    wire arvalid, arready, rvalid, rready, rlast;
    wire [63:0] wdata, rdata;
    wire [7:0] wstrb;

    soc_top #(.CORE_RESET_HELD(0)) soc (
        .clk(clk), .rst_n(rst_n),
        .uart_tx(uart_tx), .uart_rx(uart_rx),
        .m_axi_awid(), .m_axi_awaddr(awaddr), .m_axi_awlen(awlen), .m_axi_awsize(awsize),
        .m_axi_awburst(), .m_axi_awlock(), .m_axi_awcache(), .m_axi_awprot(), .m_axi_awqos(),
        .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wid(), .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wlast(wlast),
        .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bid(6'd0), .m_axi_bresp(2'b00), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .m_axi_arid(), .m_axi_araddr(araddr), .m_axi_arlen(arlen), .m_axi_arsize(arsize),
        .m_axi_arburst(), .m_axi_arlock(), .m_axi_arcache(), .m_axi_arprot(), .m_axi_arqos(),
        .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rid(6'd0), .m_axi_rdata(rdata), .m_axi_rresp(2'b00), .m_axi_rlast(rlast),
        .m_axi_rvalid(rvalid), .m_axi_rready(rready),
        .s_axi_awid(12'd0), .s_axi_awaddr(32'd0), .s_axi_awlen(4'd0), .s_axi_awsize(3'd0),
        .s_axi_awburst(2'd0), .s_axi_awvalid(1'b0), .s_axi_awready(),
        .s_axi_wid(12'd0), .s_axi_wdata(32'd0), .s_axi_wstrb(4'd0), .s_axi_wlast(1'b0),
        .s_axi_wvalid(1'b0), .s_axi_wready(),
        .s_axi_bid(), .s_axi_bresp(), .s_axi_bvalid(), .s_axi_bready(1'b1),
        .s_axi_arid(12'd0), .s_axi_araddr(32'd0), .s_axi_arlen(4'd0), .s_axi_arsize(3'd0),
        .s_axi_arburst(2'd0), .s_axi_arvalid(1'b0), .s_axi_arready(),
        .s_axi_rid(), .s_axi_rdata(), .s_axi_rresp(), .s_axi_rlast(), .s_axi_rvalid(),
        .s_axi_rready(1'b1)
    );

    axi_ddr_model #(.READ_LATENCY(READ_LATENCY), .STALL_EVERY(STALL_EVERY), .MEM_WORDS(DDR_WORDS)) ddr (
        .clk(clk), .rst(rst),
        .awaddr(awaddr), .awlen(awlen), .awsize(awsize), .awvalid(awvalid), .awready(awready),
        .wdata(wdata), .wstrb(wstrb), .wlast(wlast), .wvalid(wvalid), .wready(wready),
        .bvalid(bvalid), .bready(bready),
        .araddr(araddr), .arlen(arlen), .arsize(arsize), .arvalid(arvalid), .arready(arready),
        .rdata(rdata), .rlast(rlast), .rvalid(rvalid), .rready(rready)
    );

    // +signature=<file> +sig_begin=<hex> +sig_end=<hex>: dump [begin, end) as 32-bit hex words
    // (Spike's --signature-granularity=4 format) once the test is done and stores have drained.
    reg [1023:0] sig_path;
    reg [31:0] sig_begin, sig_end;
    reg sig_on, sig_done;
    integer sig_fd, sig_i;
    initial begin
        sig_on = $value$plusargs("signature=%s", sig_path);
        if (!$value$plusargs("sig_begin=%h", sig_begin)) sig_begin = 0;
        if (!$value$plusargs("sig_end=%h", sig_end)) sig_end = 0;
        sig_done = 0;
    end
    always @(posedge clk) begin
        if (sig_on && done && !sig_done && soc.write_idle && !soc.vec_busy) begin
            sig_done <= 1'b1;
            sig_fd = $fopen(sig_path, "w");
            for (sig_i = sig_begin; sig_i < sig_end; sig_i = sig_i + 4)
                $fwrite(sig_fd, "%08x\n", ddr.mem[ddr.word_index(sig_i - 32'h80000000 + 32'h20000000)]);
            $fclose(sig_fd);
        end
    end
    wire finished = done && (!sig_on || sig_done);

    // UART transmit echo: bytes written to THR (DLAB clear) go to stdout.
    always @(posedge clk) begin
        if (!rst && soc.io_wr && soc.uart_sel && soc.io_addr == 32'h20000000 && !soc.uart_inst.lcr[7]) begin
            $write("%c", soc.io_wdata[7:0]);
            $fflush;
        end
    end

    // +uart_line: decode the serial TX pin itself (115200 8N1 at 50 MHz = 432 clocks/bit)
    // and print it, prefixed, to check the real line timing.
    reg uart_line_on;
    initial uart_line_on = $test$plusargs("uart_line");
    reg [15:0] ul_cnt;
    reg [3:0]  ul_bit;
    reg [7:0]  ul_byte;
    reg        ul_busy;
    always @(posedge clk) begin
        if (rst) begin
            ul_busy <= 1'b0;
        end else if (uart_line_on) begin
            if (!ul_busy) begin
                if (!uart_tx) begin ul_busy <= 1'b1; ul_cnt <= 16'd216; ul_bit <= 4'd0; end
            end else if (ul_cnt != 0) begin
                ul_cnt <= ul_cnt - 16'd1;
            end else begin
                ul_cnt <= 16'd431;
                if (ul_bit == 4'd0) begin
                    if (uart_tx) ul_busy <= 1'b0;          // false start
                    ul_bit <= 4'd1;
                end else if (ul_bit <= 4'd8) begin
                    ul_byte <= {uart_tx, ul_byte[7:1]};
                    ul_bit <= ul_bit + 4'd1;
                end else begin
                    if (!uart_tx) $display("[uart-line] framing error");
                    else $write("[%c]", ul_byte);
                    ul_busy <= 1'b0;
                end
            end
        end
    end

    // +fptrace: coprocessor FP activity
    reg fptrace_on;
    initial fptrace_on = $test$plusargs("fptrace");
    always @(posedge clk) begin
        if (fptrace_on && !rst) begin
            if (soc.vpu.state != 0)
                $display("t=%0t vpu st=%0d kind=%0d instr=%h fflags_set=%b fp_flags=%b sf1=%h sf2=%h fp_res=%h",
                         $time, soc.vpu.state, soc.vpu.c_kind, soc.vpu.c_instr, soc.vpu.fflags_set,
                         soc.vpu.fp_flags, soc.vpu.sf1, soc.vpu.sf2, soc.vpu.fp_res);
            if (soc.cpu_inst.id_ex_inst0_instr_out == 32'h001015f3)
                $display("t=%0t fsflags in EX: fflags=%b vcsr_stall=%b busy=%b memwait=%b rd_data=%h", $time,
                         soc.cpu_inst.csr_file_inst.fflags, soc.cpu_inst.vcsr_stall, soc.vec_busy,
                         soc.cpu_inst.mem_wait, soc.cpu_inst.csr_read_data);
            if (soc.cpu_inst.csr_file_inst.fflags_set != 0)
                $display("t=%0t csr fflags_set=%b fflags=%b", $time, soc.cpu_inst.csr_file_inst.fflags_set,
                         soc.cpu_inst.csr_file_inst.fflags);
        end
    end

    reg [31:0] tohost;
    initial begin
        if (!$value$plusargs("tohost=%h", tohost)) tohost = 32'h80001000;
    end

    always @(posedge clk) begin
        if (rst) begin
            done <= 1'b0;
            result <= 32'b0;
        end else if (soc.d_wr_en && soc.d_rvalid && soc.d_paddr == tohost &&
                     soc.d_wdata[0] && !done) begin
            done <= 1'b1;
            result <= soc.d_wdata;
        end
    end

    // +trace: per-cycle bring-up log of fetch, data port and memory traffic.
    reg trace_on;
    initial trace_on = $test$plusargs("trace");
    always @(posedge clk) begin
        if (trace_on && !rst)
            $display("t=%0t pc=%h ird=%b ist=%0d | drd=%b dwr=%b va=%h dval=%b dst=%0d wb=%0d widle=%b | rs=%0d cl=%0d ar=%b r=%b | stall=%b memwait=%b",
                $time, soc.pc, soc.instr_rvalid, soc.icache_inst.state,
                soc.d_rd_en, soc.d_wr_en, soc.d_vaddr, soc.d_rvalid, soc.dcache_inst.state,
                soc.dcache_inst.sb_count, soc.write_idle,
                soc.mem_master.rstate, soc.mem_master.client, soc.m_axi_arvalid, soc.m_axi_rvalid,
                soc.cpu_inst.pipeline_stall, soc.cpu_inst.mem_wait);
    end

endmodule
