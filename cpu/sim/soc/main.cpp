// Fast standalone simulator for soc_sim_top (no cocotb).
//   ./obj/Vsoc_sim_top +hex=<image> [+tohost=<hex>] [+max_cycles=N] [+trace]
// Exit code: 0 = tohost reported 1 (pass), 1 = fail code, 2 = timeout.
#include "Vsoc_sim_top.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    unsigned long long max_cycles = 2000000;
    const char *arg = Verilated::commandArgsPlusMatch("max_cycles=");
    if (arg && *arg) max_cycles = strtoull(strchr(arg, '=') + 1, nullptr, 10);

    Vsoc_sim_top *top = new Vsoc_sim_top;
    top->uart_rx = 1;
    top->rst = 1;
    top->clk = 0;
    unsigned long long cycle = 0;
    for (; cycle < max_cycles && !Verilated::gotFinish(); ++cycle) {
        if (cycle == 5) top->rst = 0;
        top->clk = 1; top->eval();
        top->clk = 0; top->eval();
        if (top->done) { for (int k = 0; k < 200; ++k) { top->clk = 1; top->eval(); top->clk = 0; top->eval(); } break; }
    }
    int rc;
    if (top->done) {
        unsigned r = top->result;
        rc = (r == 1) ? 0 : 1;
        fprintf(stderr, "\n[soc_sim] tohost=0x%x after %llu cycles: %s\n", r, cycle,
                rc ? "FAIL" : "PASS");
    } else {
        fprintf(stderr, "\n[soc_sim] TIMEOUT after %llu cycles\n", cycle);
        rc = 2;
    }
    top->final();
    delete top;
    return rc;
}
