`timescale 1ns / 1ps
`default_nettype none

`include "hart_tb_common.vh"

// hart with next-line prefetch (PREFETCH_EN = 1): the base configuration
// (LRU, fully associative, 16 KB, 128 B blocks), and a small 2-way 4 KB cache
// with 64 B blocks (32 sets) that misses often enough to keep prefetches in
// flight. Each runs traces/random_program.hex at memory latency 4 and one
// other. hart_tb_common.vh has the checks.
//
// Waveforms: vvp build/hart_prefetch_sim +vcd   (writes hart_prefetch.vcd)
module hart_prefetch_tb;

    localparam N = 4;

    reg  [N-1:0] start;
    wire [N-1:0] done;

    hart_harness #(
        .LABEL("prefetch base, L=4"), .LATENCY(4),
        .EVICT_POLICY(0), .NUM_WAYS(128), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(1)
    ) h0 (.i_start(start[0]), .o_done(done[0]));

    hart_harness #(
        .LABEL("prefetch base, L=1"), .LATENCY(1),
        .EVICT_POLICY(0), .NUM_WAYS(128), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(1)
    ) h1 (.i_start(start[1]), .o_done(done[1]));

    hart_harness #(
        .LABEL("prefetch 2-way 4K/64B, L=4"), .LATENCY(4),
        .EVICT_POLICY(0), .NUM_WAYS(2), .CACHE_SIZE(4), .BLOCK_SIZE(64), .PREFETCH_EN(1)
    ) h2 (.i_start(start[2]), .o_done(done[2]));

    hart_harness #(
        .LABEL("prefetch 2-way 4K/64B, L=7"), .LATENCY(7),
        .EVICT_POLICY(0), .NUM_WAYS(2), .CACHE_SIZE(4), .BLOCK_SIZE(64), .PREFETCH_EN(1)
    ) h3 (.i_start(start[3]), .o_done(done[3]));

    integer passed;
    integer failed;
    integer backstop;

    task summary;
        begin
            passed = h0.passed + h1.passed + h2.passed + h3.passed;
            failed = h0.failed + h1.failed + h2.failed + h3.failed + backstop;
            $display("============================================");
            $display("%0d passed, %0d failed", passed, failed);
            if (failed == 0)
                $display("ALL TESTS PASSED");
            else
                $display("TEST FAILED");
        end
    endtask

    initial begin
        if ($test$plusargs("vcd")) begin
            $dumpfile("hart_prefetch.vcd");
            $dumpvars(0, hart_prefetch_tb);
        end
        start    = {N{1'b0}};
        backstop = 0;
        $display("========== hart_prefetch testbench ==========");
        start[0] = 1'b1;
        wait (done[0]);
        start[1] = 1'b1;
        wait (done[1]);
        start[2] = 1'b1;
        wait (done[2]);
        start[3] = 1'b1;
        wait (done[3]);
        summary;
        $finish;
    end

    // Backstop in case a harness's own watchdogs never fire.
    initial begin
        #(64'd10 * N * 700000);
        $display("[FAIL] hart_prefetch: simulation time limit reached (start %b, done %b)",
                 start, done);
        backstop = 1;
        summary;
        $finish;
    end

endmodule

`default_nettype wire
