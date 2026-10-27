`timescale 1ns / 1ps
`default_nettype none

`include "hart_tb_common.vh"

// hart, base configuration (phase_5.pdf 4.11): LRU, fully associative
// (NUM_WAYS = 128), 16 KB, 128 B blocks, no prefetch. Runs
// traces/random_program.hex at the base memory latency of 4 cycles, then at
// 1 and 7. hart_tb_common.vh has the checks.
//
// Waveforms: vvp build/hart_base_sim +vcd   (writes hart_base.vcd)
module hart_base_tb;

    localparam N = 3;

    reg  [N-1:0] start;
    wire [N-1:0] done;

    hart_harness #(
        .LABEL("base, L=4"), .LATENCY(4),
        .EVICT_POLICY(0), .NUM_WAYS(128), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0)
    ) h0 (.i_start(start[0]), .o_done(done[0]));

    hart_harness #(
        .LABEL("base, L=1"), .LATENCY(1),
        .EVICT_POLICY(0), .NUM_WAYS(128), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0)
    ) h1 (.i_start(start[1]), .o_done(done[1]));

    hart_harness #(
        .LABEL("base, L=7"), .LATENCY(7),
        .EVICT_POLICY(0), .NUM_WAYS(128), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0)
    ) h2 (.i_start(start[2]), .o_done(done[2]));

    integer passed;
    integer failed;
    integer backstop;

    task summary;
        begin
            passed = h0.passed + h1.passed + h2.passed;
            failed = h0.failed + h1.failed + h2.failed + backstop;
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
            $dumpfile("hart_base.vcd");
            $dumpvars(0, hart_base_tb);
        end
        start    = {N{1'b0}};
        backstop = 0;
        $display("========== hart_base testbench ==========");
        start[0] = 1'b1;
        wait (done[0]);
        start[1] = 1'b1;
        wait (done[1]);
        start[2] = 1'b1;
        wait (done[2]);
        summary;
        $finish;
    end

    // Backstop in case a harness's own watchdogs never fire.
    initial begin
        #(64'd10 * N * 700000);
        $display("[FAIL] hart_base: simulation time limit reached (start %b, done %b)",
                 start, done);
        backstop = 1;
        summary;
        $finish;
    end

endmodule

`default_nettype wire
