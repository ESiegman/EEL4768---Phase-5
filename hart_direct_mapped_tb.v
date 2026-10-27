`timescale 1ns / 1ps
`default_nettype none

`include "hart_tb_common.vh"

// hart with direct-mapped caches: NUM_WAYS = 1, otherwise the base
// configuration (LRU, 16 KB, 128 B blocks, no prefetch), so 128 sets. Runs
// traces/random_program.hex at memory latency 4 and 1. hart_tb_common.vh has
// the checks.
//
// Waveforms: vvp build/hart_direct_mapped_sim +vcd   (writes hart_direct_mapped.vcd)
module hart_direct_mapped_tb;

    localparam N = 2;

    reg  [N-1:0] start;
    wire [N-1:0] done;

    hart_harness #(
        .LABEL("direct mapped, L=4"), .LATENCY(4),
        .EVICT_POLICY(0), .NUM_WAYS(1), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0)
    ) h0 (.i_start(start[0]), .o_done(done[0]));

    hart_harness #(
        .LABEL("direct mapped, L=1"), .LATENCY(1),
        .EVICT_POLICY(0), .NUM_WAYS(1), .CACHE_SIZE(16), .BLOCK_SIZE(128), .PREFETCH_EN(0)
    ) h1 (.i_start(start[1]), .o_done(done[1]));

    integer passed;
    integer failed;
    integer backstop;

    task summary;
        begin
            passed = h0.passed + h1.passed;
            failed = h0.failed + h1.failed + backstop;
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
            $dumpfile("hart_direct_mapped.vcd");
            $dumpvars(0, hart_direct_mapped_tb);
        end
        start    = {N{1'b0}};
        backstop = 0;
        $display("========== hart_direct_mapped testbench ==========");
        start[0] = 1'b1;
        wait (done[0]);
        start[1] = 1'b1;
        wait (done[1]);
        summary;
        $finish;
    end

    // Backstop in case a harness's own watchdogs never fire.
    initial begin
        #(64'd10 * N * 700000);
        $display("[FAIL] hart_direct_mapped: simulation time limit reached (start %b, done %b)",
                 start, done);
        backstop = 1;
        summary;
        $finish;
    end

endmodule

`default_nettype wire
