`timescale 1ns / 1ps
`default_nettype none

// Override NUM_WAYS to exercise direct-mapped and non-power-of-two trees.
module plru_tb;
    parameter NUM_WAYS = 4;
    localparam WAY_BITS = (NUM_WAYS > 1) ? $clog2(NUM_WAYS) : 1;
    reg clk = 1'b0;
    always #5 clk = ~clk;
    reg rst;
    reg [NUM_WAYS-1:0] valid;
    reg touch;
    reg [WAY_BITS-1:0] touch_way;
    wire [WAY_BITS-1:0] victim;
    integer last_used [0:NUM_WAYS-1];
    integer stamp;
    integer passed;
    integer failed;
    integer seed;
    integer iteration;
    integer way;
    integer chosen;

    plru #(.NUM_WAYS(NUM_WAYS)) dut (
        .i_clk(clk), .i_rst(rst), .i_valid(valid),
        .i_touch(touch), .i_touch_way(touch_way), .o_victim_way(victim)
    );

    // Independent model: the most recent timestamp in each half tells which
    // subtree was used last. Follow the older half, ignoring padding leaves.
    function integer expected_victim;
        input unused;
        integer first;
        integer span;
        integer middle;
        integer left_time;
        integer right_time;
        integer j;
        integer invalid_way;
        begin
            invalid_way = -1;
            for (j = 0; j < NUM_WAYS; j = j + 1)
                if (!valid[j] && invalid_way == -1)
                    invalid_way = j;
            first = 0;
            span = 2 ** $clog2(NUM_WAYS);
            while (span > 1) begin
                middle = first + span/2;
                left_time = -1;
                right_time = -1;
                for (j = 0; j < NUM_WAYS; j = j + 1) begin
                    if (j >= first && j < middle && last_used[j] > left_time)
                        left_time = last_used[j];
                    if (j >= middle && j < first+span && last_used[j] > right_time)
                        right_time = last_used[j];
                end
                if (right_time != -1 && left_time > right_time)
                    first = middle;
                span = span/2;
            end
            expected_victim = (invalid_way == -1) ? first : invalid_way;
        end
    endfunction

    task check;
        input [511:0] label;
        integer expected;
        begin
            #1;
            expected = expected_victim(1'b0);
            if (victim === expected[WAY_BITS-1:0] && victim < NUM_WAYS)
                passed = passed + 1;
            else begin
                failed = failed + 1;
                $display("[FAIL] %0s: victim=%0d expected=%0d", label, victim, expected);
            end
        end
    endtask

    task reset_tree;
        begin
            @(negedge clk);
            rst = 1'b1;
            touch = 1'b0;
            @(posedge clk);
            stamp = 0;
            for (way = 0; way < NUM_WAYS; way = way + 1)
                last_used[way] = 0;
            check("reset");
            @(negedge clk);
            rst = 1'b0;
        end
    endtask

    initial begin
        rst = 1'b1;
        touch = 1'b0;
        touch_way = 0;
        valid = {NUM_WAYS{1'b1}};
        passed = 0;
        failed = 0;
        seed = 32'd71027;
        reset_tree();

        // Every invalid position must beat a valid victim, lowest first.
        for (chosen = NUM_WAYS-1; chosen >= 0; chosen = chosen - 1) begin
            valid[chosen] = 1'b0;
            check("lowest invalid way");
        end
        valid = {NUM_WAYS{1'b1}};

        for (iteration = 0; iteration < 1000; iteration = iteration + 1) begin
            @(negedge clk);
            touch = (($random(seed) & 3) != 0);
            touch_way = $random(seed);
            // Mostly all valid so tree selection gets substantial coverage.
            for (way = 0; way < NUM_WAYS; way = way + 1)
                valid[way] = (($random(seed) & 31) != 0);
            @(posedge clk);
            if (touch && touch_way < NUM_WAYS) begin
                stamp = stamp + 1;
                last_used[touch_way] = stamp;
            end
            check("random access / no access / out-of-range way");
            valid = {NUM_WAYS{1'b1}};
            check("tree with all ways valid");
        end
        reset_tree();
        check("reset clears access history");
        $display("PLRU NUM_WAYS=%0d", NUM_WAYS);
        $display("%0d passed, %0d failed", passed, failed);
        if (failed == 0)
            $display("ALL TESTS PASSED");
        else
            $display("TEST FAILED");
        $finish;
    end
endmodule

`default_nettype wire
