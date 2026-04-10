// Testbench for i2c_master
// Compile: iverilog -g2012 -o sim i2c_master.v i2c_master_tb.v && vvp sim
// Waves:   gtkwave i2c_master_tb.vcd

`timescale 1ns/1ps

module i2c_master_tb;

    // ----------------------------------------------------------------
    // Small CLK_DIV so simulation finishes quickly
    // ----------------------------------------------------------------
    localparam CLK_DIV = 4;

    reg  clk = 0, rst_n = 0;
    reg  start = 0, rw = 0;
    reg  [6:0] addr  = 0;
    reg  [7:0] wdata = 0;
    wire [7:0] rdata;
    wire       done, ack_err, scl_oe, sda_oe;

    // Slave signals
    reg slave_sda_oe    = 0;
    reg slave_nack_addr = 0;
    reg [7:0] slave_read_byte = 8'hA5;
    reg [7:0] slave_rx        = 0;

    // Wired-AND open-drain bus
    wire SCL = scl_oe      ? 1'b0 : 1'b1;
    wire SDA = (sda_oe | slave_sda_oe) ? 1'b0 : 1'b1;

    // ----------------------------------------------------------------
    // DUT
    // ----------------------------------------------------------------
    i2c_master #(.CLK_DIV(CLK_DIV)) dut (
        .clk    (clk),
        .rst_n  (rst_n),
        .start  (start),
        .rw     (rw),
        .addr   (addr),
        .wdata  (wdata),
        .rdata  (rdata),
        .done   (done),
        .ack_err(ack_err),
        .scl_oe (scl_oe),
        .sda_oe (sda_oe),
        .sda_in (SDA)
    );

    always #5 clk = ~clk;   // 100 MHz

    // ----------------------------------------------------------------
    // Behavioural I2C slave (process-based, open-drain aware)
    // Detects START by watching SDA negedge while SCL is high.
    // ----------------------------------------------------------------
    reg [7:0] sl_sh;
    reg       sl_rw;
    integer   sl_i;

    initial begin
        forever begin
            // ---- Wait for START condition (SDA falls while SCL high) ----
            @(negedge SDA);
            if (SCL !== 1'b1) disable fork; // not a START; loop again
            else begin
                // Receive 8 bits: 7-bit address + R/W, MSB first
                sl_sh = 8'h00;
                repeat (8) begin
                    @(posedge SCL);
                    sl_sh = {sl_sh[6:0], SDA};
                end
                sl_rw = sl_sh[0];   // LSB of the received byte is the R/W bit

                // Drive ACK (pull SDA low) starting on next SCL falling edge
                @(negedge SCL);
                slave_sda_oe = slave_nack_addr ? 1'b0 : 1'b1;

                // Release ACK on the following SCL falling edge
                @(negedge SCL);
                slave_sda_oe = 1'b0;

                if (!slave_nack_addr) begin
                    if (!sl_rw) begin
                        // ---- WRITE: receive 8 data bits then ACK ----
                        slave_rx = 8'h00;
                        repeat (8) begin
                            @(posedge SCL);
                            slave_rx = {slave_rx[6:0], SDA};
                        end
                        @(negedge SCL); slave_sda_oe = 1'b1;  // ACK data
                        @(negedge SCL); slave_sda_oe = 1'b0;  // release

                    end else begin
                        // ---- READ: drive 8 data bits (MSB first) ----
                        for (sl_i = 7; sl_i >= 0; sl_i = sl_i - 1) begin
                            slave_sda_oe = ~slave_read_byte[sl_i];
                            @(posedge SCL);   // master samples
                            @(negedge SCL);   // SCL falls; next bit set at loop top
                        end
                        slave_sda_oe = 1'b0;  // release for master NACK
                        @(posedge SCL);        // NACK bit (master releases SDA)
                        @(negedge SCL);        // end of NACK slot
                    end
                end
                // Transaction done — loop back and wait for next START
            end
        end
    end

    // ----------------------------------------------------------------
    // Test helpers
    // ----------------------------------------------------------------
    integer pass_cnt = 0;
    integer fail_cnt = 0;

    task check_cond;
        input       cond;
        input [8*32-1:0] msg;
        begin
            if (cond) begin
                $display("  PASS: %0s", msg);
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  FAIL: %0s", msg);
                fail_cnt = fail_cnt + 1;
            end
        end
    endtask

    task do_write;
        input [6:0] a;
        input [7:0] d;
        begin
            @(posedge clk); #1;
            addr = a; wdata = d; rw = 1'b0; start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0;
            @(posedge done);
            @(posedge clk); #1;   // let signals settle
        end
    endtask

    task do_read;
        input [6:0] a;
        begin
            @(posedge clk); #1;
            addr = a; rw = 1'b1; start = 1'b1;
            @(posedge clk); #1;
            start = 1'b0;
            @(posedge done);
            @(posedge clk); #1;
        end
    endtask

    task gap; repeat(20) @(posedge clk); endtask

    // ----------------------------------------------------------------
    // Stimulus
    // ----------------------------------------------------------------
    initial begin
        $dumpfile("i2c_master_tb.vcd");
        $dumpvars(0, i2c_master_tb);

        // Reset
        rst_n = 1'b0;
        repeat(4) @(posedge clk);
        rst_n = 1'b1;
        repeat(4) @(posedge clk);

        // ---- Test 1: Write 0xBE to address 0x4A ----
        $display("T1: Write 0xBE -> addr 0x4A");
        do_write(7'h4A, 8'hBE);
        check_cond(!ack_err,             "no ack_err");
        check_cond(slave_rx === 8'hBE,   "slave_rx == 0xBE");
        gap;

        // ---- Test 2: Read from address 0x4A (slave returns 0xA5) ----
        $display("T2: Read from addr 0x4A (expect 0xA5)");
        do_read(7'h4A);
        check_cond(!ack_err,             "no ack_err");
        check_cond(rdata === 8'hA5,      "rdata == 0xA5");
        gap;

        // ---- Test 3: NACK on address ----
        $display("T3: Address NACK (addr 0x7F)");
        slave_nack_addr = 1'b1;
        do_write(7'h7F, 8'hFF);
        check_cond(ack_err,              "ack_err set on NACK");
        slave_nack_addr = 1'b0;
        gap;

        // ---- Test 4: Write a second data byte ----
        $display("T4: Write 0x3C -> addr 0x4A");
        do_write(7'h4A, 8'h3C);
        check_cond(!ack_err,             "no ack_err");
        check_cond(slave_rx === 8'h3C,   "slave_rx == 0x3C");
        gap;

        // ---- Test 5: Read different slave data ----
        $display("T5: Read from addr 0x4A (expect 0xD4)");
        slave_read_byte = 8'hD4;
        do_read(7'h4A);
        check_cond(!ack_err,             "no ack_err");
        check_cond(rdata === 8'hD4,      "rdata == 0xD4");
        gap;

        // ---- Test 6: SCL stays high (bus idle) between transactions ----
        $display("T6: SCL idle check");
        check_cond(SCL === 1'b1,         "SCL high in idle");
        check_cond(SDA === 1'b1,         "SDA high in idle");

        $display("");
        $display("=== %0d passed, %0d failed ===", pass_cnt, fail_cnt);
        if (fail_cnt == 0) $display("ALL TESTS PASSED");
        else                $display("SOME TESTS FAILED");
        $finish;
    end

    // Watchdog
    initial begin
        #5_000_000;
        $display("TIMEOUT — simulation stalled");
        $finish;
    end

endmodule
