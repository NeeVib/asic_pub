// I2C Master Controller
// Supports 7-bit addressing, single-byte read/write
// SCL frequency = clk / (4 * CLK_DIV)

module i2c_master #(
    parameter CLK_DIV = 125  // For 100kHz SCL with 50MHz clk: 50e6/(4*125) = 100kHz
) (
    input  wire       clk,
    input  wire       rst_n,

    // User interface
    input  wire       start,       // Pulse to begin transaction
    input  wire       rw,          // 0 = write, 1 = read
    input  wire [6:0] addr,        // 7-bit slave address
    input  wire [7:0] wdata,       // Byte to write
    output reg  [7:0] rdata,       // Byte read from slave
    output reg        done,        // Pulses high when transaction complete
    output reg        ack_err,     // High if NACK received

    // I2C bus (open-drain — connect to tristate buffers externally)
    output reg        scl_oe,      // 1 = drive SCL low
    output reg        sda_oe,      // 1 = drive SDA low
    input  wire       sda_in       // SDA sampled from bus
);

    // ----------------------------------------------------------------
    // Clock divider — generates 4 ticks per SCL period
    // ----------------------------------------------------------------
    localparam DIV_W = $clog2(CLK_DIV);

    reg [DIV_W-1:0] clk_cnt;
    reg             tick;          // 1 clk pulse every CLK_DIV cycles

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            clk_cnt <= 0;
            tick    <= 1'b0;
        end else begin
            tick <= 1'b0;
            if (clk_cnt == CLK_DIV - 1) begin
                clk_cnt <= 0;
                tick    <= 1'b1;
            end else begin
                clk_cnt <= clk_cnt + 1;
            end
        end
    end

    // ----------------------------------------------------------------
    // SCL quarter-period counter (0-3 per SCL cycle)
    // ----------------------------------------------------------------
    reg [1:0] qtr;  // quarter-period phase: 0=low, 1=rising, 2=high, 3=falling

    // ----------------------------------------------------------------
    // FSM states
    // ----------------------------------------------------------------
    localparam [3:0]
        S_IDLE    = 4'd0,
        S_START   = 4'd1,
        S_ADDR    = 4'd2,
        S_ADDR_ACK= 4'd3,
        S_WRITE   = 4'd4,
        S_WACK    = 4'd5,
        S_READ    = 4'd6,
        S_RACK    = 4'd7,
        S_STOP    = 4'd8;

    reg [3:0] state;
    reg [3:0] bit_cnt;   // counts bits 7..0
    reg [7:0] shift;     // shift register

    // ----------------------------------------------------------------
    // FSM
    // ----------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state   <= S_IDLE;
            scl_oe  <= 1'b0;
            sda_oe  <= 1'b0;
            rdata   <= 8'h00;
            done    <= 1'b0;
            ack_err <= 1'b0;
            qtr     <= 2'd0;
            bit_cnt <= 4'd0;
            shift   <= 8'h00;
        end else begin
            done    <= 1'b0;  // default pulse-low

            if (tick) begin
                case (state)

                // --------------------------------------------------
                S_IDLE: begin
                    scl_oe <= 1'b0;   // release SCL (high)
                    sda_oe <= 1'b0;   // release SDA (high)
                    qtr    <= 2'd0;
                    if (start) begin
                        shift   <= {addr, rw};
                        bit_cnt <= 4'd7;
                        ack_err <= 1'b0;
                        state   <= S_START;
                    end
                end

                // --------------------------------------------------
                // START: pull SDA low while SCL high
                S_START: begin
                    case (qtr)
                        2'd0: begin scl_oe <= 1'b0; sda_oe <= 1'b0; end  // SCL=1, SDA=1
                        2'd1: begin scl_oe <= 1'b0; sda_oe <= 1'b1; end  // SCL=1, SDA=0 (START)
                        2'd2: begin scl_oe <= 1'b1; sda_oe <= 1'b1; end  // SCL=0, SDA=0
                        2'd3: begin
                            // put first bit on SDA
                            sda_oe <= ~shift[7];
                            state  <= S_ADDR;
                            qtr    <= 2'd0;
                        end
                    endcase
                    if (qtr != 2'd3) qtr <= qtr + 1;
                end

                // --------------------------------------------------
                // ADDR: clock out 8 bits (7-bit addr + R/W)
                S_ADDR: begin
                    case (qtr)
                        2'd0: begin scl_oe <= 1'b1; end                  // SCL low
                        2'd1: begin scl_oe <= 1'b0; end                  // SCL rising
                        2'd2: begin /* SCL high — slave samples */ end
                        2'd3: begin
                            scl_oe <= 1'b1;                               // SCL falling
                            if (bit_cnt == 0) begin
                                sda_oe <= 1'b0;   // release SDA for ACK
                                state  <= S_ADDR_ACK;
                                qtr    <= 2'd0;
                            end else begin
                                bit_cnt <= bit_cnt - 1;
                                shift   <= {shift[6:0], 1'b0};
                                sda_oe  <= ~shift[6];  // next bit
                                qtr     <= 2'd0;
                            end
                        end
                    endcase
                    if (qtr != 2'd3) qtr <= qtr + 1;
                end

                // --------------------------------------------------
                // Address ACK from slave
                S_ADDR_ACK: begin
                    case (qtr)
                        2'd0: begin scl_oe <= 1'b1; end
                        2'd1: begin scl_oe <= 1'b0; end
                        2'd2: begin
                            // sample ACK: slave pulls SDA low = ACK
                            if (sda_in) ack_err <= 1'b1;
                        end
                        2'd3: begin
                            scl_oe <= 1'b1;
                            qtr    <= 2'd0;
                            if (ack_err) begin
                                state <= S_STOP;
                            end else if (!rw) begin
                                shift   <= wdata;
                                bit_cnt <= 4'd7;
                                sda_oe  <= ~wdata[7];
                                state   <= S_WRITE;
                            end else begin
                                sda_oe  <= 1'b0;  // release SDA for read
                                bit_cnt <= 4'd7;
                                state   <= S_READ;
                            end
                        end
                    endcase
                    if (qtr != 2'd3) qtr <= qtr + 1;
                end

                // --------------------------------------------------
                // WRITE: clock out 8 data bits
                S_WRITE: begin
                    case (qtr)
                        2'd0: begin scl_oe <= 1'b1; end
                        2'd1: begin scl_oe <= 1'b0; end
                        2'd2: begin /* SCL high */ end
                        2'd3: begin
                            scl_oe <= 1'b1;
                            if (bit_cnt == 0) begin
                                sda_oe <= 1'b0;   // release for ACK
                                state  <= S_WACK;
                                qtr    <= 2'd0;
                            end else begin
                                bit_cnt <= bit_cnt - 1;
                                shift   <= {shift[6:0], 1'b0};
                                sda_oe  <= ~shift[6];
                                qtr     <= 2'd0;
                            end
                        end
                    endcase
                    if (qtr != 2'd3) qtr <= qtr + 1;
                end

                // --------------------------------------------------
                // Write ACK
                S_WACK: begin
                    case (qtr)
                        2'd0: begin scl_oe <= 1'b1; end
                        2'd1: begin scl_oe <= 1'b0; end
                        2'd2: begin
                            if (sda_in) ack_err <= 1'b1;
                        end
                        2'd3: begin
                            scl_oe <= 1'b1;
                            sda_oe <= 1'b1;   // hold SDA low for STOP setup
                            state  <= S_STOP;
                            qtr    <= 2'd0;
                        end
                    endcase
                    if (qtr != 2'd3) qtr <= qtr + 1;
                end

                // --------------------------------------------------
                // READ: clock in 8 data bits
                S_READ: begin
                    case (qtr)
                        2'd0: begin scl_oe <= 1'b1; end
                        2'd1: begin scl_oe <= 1'b0; end
                        2'd2: begin
                            shift <= {shift[6:0], sda_in};  // sample on SCL high
                        end
                        2'd3: begin
                            scl_oe <= 1'b1;
                            if (bit_cnt == 0) begin
                                rdata  <= shift;
                                // send NACK (master releases SDA = high) to end read
                                sda_oe <= 1'b0;
                                state  <= S_RACK;
                                qtr    <= 2'd0;
                            end else begin
                                bit_cnt <= bit_cnt - 1;
                                qtr     <= 2'd0;
                            end
                        end
                    endcase
                    if (qtr != 2'd3) qtr <= qtr + 1;
                end

                // --------------------------------------------------
                // Master sends NACK after last read byte
                S_RACK: begin
                    case (qtr)
                        2'd0: begin scl_oe <= 1'b1; end
                        2'd1: begin scl_oe <= 1'b0; end
                        2'd2: begin /* SCL high */ end
                        2'd3: begin
                            scl_oe <= 1'b1;
                            sda_oe <= 1'b1;  // SDA low for STOP setup
                            state  <= S_STOP;
                            qtr    <= 2'd0;
                        end
                    endcase
                    if (qtr != 2'd3) qtr <= qtr + 1;
                end

                // --------------------------------------------------
                // STOP: release SDA while SCL high
                S_STOP: begin
                    case (qtr)
                        2'd0: begin scl_oe <= 1'b1; sda_oe <= 1'b1; end  // SCL=0, SDA=0
                        2'd1: begin scl_oe <= 1'b0; end                   // SCL rising
                        2'd2: begin sda_oe <= 1'b0; end                   // SDA rising (STOP)
                        2'd3: begin
                            done  <= 1'b1;
                            state <= S_IDLE;
                            qtr   <= 2'd0;
                        end
                    endcase
                    if (qtr != 2'd3) qtr <= qtr + 1;
                end

                default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule
