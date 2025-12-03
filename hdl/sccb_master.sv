/*
 * SCCB Master Module
 * 
 * Description:
 *   Serial Camera Control Bus (SCCB) master controller for OV7670 camera configuration.
 *   Implements 3-phase write and 2-phase read protocols with configurable clock generation.
 * 
 * Purpose:
 *   - Generate SIOC clock from system clock
 *   - Handle SCCB write transactions (3-phase: ID+reg+data)
 *   - Handle SCCB read transactions (2-phase: ID+reg, then ID+read)
 *   - Provide handshaking interface (start/ready/done)
 * 
 * Notes:
 *   - SIOC is push-pull output (idles high)
 *   - SIOD is open-drain bidirectional (drive 0 or release to Z)
 *   - Default configuration: 100 MHz system clock, 400 kHz SCCB clock
 *   - State machine ensures proper setup/hold timing
 */
module sccb_master #(
    parameter int unsigned SYS_CLK_HZ = 100_000_000, // FPGA clock
    parameter int unsigned SIOC_HZ    = 400_000      // SCCB clock target
)(
    // System
    input  logic clk,
    input  logic rst,

    // Handshaking I/O
    input  logic       start,   // 1-cycle pulse to start transaction
    output logic       ready,   // High when idle and able to accept start
    output logic       done,    // 1-cycle pulse at successful completion
    
    // Transaction parameters
    input  logic       read,       // 1=read, 0=write
    input  logic [6:0] dev_addr,   // 8'h42 write, 8'h43 read
    input  logic [7:0] reg_addr,
    input  logic [7:0] wr_data,
    output logic [7:0] rd_data,

    // SCCB Physical
    output logic sioc,  // push-pull clock (idles high)
    inout  wire  siod   // open-drain data (drive 0 or release Z)
);

    // ------------------------------------------------------------
    // SCCB Clock Generation - SIOC
    // ------------------------------------------------------------
    localparam int unsigned SIOC_PERIOD        = SYS_CLK_HZ / SIOC_HZ;
    localparam int unsigned SIOC_HALF_PERIOD   = SIOC_PERIOD / 2;
    localparam int unsigned SIOC_QUARTER_PERIOD = SIOC_PERIOD / 4;

    localparam int unsigned SIOC_W = (SIOC_HALF_PERIOD <= 1) ? 1 : $clog2(SIOC_HALF_PERIOD);
    logic [SIOC_W-1:0] sioc_count;
    logic sioc_enable;     // Asserted by FSM when bus active
    logic sioc_prev;       // Used for edge detection
    wire  sioc_rising_edge = (sioc & ~sioc_prev) && sioc_enable;
    wire  sioc_falling_edge = (~sioc & sioc_prev) && sioc_enable;

    always_ff @(posedge clk) begin
        if (rst || !sioc_enable) begin
            sioc_count <= '0;
            sioc       <= 1'b1; // idle high
        end else begin
            if (sioc_count == SIOC_HALF_PERIOD-1) begin
                sioc_count <= 0;
                sioc       <= ~sioc; // toggle SIOC
            end else begin
                sioc_count <= sioc_count + 1;
            end
        end

        sioc_prev <= rst ? 1'b1 : sioc;
    end

    // ------------------------------------------------------------
    // Hold Time Counter - Start/Stop Conditions
    // ------------------------------------------------------------
    localparam int unsigned HOLD_HALF = SIOC_HALF_PERIOD;    // Full High Pulse
    localparam int unsigned HOLD_QTR  = SIOC_QUARTER_PERIOD; // Half High Pulse
    localparam int unsigned HOLD_W    = (HOLD_HALF <= 1) ? 1 : $clog2(HOLD_HALF);

    logic [HOLD_W-1:0] hold_count;
    wire HOLD_HALF_DONE    = (hold_count == HOLD_HALF-1);
    wire HOLD_QUARTER_DONE = (hold_count == HOLD_QTR-1);

    always_ff @(posedge clk) begin
        if (rst) begin
            hold_count <= '0;
        end else if (current_state == START_1 || current_state == START_2 ||
                 current_state == STOP_1  || current_state == STOP_2) begin
            if (!HOLD_HALF_DONE)
                hold_count <= hold_count + 1;
        end else begin
            hold_count <= '0;
        end
    end

    // ------------------------------------------------------------
    // SIOD Control - Open Drain
    // ------------------------------------------------------------
    logic siod_out_en; // 1=drive low, 0=release Z
    assign siod = siod_out_en ? 1'b0 : 1'bz;

    //------------------------------------------------------------
    // Bit Counter and Data Shift Register
    //------------------------------------------------------------
    logic [7:0] transmission_byte;
    logic [3:0] bit_count; //9 Total bits (8 data + 1 ack)

    always_ff @(posedge clk) begin
        if (rst) begin
            transmission_byte   <= 8'h00;
            bit_count  <= 4'd8;
            siod_out_en <= 1'b0; // release SIOD (idle high)
        end else begin
            // ---- Load new byte when entering a WRITE_* state ----
            if (current_state!=WRITE_DEV && next_state==WRITE_DEV) begin
                // First device byte is ALWAYS write (R/W=0), even for read transactions
                transmission_byte <= {dev_addr, 1'b0};
                bit_count <= 4'd8;
            end
            if (current_state!=WRITE_REG && next_state==WRITE_REG) begin
                transmission_byte <= reg_addr;
                bit_count <= 4'd8;
            end
            if (current_state!=WRITE_DATA && next_state==WRITE_DATA) begin
                transmission_byte <= wr_data;
                bit_count <= 4'd8;
            end
            if (current_state!=WRITE_DEV_RD && next_state==WRITE_DEV_RD) begin
                // Second device byte after repeated START is read (R/W=1)
                transmission_byte <= {dev_addr, 1'b1};
                bit_count <= 4'd8;
            end

            // ---- While in any WRITE_* state, handle bits/ACK on SCL edges ----
            if (current_state==WRITE_DEV || current_state==WRITE_REG || current_state==WRITE_DATA || current_state==WRITE_DEV_RD) begin
                // Place next value on SIOD at SCL falling edge
                if (sioc_falling_edge) begin
                    if (bit_count > 0) begin
                        // Drive data bit: 0 -> pull low, 1 -> release (open drain)
                        siod_out_en <= ~transmission_byte[bit_count-1];
                    end else begin
                        // ACK bit (9th): release SDA so the slave could pull low (SCCB may ignore ACK)
                        siod_out_en <= 1'b0;
                    end
                end

                // Advance on SCL rising edge
                if (sioc_rising_edge) begin
                    if (bit_count > 0) begin
                        bit_count <= bit_count - 1'b1;
                    end else begin
                        // Rising edge of ACK bit -> byte complete
                        bit_count <= 4'd8;
                        // (State advance happens in next-state logic above)
                    end
                end
            end

            // ---- READ phase: slave drives data, master samples on SCL rising edges ----
            if (current_state==READ_DATA) begin
                // During the 8 data bits: release SIOD so slave can drive
                if (bit_count > 0) begin
                    siod_out_en <= 1'b0; // release (open-drain high)
                    if (sioc_rising_edge) begin
                    // Shift in on SCL rising edges, MSB-first
                    rd_data[bit_count-1] <= siod;
                    bit_count <= bit_count - 1'b1;
                    end
                end else begin
                    // 9th bit = master NACK (for last byte): keep SIOD released (high)
                    siod_out_en <= 1'b0; // NACK
                    // After the rising edge of the 9th clock, we'll STOP
                    // (state transition handles leaving READ_DATA)
                end
            end

            if(current_state==ACK) begin
                if (sioc_falling_edge) begin
                    siod_out_en <= 1'b0; // release (open-drain high) - was incorrectly 1'b1
                end
            end

            // Handle SIOD control for state transitions
            case (next_state)
                START_1, RESTART_1: siod_out_en <= 1'b0; // SDA high (prepare for START)
                START_2, RESTART_2: siod_out_en <= 1'b1; // SDA low (START / repeated START)
                STOP_1:  siod_out_en <= 1'b1;           // SDA low (prepare for STOP)
                STOP_2:  siod_out_en <= 1'b0;           // release (STOP)
                default: ; // Keep current value
            endcase
        end
    end


    // ------------------------------------------------------------
    // SCCB FSM
    // ------------------------------------------------------------
    typedef enum logic [3:0] {
        IDLE,
        START_1,
        START_2,
        STOP_1,
        STOP_2,
        WRITE_DEV,
        WRITE_REG,
        WRITE_DATA,
        RESTART_1, 
        RESTART_2,
        ACK,
        WRITE_DEV_RD,   // device addr + R
        READ_DATA,      // read 8 bits, then NACK
        DONE
    } state_t;

    state_t current_state, next_state;

    // Output logic
    always_comb begin
        // defaults
        ready       = 0;
        done        = 0;
        sioc_enable = 1'b0; // disable SCL clocking

        unique case (current_state)
            IDLE: ready = 1'b1;

            START_1,
            START_2: begin end//SCL high, SDA handled in bit counter

            WRITE_DEV,
            WRITE_REG,
            WRITE_DATA,
            WRITE_DEV_RD,
            READ_DATA,
            ACK: begin
                sioc_enable = 1'b1; // enable SCL clocking
                //SIOD driven in bit counter logic
            end

            STOP_1,
            STOP_2: begin end//SCL high, SDA handled in bit counter

            DONE: done = 1'b1;
            
            default: begin end
        endcase
    end

    // State transition logic
    always_comb begin
        case (current_state)
            IDLE:     next_state = start ? START_1 : IDLE;

            START_1:  next_state = HOLD_QUARTER_DONE ? START_2 : START_1;
            START_2:  next_state = HOLD_HALF_DONE    ? WRITE_DEV : START_2;

            // Write first device addr (R/W=0) then register
            WRITE_DEV: next_state = (bit_count==0 && sioc_rising_edge) ? WRITE_REG : WRITE_DEV;

            WRITE_REG: begin
                if (bit_count==0 && sioc_rising_edge) begin
                    if (!read)   next_state = WRITE_DATA;   // pure write path
                    else         next_state = RESTART_1;    // read path -> repeated START
                end else next_state = WRITE_REG;
            end

            // Pure write path
            WRITE_DATA: next_state = (bit_count==0 && sioc_rising_edge) ? ACK : WRITE_DATA;

            // Repeated START, then device addr with R/W=1
            RESTART_1: next_state = HOLD_QUARTER_DONE ? RESTART_2 : RESTART_1;
            RESTART_2: next_state = HOLD_HALF_DONE    ? WRITE_DEV_RD : RESTART_2;

            WRITE_DEV_RD: next_state = (bit_count==0 && sioc_rising_edge) ? READ_DATA : WRITE_DEV_RD;

            ACK: next_state = sioc_rising_edge ? STOP_1 : ACK;

            // Read one byte, then STOP (NACK generated by keeping SDA released)
            READ_DATA: next_state = (bit_count==0 && sioc_rising_edge) ? ACK : READ_DATA;

            STOP_1:   next_state = HOLD_QUARTER_DONE ? STOP_2 : STOP_1;
            STOP_2:   next_state = HOLD_HALF_DONE    ? DONE   : STOP_2;

            DONE:     next_state = IDLE;
            default:  next_state = IDLE;
        endcase
    end


    // State register
    always_ff @(posedge clk) begin
        if (rst) begin
            current_state <= IDLE;
        end else begin
            current_state <= next_state;
        end
    end

endmodule
