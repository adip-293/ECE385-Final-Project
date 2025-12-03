/*
 * Camera Initializer Module
 * 
 * Description:
 *   Configures OV7670 camera registers via SCCB protocol.
 *   Sequences through ROM-stored register values with timing delays.
 * 
 * Purpose:
 *   Initialize camera with specific settings for RGB444 output mode.
 *   Handles SCCB write transactions and inter-command delays.
 * 
 * Notes:
 *   - Special ROM codes: 0xFFF0 = 10ms delay, 0xFFFF = done
 *   - 10ms delay calculated as (CLK_F * 10) / 1000
 *   - Uses async reset style to match original design
 */

module cam_initializer #(
    parameter int unsigned CLK_F  = 100_000_000,
    parameter int unsigned SCCB_F = 400_000
)(
    // System
    input  logic clk,
    input  logic rst_n,

    // Control signals
    input  logic cam_init_start,
    output logic cam_init_done,
    
    // SCCB Physical
    output logic sioc,
    inout  wire  siod, 
    
    // Debug/testbench signals
    output logic       data_sent_done,
    output logic [7:0] sccb_dout
);
    
    // ------------------------------------------------------------
    // Internal signals
    // ------------------------------------------------------------
    logic [7:0]  cam_rom_addr;
    logic [15:0] cam_rom_data;    
    logic [7:0]  sccb_reg_addr, sccb_wr_data;  
    logic        sccb_start, sccb_ready; 
    
    // ------------------------------------------------------------
    // Camera ROM Instance
    // ------------------------------------------------------------
    cam_rom data_rom (
        .clk(clk),
        .reset(rst_n), 
        .addr(cam_rom_addr),
        .dout(cam_rom_data)
    );
    
    // ------------------------------------------------------------
    // Configuration State Machine
    // ------------------------------------------------------------
    localparam ten_ms_delay = (CLK_F * 10) / 1000;
    localparam timer_size   = $clog2(ten_ms_delay);
    
    typedef enum logic [2:0] {
        IDLE  = 3'b000,
        SEND  = 3'b001, 
        DONE  = 3'b010,
        TIMER = 3'b011
    } state_t;

    state_t current_state, return_state;
    logic [timer_size-1:0] timer; 
    
    // Configuration state machine - using original async reset style
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state    <= IDLE;
            return_state     <= IDLE;
            cam_init_done    <= 1'b0;
            cam_rom_addr     <= 8'h00; 
            sccb_reg_addr    <= 8'h00;
            sccb_wr_data     <= 8'h00;
            sccb_start       <= 1'b0;
            timer            <= '0;
        end else begin
            case (current_state)
                IDLE: begin
                    current_state <= cam_init_start ? SEND : IDLE;
                end
                
                SEND: begin
                    if (sccb_ready) begin
                        case (cam_rom_data)
                            16'hFF_FF: begin
                                current_state <= DONE;
                            end
                            16'hFF_F0: begin
                                current_state <= TIMER;
                                return_state  <= SEND;
                                timer         <= ten_ms_delay;
                                cam_rom_addr  <= cam_rom_addr + 1'b1;
                            end
                            default: begin
                                current_state <= TIMER;
                                return_state  <= SEND;
                                timer         <= 1;
                                sccb_start    <= 1'b1;
                                sccb_reg_addr <= cam_rom_data[15:8];
                                sccb_wr_data  <= cam_rom_data[7:0];
                                cam_rom_addr  <= cam_rom_addr + 1'b1;
                            end
                        endcase
                    end
                end
                
                DONE: begin
                    current_state <= IDLE;
                    cam_init_done <= 1'b1;
                end
                
                TIMER: begin
                    current_state <= (timer == 1) ? return_state : TIMER;
                    timer         <= (timer == 1) ? '0 : timer - 1'b1;
                    sccb_start    <= 1'b0;
                end
                
                default: begin
                    current_state <= IDLE;
                end
            endcase
        end
    end
    // ------------------------------------------------------------
    // SCCB Master Instance
    // ------------------------------------------------------------      
    sccb_master #(
        .SYS_CLK_HZ(CLK_F),
        .SIOC_HZ(SCCB_F)
    ) sccb_master (
        // System
        .clk(clk),
        .rst(~rst_n),
        
        // Handshaking I/O
        .start(sccb_start),
        .ready(sccb_ready),
        .done(data_sent_done),
        
        // Transaction parameters
        .read(1'b0),              // Always write for camera config
        .dev_addr(7'h21),         // OV7670 device address 
        .reg_addr(sccb_reg_addr),
        .wr_data(sccb_wr_data),
        .rd_data(sccb_dout),      // Not used for writes but connected
        
        // SCCB Physical
        .sioc(sioc),      
        .siod(siod)
    );

endmodule