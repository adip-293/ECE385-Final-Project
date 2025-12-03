/*
 * Camera Interface Module
 * 
 * Description:
 *   Captures pixel data from OV7670 camera and writes to BRAM.
 *   Assembles 8-bit byte stream into 12-bit RGB444 pixels.
 * 
 * Purpose:
 *   Bridge between camera's pixel clock domain and memory interface.
 *   Handle byte-to-pixel conversion and address generation.
 * 
 * Notes:
 *   - Camera outputs RGB444 as two 8-bit bytes per pixel
 *   - Byte 1: {R[3:0], G[3:2], xx}, Byte 2: {G[1:0], B[3:0], xx}
 *   - Address = y * 640 + x (linear frame buffer)
 */

module cam_interface (
    // Camera pixel clock domain inputs
    input  logic        pclk,           // Pixel clock from camera
    input  logic        vsync,          // Vertical sync (frame boundary)
    input  logic        href,           // Horizontal reference (line valid)    
    input  logic [7:0]  pixel_data,     // 8-bit pixel data from camera
    input  logic        cam_init_done,  // Camera initialization complete
    
    // Memory interface outputs
    output logic [18:0] mem_addr,       // Memory address for pixel storage
    output logic [11:0] mem_data,       // 12-bit RGB pixel data
    output logic        mem_write,      // Write enable for memory

    // Line status (for tear-free read coordination)
    output logic [9:0]  line_y,         // Current line index (0..479) in pclk domain
    output logic        line_ready_pulse // 1-cycle pulse when a line finishes (href falling edge)
);

    // ------------------------------------------------------------
    // Frame Edge Detection - VSYNC Processing
    // ------------------------------------------------------------
    logic vsync_d1, vsync_d2; 
    wire  frame_start = (~vsync_d1) & vsync_d2;   // Falling edge of vsync
    wire  frame_end   = vsync_d1 & (~vsync_d2);   // Rising edge of vsync
    
    always_ff @(posedge pclk) begin
        vsync_d1 <= vsync;
        vsync_d2 <= vsync_d1;
    end

    // ------------------------------------------------------------
    // Camera Capture State Machine
    // ------------------------------------------------------------
    typedef enum logic [1:0] {
        WAIT_FOR_INIT,   // Wait for camera initialization to complete
        FRAME_IDLE,      // Wait for frame start
        CAPTURE_PIXELS   // Actively capturing pixel data
    } capture_state_t;

    capture_state_t current_state, next_state;
    
    // ------------------------------------------------------------
    // Pixel Data Assembly - 8-bit to 12-bit Conversion
    // ------------------------------------------------------------
    logic        byte_toggle;     // Tracks first/second byte of pixel pair
    logic [3:0]  first_nibble;    // Stores first 4 bits from first byte
    logic [18:0] pixel_address;   // Current pixel memory address
    logic [11:0] assembled_pixel; // 12-bit assembled pixel data
    logic        write_enable;    // Memory write control
    logic [9:0]  x_cnt;           // X position within line (0..639)
    logic [9:0]  y_cnt;           // Y line counter (0..479)
    logic        href_d1;         // Delayed href for edge detection
    
    // State machine combinational logic
    always_comb begin
        case (current_state)
            WAIT_FOR_INIT: begin
                // Skip first two frames after initialization
                next_state = (frame_start && cam_init_done) ? FRAME_IDLE : WAIT_FOR_INIT;
            end
            
            FRAME_IDLE: begin
                // Wait for new frame to start capturing
                next_state = frame_start ? CAPTURE_PIXELS : FRAME_IDLE;
            end
            
            CAPTURE_PIXELS: begin
                // Continue capturing until frame ends
                next_state = frame_end ? FRAME_IDLE : CAPTURE_PIXELS;
            end
            
            default: begin
                next_state = WAIT_FOR_INIT;
            end
        endcase
    end

    // State machine sequential logic and pixel processing
    always_ff @(posedge pclk) begin
        // Default assignments
        write_enable <= 1'b0;
        line_ready_pulse <= 1'b0;
        href_d1 <= href;
        
        case (current_state)
            WAIT_FOR_INIT: begin
                pixel_address  <= 19'b0;
                assembled_pixel <= 12'b0;
                byte_toggle    <= 1'b0;
                x_cnt <= 10'd0;
                y_cnt <= 10'd0;
            end
            
            FRAME_IDLE: begin
                pixel_address  <= 19'b0;
                assembled_pixel <= 12'b0;
                byte_toggle    <= 1'b0;
                x_cnt <= 10'd0;
                y_cnt <= 10'd0;
            end
            
            CAPTURE_PIXELS: begin
                // Only process pixels when href is active (valid line data)
                if (href) begin
                    if (~byte_toggle) begin
                        // First byte: store lower 4 bits for later use
                        first_nibble <= pixel_data[3:0];
                        byte_toggle  <= 1'b1;
                    end else begin
                        // Second byte: combine with first byte to form 12-bit pixel
                        assembled_pixel <= {first_nibble, pixel_data};
                        
                        // Only write if within valid frame bounds (640x480)
                        // Address = y * 640 + x
                        if (y_cnt < 10'd480 && x_cnt < 10'd640) begin
                            write_enable    <= 1'b1;
                            // Calculate address from coordinates to prevent overflow
                            pixel_address   <= ({9'b0, y_cnt} << 9) + ({9'b0, y_cnt} << 7) + {9'b0, x_cnt};
                        end
                        
                        byte_toggle     <= 1'b0;

                        // X counter advances per pixel
                        if (x_cnt == 10'd639) begin
                            x_cnt <= 10'd0;
                        end else begin
                            x_cnt <= x_cnt + 10'd1;
                        end
                    end
                end else begin
                    // When href is low, reset byte toggle to ensure we start fresh on next line
                    byte_toggle <= 1'b0;
                    x_cnt <= 10'd0;
                end

                // Detect end-of-line on href falling edge; update y counter and pulse ready
                if (href_d1 && ~href) begin
                    line_ready_pulse <= 1'b1; // single-cycle pulse
                    if (y_cnt == 10'd479) begin
                        y_cnt <= 10'd0;
                    end else begin
                        y_cnt <= y_cnt + 10'd1;
                    end
                end
            end
        endcase
        
        // Update state register
        current_state <= next_state;
    end

    // ------------------------------------------------------------
    // Output Assignments
    // ------------------------------------------------------------
    assign mem_addr  = pixel_address;
    assign mem_data  = assembled_pixel;
    assign mem_write = write_enable;
    assign line_y    = y_cnt;
             
endmodule