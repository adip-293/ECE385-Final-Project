/*
 * Line Buffer Module
 * 
 * Description:
 *   Stores 3 rows of image data for spatial filtering operations.
 *   Provides 3x3 pixel neighborhood window for convolution/median filtering.
 * 
 * Purpose:
 *   Enable spatial operations by buffering multiple scan lines.
 *   Support real-time image processing without frame buffer.
 * 
 * Notes:
 *   - Uses distributed RAM (LUTs) for efficient small storage
 *   - Total storage: 3 rows × 640 pixels × 4 bits = 7,680 bits
 *   - Center pixel (p11) coordinates lag by 1 clock due to write latency
 */

module line_buffer (
    input  logic        clk,
    input  logic        rst_n,
    
    // Pixel input from camera (one pixel per clock when valid)
    input  logic [3:0]  pixel_in,      // 4-bit grayscale value
    input  logic        pixel_valid,   // High when pixel_in is valid
    input  logic        frame_start,   // Pulse at start of new frame (vsync edge)
    
    // 3x3 neighborhood output
    output logic [3:0]  p00, p01, p02,  // Top row
    output logic [3:0]  p10, p11, p12,  // Middle row
    output logic [3:0]  p20, p21, p22,  // Bottom row
    
    // Control outputs
    output logic        neighborhood_valid,  // High when 3x3 data is valid
    output logic [9:0]  out_x,              // X coordinate of center pixel (p11)
    output logic [9:0]  out_y               // Y coordinate of center pixel (p11)
);

    // Image dimensions
    localparam int WIDTH = 640;
    localparam int HEIGHT = 480;
    
    // Three line buffers (distributed RAM)
    logic [3:0] row0 [0:WIDTH-1];  // Oldest row
    logic [3:0] row1 [0:WIDTH-1];  // Middle row
    logic [3:0] row2 [0:WIDTH-1];  // Newest row
    
    // Position tracking
    logic [9:0] x_pos;      // Current X position (0-639)
    logic [9:0] y_pos;      // Current Y position (0-479)
    logic [1:0] row_count;  // How many rows we've accumulated (0-3)
    
    // Read indices for 3x3 neighborhood
    logic [9:0] x_left, x_center, x_right;
    
    // Position counter
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            x_pos <= 10'd0;
            y_pos <= 10'd0;
            row_count <= 2'd0;
        end else if (frame_start) begin
            x_pos <= 10'd0;
            y_pos <= 10'd0;
            row_count <= 2'd0;
        end else if (pixel_valid) begin
            if (x_pos == WIDTH - 1) begin
                // End of row
                x_pos <= 10'd0;
                if (y_pos < HEIGHT - 1) begin
                    y_pos <= y_pos + 10'd1;
                    if (row_count < 2'd3)
                        row_count <= row_count + 2'd1;
                end else begin
                    y_pos <= 10'd0;
                    row_count <= 2'd0;
                end
            end else begin
                x_pos <= x_pos + 10'd1;
            end
        end
    end
    
    // Shift rows and write new pixel
    always_ff @(posedge clk) begin
        if (pixel_valid) begin
            // Write incoming pixel to row2 (newest)
            row2[x_pos] <= pixel_in;
            
            // At end of each row, shift the row buffers
            if (x_pos == WIDTH - 1) begin
                // Shift: row0 <- row1 <- row2
                for (int i = 0; i < WIDTH; i++) begin
                    row0[i] <= row1[i];
                    row1[i] <= row2[i];
                end
            end
        end
    end
    
    // Calculate read positions for 3x3 window
    // Center pixel is at (x_pos-1, y_pos) due to 1-cycle write delay
    always_comb begin
        if (x_pos == 10'd0) begin
            x_left   = 10'd0;
            x_center = 10'd0;
            x_right  = 10'd1;
        end else if (x_pos == 10'd1) begin
            x_left   = 10'd0;
            x_center = 10'd0;
            x_right  = 10'd1;
        end else if (x_pos == WIDTH) begin
            x_left   = WIDTH - 2;
            x_center = WIDTH - 1;
            x_right  = WIDTH - 1;
        end else begin
            x_left   = x_pos - 10'd2;
            x_center = x_pos - 10'd1;
            x_right  = x_pos;
        end
    end
    
    // Read 3x3 neighborhood
    // row0 = top, row1 = middle, row2 = bottom
    assign p00 = row0[x_left];
    assign p01 = row0[x_center];
    assign p02 = row0[x_right];
    
    assign p10 = row1[x_left];
    assign p11 = row1[x_center];
    assign p12 = row1[x_right];
    
    assign p20 = row2[x_left];
    assign p21 = row2[x_center];
    assign p22 = row2[x_right];
    
    // Neighborhood is valid after we have at least 3 rows
    // For border pixels (x<1 or y<2), we'll still output but with clamped coordinates
    assign neighborhood_valid = (row_count >= 2'd3);
    
    // Output coordinates (center pixel position)
    // Center x corresponds to (x_pos - 1) due to write/read latency
    // Center y corresponds to previous line (y_pos - 1) once 3 rows are available
    assign out_x = (x_pos >= 10'd1) ? (x_pos - 10'd1) : 10'd0;
    assign out_y = (row_count >= 2'd3) ? (y_pos - 10'd1) : 10'd0;

endmodule
