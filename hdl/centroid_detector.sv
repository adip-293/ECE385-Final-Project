/*
 * Centroid Detector Module
 * 
 * Description:
 *   Computes center of mass of white pixels in binary mask.
 *   Calculates blob centroid and bounding box using spatial moments.
 * 
 * Purpose:
 *   Track object position for motion control and overlay rendering.
 *   Filter noise blobs using minimum size threshold.
 * 
 * Notes:
 *   - Algorithm: centroid_x = sum(x_i) / N, centroid_y = sum(y_i) / N
 *   - Moment accumulators sized for 640x480 frame (19-bit count, 29-bit sums)
 *   - Centroid valid only if blob_area >= MIN_BLOB_SIZE
 *   - Bounding box tracks min/max coordinates of white pixels
 */

module centroid_detector #(
    parameter FRAME_WIDTH  = 640,
    parameter FRAME_HEIGHT = 480,
    parameter MIN_BLOB_SIZE = 300  // Minimum pixels to consider valid (filters noise)
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,
    
    // Pixel input
    input  logic [3:0]  mask_pixel,      // Binary mask (0x0 or 0xF)
    input  logic [9:0]  pixel_x,         // Current pixel X coordinate
    input  logic [9:0]  pixel_y,         // Current pixel Y coordinate
    input  logic        pixel_valid,     // Pixel data is valid
    
    // Frame synchronization
    input  logic        frame_start,     // Pulse at start of new frame
    
    // Centroid outputs (updated at end of frame)
    output logic [9:0]  centroid_x,      // X coordinate of centroid
    output logic [9:0]  centroid_y,      // Y coordinate of centroid
    output logic        centroid_valid,  // Centroid is valid (blob found)
    output logic [19:0] blob_area,       // Number of white pixels (for debugging)
    output logic [9:0]  bbox_min_x,      // Bounding box X min
    output logic [9:0]  bbox_min_y,      // Bounding box Y min
    output logic [9:0]  bbox_max_x,      // Bounding box X max
    output logic [9:0]  bbox_max_y       // Bounding box Y max
);

    localparam logic [9:0] FRAME_MAX_X = FRAME_WIDTH - 1;
    localparam logic [9:0] FRAME_MAX_Y = FRAME_HEIGHT - 1;

    // Moment accumulators (need to hold large sums)
    // Max values for 640x480 frame:
    //   sum_1: max 640*480 = 307200 → need 19 bits
    //   sum_x: max 640*307200 = 196608000 → need 28 bits
    //   sum_y: max 480*307200 = 147456000 → need 28 bits
    logic [19:0] sum_1;      // M00: Total pixel count
    logic [28:0] sum_x;      // M10: Sum of x coordinates
    logic [28:0] sum_y;      // M01: Sum of y coordinates
    
    // Next state accumulators
    logic [19:0] sum_1_next;
    logic [28:0] sum_x_next;
    logic [28:0] sum_y_next;
    
    // Frame state tracking
    logic frame_active;      // Currently accumulating frame
    logic frame_complete;    // Frame just finished, compute centroid
    logic frame_start_prev;
    
    // Detect frame start (rising edge)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            frame_start_prev <= 1'b0;
        else
            frame_start_prev <= frame_start;
    end
    
    assign frame_complete = frame_start & ~frame_start_prev;
    
    // Accumulator logic
    always_comb begin
        if (enable && pixel_valid && (|mask_pixel)) begin
            // Pixel is white (skin), accumulate moments
            sum_1_next = sum_1 + 20'd1;
            sum_x_next = sum_x + {19'd0, pixel_x};
            sum_y_next = sum_y + {19'd0, pixel_y};
        end else begin
            // No accumulation
            sum_1_next = sum_1;
            sum_x_next = sum_x;
            sum_y_next = sum_y;
        end
    end
    
    // Accumulator registers
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum_1 <= 20'd0;
            sum_x <= 29'd0;
            sum_y <= 29'd0;
            frame_active <= 1'b0;
        end else if (frame_complete) begin
            // Frame finished, reset accumulators for next frame
            sum_1 <= 20'd0;
            sum_x <= 29'd0;
            sum_y <= 29'd0;
            frame_active <= 1'b1;
        end else if (enable) begin
            // Accumulate during frame
            sum_1 <= sum_1_next;
            sum_x <= sum_x_next;
            sum_y <= sum_y_next;
            frame_active <= 1'b1;
        end
    end
    
    // Centroid computation (registered at end of frame).
    logic [9:0] centroid_x_calc;
    logic [9:0] centroid_y_calc;
    logic       centroid_valid_calc;

    logic [9:0] min_x_acc, min_x_next;
    logic [9:0] min_y_acc, min_y_next;
    logic [9:0] max_x_acc, max_x_next;
    logic [9:0] max_y_acc, max_y_next;

    always_comb begin
        if (sum_1 >= MIN_BLOB_SIZE[19:0]) begin
            // Valid blob detected, compute centroid
            centroid_x_calc = sum_x[28:0] / sum_1[19:0];  // M10 / M00
            centroid_y_calc = sum_y[28:0] / sum_1[19:0];  // M01 / M00
            centroid_valid_calc = 1'b1;
        end else begin
            centroid_x_calc = 10'd0;
            centroid_y_calc = 10'd0;
            centroid_valid_calc = 1'b0;
        end
    end

    always_comb begin
        min_x_next = min_x_acc;
        min_y_next = min_y_acc;
        max_x_next = max_x_acc;
        max_y_next = max_y_acc;

        if (enable && pixel_valid && (|mask_pixel)) begin
            min_x_next = (pixel_x < min_x_acc) ? pixel_x : min_x_acc;
            min_y_next = (pixel_y < min_y_acc) ? pixel_y : min_y_acc;
            max_x_next = (pixel_x > max_x_acc) ? pixel_x : max_x_acc;
            max_y_next = (pixel_y > max_y_acc) ? pixel_y : max_y_acc;
        end
    end

    // Output registers (update at frame completion)
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            centroid_x     <= 10'd0;
            centroid_y     <= 10'd0;
            centroid_valid <= 1'b0;
            blob_area      <= 20'd0;
            bbox_min_x     <= 10'd0;
            bbox_min_y     <= 10'd0;
            bbox_max_x     <= 10'd0;
            bbox_max_y     <= 10'd0;
            min_x_acc      <= FRAME_MAX_X;
            min_y_acc      <= FRAME_MAX_Y;
            max_x_acc      <= 10'd0;
            max_y_acc      <= 10'd0;
        end else if (frame_complete && enable) begin
            centroid_x     <= centroid_x_calc;
            centroid_y     <= centroid_y_calc;
            centroid_valid <= centroid_valid_calc;
            blob_area      <= sum_1;

            if (centroid_valid_calc) begin
                bbox_min_x <= min_x_acc;
                bbox_min_y <= min_y_acc;
                bbox_max_x <= max_x_acc;
                bbox_max_y <= max_y_acc;
            end else begin
                bbox_min_x <= 10'd0;
                bbox_min_y <= 10'd0;
                bbox_max_x <= 10'd0;
                bbox_max_y <= 10'd0;
            end

            min_x_acc <= FRAME_MAX_X;
            min_y_acc <= FRAME_MAX_Y;
            max_x_acc <= 10'd0;
            max_y_acc <= 10'd0;
        end else if (enable) begin
            min_x_acc <= min_x_next;
            min_y_acc <= min_y_next;
            max_x_acc <= max_x_next;
            max_y_acc <= max_y_next;
        end
    end

endmodule
