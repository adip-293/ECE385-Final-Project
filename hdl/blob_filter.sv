/*
 * Blob Filter Module
 * 
 * Description:
 *   Filters blob mask to region around previously detected centroid with adaptive padding.
 *   Increases padding when motion is detected for better tracking continuity.
 * 
 * Purpose:
 *   Reduce false positives by restricting processing to region of interest.
 *   Persist bounding box for several frames after detection loss.
 * 
 * Notes:
 *   - Adaptive padding: larger when motion detected (centroid displacement > threshold)
 *   - Bbox persistence: maintains last valid bbox for BBOX_PERSIST_FRAMES after loss
 *   - Motion detection: tracks centroid displacement between frames
 */

module blob_filter #(
    parameter logic [9:0] PADDING_X = 10'd40,
    parameter logic [9:0] PADDING_Y = 10'd30,
    parameter logic [9:0] MOTION_PADDING_X = 10'd25,
    parameter logic [9:0] MOTION_PADDING_Y = 10'd20,
    // Suppress pixels near image borders to avoid bbox stretching when touching edges
    parameter logic [9:0] EDGE_MARGIN = 10'd4
)(
    // System
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,
    
    // Pixel inputs
    input  logic [3:0]  mask_pixel,
    input  logic [9:0]  pixel_x,
    input  logic [9:0]  pixel_y,
    
    // Bounding box from centroid detector
    input  logic [9:0]  bbox_min_x,
    input  logic [9:0]  bbox_min_y,
    input  logic [9:0]  bbox_max_x,
    input  logic [9:0]  bbox_max_y,
    input  logic        bbox_valid,
    input  logic        frame_start,
    
    // Output
    output logic [3:0]  pixel_out
);

    localparam logic [9:0] FRAME_MAX_X = 10'd639;
    localparam logic [9:0] FRAME_MAX_Y = 10'd479;
    localparam int MOTION_THRESHOLD = 15;  // Pixels of movement to consider "moving"
    localparam int BBOX_PERSIST_FRAMES = 3; // Keep bbox valid for this many frames after loss

    // Track previous bbox for motion detection
    logic [9:0] prev_bbox_min_x, prev_bbox_min_y, prev_bbox_max_x, prev_bbox_max_y;
    logic       prev_bbox_was_valid;
    logic [2:0] bbox_invalid_counter;  // Count frames since bbox became invalid
    logic       bbox_persistent_valid; // Extended validity
    
    // Detect motion by comparing current and previous bbox centers
    logic signed [10:0] bbox_center_x, bbox_center_y;
    logic signed [10:0] prev_center_x, prev_center_y;
    logic signed [10:0] motion_dx, motion_dy;
    logic motion_detected;
    
    assign bbox_center_x = {1'b0, bbox_min_x} + {1'b0, bbox_max_x}; // 2x center
    assign bbox_center_y = {1'b0, bbox_min_y} + {1'b0, bbox_max_y}; // 2x center
    assign prev_center_x = {1'b0, prev_bbox_min_x} + {1'b0, prev_bbox_max_x};
    assign prev_center_y = {1'b0, prev_bbox_min_y} + {1'b0, prev_bbox_max_y};
    
    // Motion delta (divided by 2 since centers are 2x actual)
    assign motion_dx = (bbox_center_x - prev_center_x) >>> 1;
    assign motion_dy = (bbox_center_y - prev_center_y) >>> 1;
    
    // Motion detected if either axis exceeds threshold
    assign motion_detected = prev_bbox_was_valid && bbox_valid && 
                            ((motion_dx > MOTION_THRESHOLD) || (motion_dx < -MOTION_THRESHOLD) ||
                             (motion_dy > MOTION_THRESHOLD) || (motion_dy < -MOTION_THRESHOLD));
    
    // Maintain bbox validity for several frames after loss
    assign bbox_persistent_valid = bbox_valid || (bbox_invalid_counter < BBOX_PERSIST_FRAMES);
    
    // Update motion tracking on each frame
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_bbox_min_x <= 10'd0;
            prev_bbox_min_y <= 10'd0;
            prev_bbox_max_x <= 10'd0;
            prev_bbox_max_y <= 10'd0;
            prev_bbox_was_valid <= 1'b0;
            bbox_invalid_counter <= 3'd0;
        end else if (frame_start) begin
            if (bbox_valid) begin
                prev_bbox_min_x <= bbox_min_x;
                prev_bbox_min_y <= bbox_min_y;
                prev_bbox_max_x <= bbox_max_x;
                prev_bbox_max_y <= bbox_max_y;
                prev_bbox_was_valid <= 1'b1;
                bbox_invalid_counter <= 3'd0;
            end else begin
                // Keep previous bbox but increment invalid counter
                if (bbox_invalid_counter < 3'd7) begin
                    bbox_invalid_counter <= bbox_invalid_counter + 3'd1;
                end
                if (bbox_invalid_counter >= BBOX_PERSIST_FRAMES) begin
                    prev_bbox_was_valid <= 1'b0;
                end
            end
        end
    end

    function automatic logic [9:0] saturating_sub(input logic [9:0] value, input logic [9:0] amount);
        saturating_sub = (value > amount) ? (value - amount) : 10'd0;
    endfunction

    function automatic logic [9:0] saturating_add(input logic [9:0] value, input logic [9:0] amount, input logic [9:0] limit);
        saturating_add = (value + amount >= limit) ? limit : (value + amount);
    endfunction

    logic [9:0] padded_min_x, padded_max_x, padded_min_y, padded_max_y;
    logic [9:0] effective_padding_x, effective_padding_y;
    logic [9:0] bbox_to_use_min_x, bbox_to_use_min_y, bbox_to_use_max_x, bbox_to_use_max_y;
    
    // Use adaptive padding: larger when motion is detected
    assign effective_padding_x = motion_detected ? (PADDING_X + MOTION_PADDING_X) : PADDING_X;
    assign effective_padding_y = motion_detected ? (PADDING_Y + MOTION_PADDING_Y) : PADDING_Y;
    
    // Use current bbox if valid, otherwise use previous (persistent) bbox
    assign bbox_to_use_min_x = bbox_valid ? bbox_min_x : prev_bbox_min_x;
    assign bbox_to_use_min_y = bbox_valid ? bbox_min_y : prev_bbox_min_y;
    assign bbox_to_use_max_x = bbox_valid ? bbox_max_x : prev_bbox_max_x;
    assign bbox_to_use_max_y = bbox_valid ? bbox_max_y : prev_bbox_max_y;

    assign padded_min_x = saturating_sub(bbox_to_use_min_x, effective_padding_x);
    assign padded_min_y = saturating_sub(bbox_to_use_min_y, effective_padding_y);
    assign padded_max_x = saturating_add(bbox_to_use_max_x, effective_padding_x, FRAME_MAX_X);
    assign padded_max_y = saturating_add(bbox_to_use_max_y, effective_padding_y, FRAME_MAX_Y);

    logic inside_bbox;
    assign inside_bbox = (pixel_x >= padded_min_x) && (pixel_x <= padded_max_x) &&
                         (pixel_y >= padded_min_y) && (pixel_y <= padded_max_y);

    // Edge guard: true when pixel is within EDGE_MARGIN of any frame border
    logic is_edge_pixel;
    always_comb begin
        is_edge_pixel = (pixel_x < EDGE_MARGIN) || (pixel_x > (FRAME_MAX_X - EDGE_MARGIN)) ||
                        (pixel_y < EDGE_MARGIN) || (pixel_y > (FRAME_MAX_Y - EDGE_MARGIN));
    end

    always_comb begin
        if (!enable || !bbox_persistent_valid) begin
            // Pass through when disabled or no bbox history, but still suppress edge pixels
            pixel_out = is_edge_pixel ? 4'b0 : mask_pixel;
        end else if (inside_bbox && !is_edge_pixel) begin
            // Keep pixels inside padded bbox and not on frame edges
            pixel_out = mask_pixel;
        end else begin
            // Filter out pixels outside bbox or on edges
            pixel_out = 4'b0;
        end
    end

endmodule

