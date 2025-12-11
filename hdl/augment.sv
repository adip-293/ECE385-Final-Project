`timescale 1ns / 1ps

/*
 * Augment Module
 * 
 * Description:
 *   Draws a square and circle overlay on the video output.
 *   Both can be grabbed and dragged when fist gesture is detected and centroid is within bounds.
 *   Otherwise, they stay in their current positions.
 * 
 * Purpose:
 *   Visual augmentation for gesture recognition demo.
 *   Demonstrates interactive control via gesture recognition.
 * 
 * Notes:
 *   - Square size: 80x80 pixels
 *   - Circle radius: 40 pixels
 *   - Square color: bright blue, Circle color: bright green
 *   - Objects grabbed when fist detected and centroid within bounds
 *   - Objects maintain offset from grab point while being dragged
 *   - Square initialized to center-left, circle to center-right
 *   - Uses pre-grip centroid position to prevent shifting during grab
 *   - Has tolerance zone to prevent dropping when moving fast
 */

module augment #(
    parameter FRAME_WIDTH  = 640,
    parameter FRAME_HEIGHT = 480,
    parameter SQUARE_SIZE  = 80,
    parameter CIRCLE_RADIUS = 40
)(
    // System
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,
    
    // Centroid inputs
    input  logic [9:0]  centroid_x,
    input  logic [9:0]  centroid_y,
    input  logic        centroid_valid,
    
    // Gesture inputs
    input  logic        gesture_fist,
    input  logic        gesture_open,
    input  logic        gesture_wave,
    
    // VGA timing
    input  logic [9:0]  draw_x,
    input  logic [9:0]  draw_y,
    input  logic        vde,
    
    // Input pixels
    input  logic [2:0]  pixel_red_in,
    input  logic [2:0]  pixel_green_in,
    input  logic [2:0]  pixel_blue_in,
    
    // Output pixels (with square overlay)
    output logic [2:0]  pixel_red_out,
    output logic [2:0]  pixel_green_out,
    output logic [2:0]  pixel_blue_out
);

    // Shape colors
    // Blue: unselected/default
    localparam logic [2:0] SHAPE_BLUE_RED   = 3'b000;
    localparam logic [2:0] SHAPE_BLUE_GREEN = 3'b000;
    localparam logic [2:0] SHAPE_BLUE_BLUE  = 3'b111;
    
    // Green: hovered and ready to grab (fist detected + centroid in bounds)
    localparam logic [2:0] SHAPE_GREEN_RED   = 3'b000;
    localparam logic [2:0] SHAPE_GREEN_GREEN = 3'b111;
    localparam logic [2:0] SHAPE_GREEN_BLUE  = 3'b000;
    
    // Square position (center of square)
    logic [9:0] square_x, square_x_next;
    logic [9:0] square_y, square_y_next;
    
    // Circle position (center of circle)
    logic [9:0] circle_x, circle_x_next;
    logic [9:0] circle_y, circle_y_next;
    
    // Grab state: square and circle being dragged
    logic square_grabbed, square_grabbed_next;
    logic circle_grabbed, circle_grabbed_next;
    
    // Square grab offset: offset from square center to where it was grabbed (signed)
    logic signed [10:0] square_grab_offset_x, square_grab_offset_x_next;
    logic signed [10:0] square_grab_offset_y, square_grab_offset_y_next;
    
    // Circle grab offset: offset from circle center to where it was grabbed (signed)
    logic signed [10:0] circle_grab_offset_x, circle_grab_offset_x_next;
    logic signed [10:0] circle_grab_offset_y, circle_grab_offset_y_next;
    
    // Frame synchronization
    logic frame_start;
    assign frame_start = (draw_x == 10'd0) && (draw_y == 10'd0) && vde;
    logic frame_start_prev;
    
    // Tolerance zone for preventing drops when moving fast
    // Don't release if centroid moves slightly outside square
    localparam [9:0] TOLERANCE_ZONE = 10'd30;  // Extra pixels outside square before releasing
    
    // Square bounds
    logic [9:0] square_min_x, square_max_x;
    logic [9:0] square_min_y, square_max_y;
    
    // Compute square bounds
    always_comb begin
        square_min_x = (square_x >= (SQUARE_SIZE / 2)) ? (square_x - (SQUARE_SIZE / 2)) : 10'd0;
        square_max_x = (square_x + (SQUARE_SIZE / 2) < FRAME_WIDTH) ? (square_x + (SQUARE_SIZE / 2)) : (FRAME_WIDTH - 10'd1);
        square_min_y = (square_y >= (SQUARE_SIZE / 2)) ? (square_y - (SQUARE_SIZE / 2)) : 10'd0;
        square_max_y = (square_y + (SQUARE_SIZE / 2) < FRAME_HEIGHT) ? (square_y + (SQUARE_SIZE / 2)) : (FRAME_HEIGHT - 10'd1);
    end
    
    // Check if centroid is within square bounds
    logic centroid_in_square;
    logic centroid_in_square_tolerance;
    
    // Check if centroid is within circle bounds
    logic centroid_in_circle;
    logic centroid_in_circle_tolerance;
    
    // Distance from centroid to circle center
    logic signed [10:0] circle_dx, circle_dy;
    logic [20:0] circle_dist_sq;  // Distance squared (to avoid sqrt)
    
    always_comb begin
        // Square bounds check
        centroid_in_square = centroid_valid && 
                             (centroid_x >= square_min_x) && (centroid_x <= square_max_x) &&
                             (centroid_y >= square_min_y) && (centroid_y <= square_max_y);
        
        // Square tolerance check: within square + tolerance zone (only used when grabbed)
        if (square_grabbed) begin
            centroid_in_square_tolerance = centroid_valid &&
                                    (centroid_x >= (square_min_x > TOLERANCE_ZONE ? square_min_x - TOLERANCE_ZONE : 10'd0)) &&
                                    (centroid_x <= (square_max_x + TOLERANCE_ZONE < FRAME_WIDTH ? square_max_x + TOLERANCE_ZONE : FRAME_WIDTH - 10'd1)) &&
                                    (centroid_y >= (square_min_y > TOLERANCE_ZONE ? square_min_y - TOLERANCE_ZONE : 10'd0)) &&
                                    (centroid_y <= (square_max_y + TOLERANCE_ZONE < FRAME_HEIGHT ? square_max_y + TOLERANCE_ZONE : FRAME_HEIGHT - 10'd1));
        end else begin
            centroid_in_square_tolerance = centroid_in_square;
        end
        
        // Circle bounds check: distance from centroid to circle center
        circle_dx = $signed({1'b0, centroid_x}) - $signed({1'b0, circle_x});
        circle_dy = $signed({1'b0, centroid_y}) - $signed({1'b0, circle_y});
        circle_dist_sq = circle_dx * circle_dx + circle_dy * circle_dy;
        
        // Check if within circle radius (using squared distance to avoid sqrt)
        centroid_in_circle = centroid_valid && (circle_dist_sq <= (CIRCLE_RADIUS * CIRCLE_RADIUS));
        
        // Circle tolerance check: within circle + tolerance zone (only used when grabbed)
        if (circle_grabbed) begin
            extended_radius = CIRCLE_RADIUS + TOLERANCE_ZONE;
            centroid_in_circle_tolerance = centroid_valid && (circle_dist_sq <= (extended_radius * extended_radius));
        end else begin
            centroid_in_circle_tolerance = centroid_in_circle;
        end
    end
    
    // Variables for position calculations (declared outside always_comb)
    logic signed [10:0] new_square_x_signed, new_square_y_signed;
    logic [9:0] new_square_x, new_square_y;
    logic signed [10:0] new_circle_x_signed, new_circle_y_signed;
    logic [9:0] new_circle_x, new_circle_y;
    logic [9:0] extended_radius;
    
    // Hover detection: shape is ready to be grabbed (centroid in bounds + fist + not already grabbed)
    logic square_hovered;  // Square can be grabbed
    logic circle_hovered;  // Circle can be grabbed
    
    // Hover detection logic (separate always_comb block)
    always_comb begin
        // Hover detection: ready to grab if centroid in bounds, fist detected, and not already grabbed
        // Also show green when already grabbed (to indicate it's being dragged)
        if (enable && centroid_valid) begin
            // Square hover: show green if ready to grab or already grabbed
            if (square_grabbed) begin
                square_hovered = 1'b1;  // Show green when grabbed (being dragged)
            end else if (gesture_fist && centroid_in_square && !circle_grabbed) begin
                square_hovered = 1'b1;  // Show green when ready to grab
            end else begin
                square_hovered = 1'b0;  // Blue otherwise
            end
            
            // Circle hover: show green if ready to grab or already grabbed
            if (circle_grabbed) begin
                circle_hovered = 1'b1;  // Show green when grabbed (being dragged)
            end else if (gesture_fist && centroid_in_circle && !square_grabbed) begin
                circle_hovered = 1'b1;  // Show green when ready to grab
            end else begin
                circle_hovered = 1'b0;  // Blue otherwise
            end
        end else begin
            square_hovered = 1'b0;
            circle_hovered = 1'b0;
        end
    end
    
    // Square and circle position and grab state update logic
    always_comb begin
        square_x_next = square_x;
        square_y_next = square_y;
        circle_x_next = circle_x;
        circle_y_next = circle_y;
        square_grabbed_next = square_grabbed;
        circle_grabbed_next = circle_grabbed;
        square_grab_offset_x_next = square_grab_offset_x;
        square_grab_offset_y_next = square_grab_offset_y;
        circle_grab_offset_x_next = circle_grab_offset_x;
        circle_grab_offset_y_next = circle_grab_offset_y;
        
        if (enable && centroid_valid) begin
            // Square grab logic
            if (!square_grabbed) begin
                // Not grabbed: check if we should grab (only if circle is not grabbed)
                if (gesture_fist && centroid_in_square && !circle_grabbed) begin
                    // Fist detected and centroid in square: grab it!
                    // Store the PRE-GRIP centroid position and calculate offset from square center
                    square_grab_offset_x_next = $signed({1'b0, centroid_x}) - $signed({1'b0, square_x});
                    square_grab_offset_y_next = $signed({1'b0, centroid_y}) - $signed({1'b0, square_y});
                    square_grabbed_next = 1'b1;
                end
            end else begin
                // Currently grabbed: check if we should release or continue dragging
                if (!gesture_fist) begin
                    // Fist released: release
                    square_grabbed_next = 1'b0;
                end else if (!centroid_in_square_tolerance) begin
                    // Centroid moved significantly outside tolerance zone: release
                    square_grabbed_next = 1'b0;
                end else begin
                    // Still grabbed: move square to maintain offset from grab point
                    new_square_x_signed = $signed({1'b0, centroid_x}) - square_grab_offset_x;
                    new_square_y_signed = $signed({1'b0, centroid_y}) - square_grab_offset_y;
                    
                    // Clamp to screen bounds
                    if (new_square_x_signed < $signed({1'b0, SQUARE_SIZE / 2})) begin
                        square_x_next = SQUARE_SIZE / 2;
                    end else if (new_square_x_signed > $signed({1'b0, FRAME_WIDTH - (SQUARE_SIZE / 2)})) begin
                        square_x_next = FRAME_WIDTH - (SQUARE_SIZE / 2);
                    end else begin
                        square_x_next = new_square_x_signed[9:0];
                    end
                    
                    if (new_square_y_signed < $signed({1'b0, SQUARE_SIZE / 2})) begin
                        square_y_next = SQUARE_SIZE / 2;
                    end else if (new_square_y_signed > $signed({1'b0, FRAME_HEIGHT - (SQUARE_SIZE / 2)})) begin
                        square_y_next = FRAME_HEIGHT - (SQUARE_SIZE / 2);
                    end else begin
                        square_y_next = new_square_y_signed[9:0];
                    end
                end
            end
            
            // Circle grab logic (similar to square)
            if (!circle_grabbed) begin
                // Not grabbed: check if we should grab (only if square is not grabbed)
                if (gesture_fist && centroid_in_circle && !square_grabbed) begin
                    // Fist detected and centroid in circle: grab it!
                    circle_grab_offset_x_next = $signed({1'b0, centroid_x}) - $signed({1'b0, circle_x});
                    circle_grab_offset_y_next = $signed({1'b0, centroid_y}) - $signed({1'b0, circle_y});
                    circle_grabbed_next = 1'b1;
                end
            end else begin
                // Currently grabbed: check if we should release or continue dragging
                if (!gesture_fist) begin
                    // Fist released: release
                    circle_grabbed_next = 1'b0;
                end else if (!centroid_in_circle_tolerance) begin
                    // Centroid moved significantly outside tolerance zone: release
                    circle_grabbed_next = 1'b0;
                end else begin
                    // Still grabbed: move circle to maintain offset from grab point
                    new_circle_x_signed = $signed({1'b0, centroid_x}) - circle_grab_offset_x;
                    new_circle_y_signed = $signed({1'b0, centroid_y}) - circle_grab_offset_y;
                    
                    // Clamp to screen bounds
                    if (new_circle_x_signed < $signed({1'b0, CIRCLE_RADIUS})) begin
                        circle_x_next = CIRCLE_RADIUS;
                    end else if (new_circle_x_signed > $signed({1'b0, FRAME_WIDTH - CIRCLE_RADIUS})) begin
                        circle_x_next = FRAME_WIDTH - CIRCLE_RADIUS;
                    end else begin
                        circle_x_next = new_circle_x_signed[9:0];
                    end
                    
                    if (new_circle_y_signed < $signed({1'b0, CIRCLE_RADIUS})) begin
                        circle_y_next = CIRCLE_RADIUS;
                    end else if (new_circle_y_signed > $signed({1'b0, FRAME_HEIGHT - CIRCLE_RADIUS})) begin
                        circle_y_next = FRAME_HEIGHT - CIRCLE_RADIUS;
                    end else begin
                        circle_y_next = new_circle_y_signed[9:0];
                    end
                end
            end
        end else begin
            // Centroid invalid: release both
            if (!centroid_valid) begin
                if (square_grabbed) square_grabbed_next = 1'b0;
                if (circle_grabbed) circle_grabbed_next = 1'b0;
            end
        end
    end
    
    // Square and circle position register and grab state
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize square to center-left
            square_x <= FRAME_WIDTH / 4;
            square_y <= FRAME_HEIGHT / 2;
            // Initialize circle to center-right
            circle_x <= 3 * FRAME_WIDTH / 4;
            circle_y <= FRAME_HEIGHT / 2;
            square_grabbed <= 1'b0;
            circle_grabbed <= 1'b0;
            square_grab_offset_x <= 11'sd0;
            square_grab_offset_y <= 11'sd0;
            circle_grab_offset_x <= 11'sd0;
            circle_grab_offset_y <= 11'sd0;
            frame_start_prev <= 1'b0;
        end else begin
            frame_start_prev <= frame_start;
            
            // Update positions and grab states at frame start (rising edge)
            if (frame_start && !frame_start_prev && enable) begin
                // Update positions once per frame
                square_x <= square_x_next;
                square_y <= square_y_next;
                circle_x <= circle_x_next;
                circle_y <= circle_y_next;
                square_grabbed <= square_grabbed_next;
                circle_grabbed <= circle_grabbed_next;
                square_grab_offset_x <= square_grab_offset_x_next;
                square_grab_offset_y <= square_grab_offset_y_next;
                circle_grab_offset_x <= circle_grab_offset_x_next;
                circle_grab_offset_y <= circle_grab_offset_y_next;
            end
        end
    end
    
    // Square drawing logic
    // Draw square outline (border only, not filled)
    logic draw_square;
    logic in_square_horizontal_edge;
    logic in_square_vertical_edge;
    
    always_comb begin
        // Check if current pixel is on square border
        // Horizontal edges (top and bottom)
        in_square_horizontal_edge = ((draw_y == square_min_y) || (draw_y == square_max_y)) &&
                                     (draw_x >= square_min_x) && (draw_x <= square_max_x);
        
        // Vertical edges (left and right)
        in_square_vertical_edge = ((draw_x == square_min_x) || (draw_x == square_max_x)) &&
                                  (draw_y >= square_min_y) && (draw_y <= square_max_y);
        
        // Draw square if on any edge
        draw_square = enable && vde && (in_square_horizontal_edge || in_square_vertical_edge);
    end
    
    // Circle drawing logic
    // Draw circle outline (border only, not filled)
    logic draw_circle;
    logic signed [10:0] pixel_dx, pixel_dy;
    logic [20:0] pixel_dist_sq;
    logic [20:0] inner_radius_sq, outer_radius_sq;
    
    always_comb begin
        // Calculate distance from current pixel to circle center
        pixel_dx = $signed({1'b0, draw_x}) - $signed({1'b0, circle_x});
        pixel_dy = $signed({1'b0, draw_y}) - $signed({1'b0, circle_y});
        pixel_dist_sq = pixel_dx * pixel_dx + pixel_dy * pixel_dy;
        
        // Approximate distance (using integer square root approximation)
        // For circle border, check if pixel is near the radius
        // Simplified: check if distance squared is close to radius squared
        inner_radius_sq = (CIRCLE_RADIUS - 1) * (CIRCLE_RADIUS - 1);
        outer_radius_sq = (CIRCLE_RADIUS + 1) * (CIRCLE_RADIUS + 1);
        
        // Draw circle if pixel is on the border (within 1 pixel of radius)
        draw_circle = enable && vde && (pixel_dist_sq >= inner_radius_sq) && (pixel_dist_sq <= outer_radius_sq);
    end
    
    // Output mux: square/circle overlay or passthrough
    // Color selection: blue = default, green = hovered and ready to grab
    always_comb begin
        if (draw_square) begin
            // Draw square: green if hovered, blue otherwise
            if (square_hovered) begin
                pixel_red_out   = SHAPE_GREEN_RED;
                pixel_green_out = SHAPE_GREEN_GREEN;
                pixel_blue_out  = SHAPE_GREEN_BLUE;
            end else begin
                pixel_red_out   = SHAPE_BLUE_RED;
                pixel_green_out = SHAPE_BLUE_GREEN;
                pixel_blue_out  = SHAPE_BLUE_BLUE;
            end
        end else if (draw_circle) begin
            // Draw circle: green if hovered, blue otherwise
            if (circle_hovered) begin
                pixel_red_out   = SHAPE_GREEN_RED;
                pixel_green_out = SHAPE_GREEN_GREEN;
                pixel_blue_out  = SHAPE_GREEN_BLUE;
            end else begin
                pixel_red_out   = SHAPE_BLUE_RED;
                pixel_green_out = SHAPE_BLUE_GREEN;
                pixel_blue_out  = SHAPE_BLUE_BLUE;
            end
        end else begin
            // Passthrough original pixel
            pixel_red_out   = pixel_red_in;
            pixel_green_out = pixel_green_in;
            pixel_blue_out  = pixel_blue_in;
        end
    end

endmodule
