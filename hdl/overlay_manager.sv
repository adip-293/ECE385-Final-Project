`timescale 1ns / 1ps

/*
 * Overlay Manager Module
 * 
 * Description:
 *   Draws graphical overlays (crosshair, bounding box) on video output.
 *   Crosshair centered at detected centroid position.
 * 
 * Purpose:
 *   Visual feedback for centroid tracking and blob detection.
 *   Helps debug and visualize tracking performance.
 * 
 * Notes:
 *   - Crosshair: vertical/horizontal lines through centroid with center gap
 *   - Bounding box: rectangle along min/max x/y coordinates
 *   - Overlay color: bright green for visibility
 */

module overlay_manager #(
    parameter FRAME_WIDTH     = 640,
    parameter FRAME_HEIGHT    = 480,
    parameter CROSSHAIR_LENGTH = 20,
    parameter CROSSHAIR_THICKNESS = 2,
    parameter CENTER_GAP      = 5
)(
    // System
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,
    input  logic        bbox_enable,
    
    // Centroid coordinates
    input  logic [9:0]  centroid_x,
    input  logic [9:0]  centroid_y,
    input  logic        centroid_valid,
    
    // Bounding box coordinates
    input  logic [9:0]  bbox_min_x,
    input  logic [9:0]  bbox_min_y,
    input  logic [9:0]  bbox_max_x,
    input  logic [9:0]  bbox_max_y,
    
    // VGA timing
    input  logic [9:0]  draw_x,
    input  logic [9:0]  draw_y,
    input  logic        vde,
    
    // Input pixels
    input  logic [2:0]  pixel_red_in,
    input  logic [2:0]  pixel_green_in,
    input  logic [2:0]  pixel_blue_in,
    
    // Output pixels (with overlay)
    output logic [2:0]  pixel_red_out,
    output logic [2:0]  pixel_green_out,
    output logic [2:0]  pixel_blue_out
);

    // Crosshair color (bright green for visibility)
    localparam logic [2:0] CROSSHAIR_RED   = 3'b111;
    localparam logic [2:0] CROSSHAIR_GREEN = 3'b000;
    localparam logic [2:0] CROSSHAIR_BLUE  = 3'b000;
    
    // Crosshair drawing logic
    logic draw_crosshair;
    logic draw_vertical_line;
    logic draw_horizontal_line;
    // Bounding box drawing logic
    logic draw_bbox;
    logic in_bbox_edge;
    logic in_bbox_vertical_edge;
    logic in_bbox_horizontal_edge;
    
    // Check if current pixel is on vertical line of crosshair
    logic [9:0] x_diff, y_diff;
    logic in_vertical_range, in_horizontal_range;
    logic in_vertical_thickness, in_horizontal_thickness;
    logic outside_center_gap_vertical, outside_center_gap_horizontal;
    
    always_comb begin
        // Calculate absolute differences
        x_diff = (draw_x >= centroid_x) ? (draw_x - centroid_x) : (centroid_x - draw_x);
        y_diff = (draw_y >= centroid_y) ? (draw_y - centroid_y) : (centroid_y - draw_y);
        
        // Vertical line: x matches centroid_x (±thickness), y within range
        in_vertical_thickness = (x_diff < CROSSHAIR_THICKNESS[9:0]);
        in_vertical_range = (y_diff <= CROSSHAIR_LENGTH[9:0]) && (y_diff >= CENTER_GAP[9:0]);
        draw_vertical_line = in_vertical_thickness && in_vertical_range;
        
        // Horizontal line: y matches centroid_y (±thickness), x within range
        in_horizontal_thickness = (y_diff < CROSSHAIR_THICKNESS[9:0]);
        in_horizontal_range = (x_diff <= CROSSHAIR_LENGTH[9:0]) && (x_diff >= CENTER_GAP[9:0]);
        draw_horizontal_line = in_horizontal_thickness && in_horizontal_range;
        
        // Combine: draw crosshair if on either line
        draw_crosshair = enable && centroid_valid && vde && 
                        (draw_vertical_line || draw_horizontal_line);

        // Bounding box edges: draw rectangle along min/max x/y
        // Vertical edges confined to bbox Y range
        in_bbox_vertical_edge   = ((draw_x == bbox_min_x) || (draw_x == bbox_max_x)) &&
                      (draw_y >= bbox_min_y) && (draw_y <= bbox_max_y);
        // Horizontal edges confined to bbox X range
        in_bbox_horizontal_edge = ((draw_y == bbox_min_y) || (draw_y == bbox_max_y)) &&
                      (draw_x >= bbox_min_x) && (draw_x <= bbox_max_x);
        in_bbox_edge = in_bbox_vertical_edge || in_bbox_horizontal_edge;
        draw_bbox = bbox_enable && centroid_valid && vde && in_bbox_edge;
    end
    
    // Output mux: overlay or passthrough
    always_comb begin
        if (draw_crosshair || draw_bbox) begin
            // Draw crosshair in bright green
            pixel_red_out   = CROSSHAIR_RED;
            pixel_green_out = CROSSHAIR_GREEN;
            pixel_blue_out  = CROSSHAIR_BLUE;
        end else begin
            // Passthrough original pixel
            pixel_red_out   = pixel_red_in;
            pixel_green_out = pixel_green_in;
            pixel_blue_out  = pixel_blue_in;
        end
    end

endmodule
