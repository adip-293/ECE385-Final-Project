`timescale 1ns / 1ps

/*
 * Postprocessor Module
 * 
 * Description:
 *   Post-processing pipeline between frame buffer and HDMI output.
 *   Handles channel selection, text overlay, and graphical overlays.
 * 
 * Purpose:
 *   Final stage before display: visual enhancements and debug information.
 *   Adds state labels, gesture text, crosshair, and bounding boxes.
 * 
 * Notes:
 *   - Data flow: Buffer -> Channel Selector -> Text Overlay -> Overlay Manager -> HDMI
 *   - Temporal filter: replicates averaged 3-bit value across RGB channels
 *   - Processing mode: replicates grayscale across RGB (from red channel [8:6])
 *   - Gesture text: latched per-frame to prevent flicker
 */

module postprocessor #(
    parameter FRAME_WIDTH         = 640,
    parameter FRAME_HEIGHT        = 480,
    parameter CROSSHAIR_LENGTH    = 20,
    parameter CROSSHAIR_THICKNESS = 2,
    parameter CENTER_GAP          = 5
)(
    // System
    input  logic        clk,
    input  logic        rst_n,
    
    // Control signals
    input  logic        channel_mode_enable,
    input  logic [2:0]  channel_select,
    input  logic        overlay_enable,
    input  logic        bbox_overlay_enable,
    input  logic        processing_enable,
    input  logic        temporal_filter_enable,
    input  logic [1:0]  frame_chunk_counter,
    input  logic        force_color,
    input  logic        force_grayscale,
    input  logic [4:0]  current_state,
    input  logic        gesture_enable,
    
    // VGA timing
    input  logic [9:0]  draw_x,
    input  logic [9:0]  draw_y,
    input  logic        vde,
    
    // Centroid for overlay
    input  logic [9:0]  centroid_x,
    input  logic [9:0]  centroid_y,
    input  logic        centroid_valid,
    input  logic [9:0]  centroid_bbox_min_x,
    input  logic [9:0]  centroid_bbox_min_y,
    input  logic [9:0]  centroid_bbox_max_x,
    input  logic [9:0]  centroid_bbox_max_y,
    
    // Gesture flags
    input  logic        gesture_fist,
    input  logic        gesture_open,
    input  logic        gesture_wave,
    //input switches
    
    // Input pixels from buffer
    input  logic [2:0]  pixel_red_in,
    input  logic [2:0]  pixel_green_in,
    input  logic [2:0]  pixel_blue_in,
    
    // Output pixels to HDMI
    output logic [2:0]  pixel_red_out,
    output logic [2:0]  pixel_green_out,
    output logic [2:0]  pixel_blue_out
);

    // Internal signals between channel selector and overlay manager
    logic [2:0] red_selected, green_selected, blue_selected;
    logic [2:0] red_with_text, green_with_text, blue_with_text;
    // Bypass channel masking when override is active (force_color or force_grayscale)
    logic [2:0] red_after_select, green_after_select, blue_after_select;
    logic [2:0] red_pre_channel, green_pre_channel, blue_pre_channel;

    // ------------------------------------------------------------
    // Gesture frame latch (select one gesture per frame to avoid flicker)
    // ------------------------------------------------------------
    // 0=none,1=FIST,2=OPEN,3=WAVE
        logic [1:0] gesture_code_lat; // Gesture frame latch: select one gesture per video frame to avoid flicker
        // Track start-of-frame using draw_x/draw_y
        logic       in_frame;
        logic       frame_start;
    //color
    

    // Simple frame boundary detection and per-frame latch
    // frame_start pulses when draw_x==0 && draw_y==0 (top-left of frame)
    always_comb begin
        in_frame    = vde; // active video region
        frame_start = (draw_x == 10'd0) && (draw_y == 10'd0);
    end

        // Latch one gesture per frame with priority: WAVE > FIST > OPEN
        // Reset at frame start and hold selected gesture until next frame
        always_ff @(posedge clk or negedge rst_n) begin
            if (!rst_n) begin
                gesture_code_lat <= 2'd0;
            end else begin
                if (frame_start) begin
                    // start of a new frame, clear previous latch
                    gesture_code_lat <= 2'd0;
                end else begin
                    // Only update latch if nothing selected yet this frame
                    if (gesture_code_lat == 2'd0) begin
                        if (gesture_enable && gesture_wave) begin
                            gesture_code_lat <= 2'd3;
                        end else if (gesture_enable && gesture_fist) begin
                            gesture_code_lat <= 2'd1;
                        end else if (gesture_enable && gesture_open) begin
                            gesture_code_lat <= 2'd2;
                        end
                    end
                end
            end
        end
    
    // ------------------------------------------------------------
    // Stage 0: Temporal Filter and Grayscale Replication
    // ------------------------------------------------------------
    // When in temporal filter mode: extract all 3 chunks and compute moving average
    // When in normal processing mode: replicate 3-bit value across RGB channels
    logic [2:0] temporal_red, temporal_green, temporal_blue;  // Temporal filter heatmap outputs
    
    // Temporal filter instance
    temporal_filter temp_filter (
        .pixel_red_in(pixel_red_in),
        .pixel_green_in(pixel_green_in),
        .pixel_blue_in(pixel_blue_in),
        .frame_chunk_counter(frame_chunk_counter),
        .red_out(temporal_red),
        .green_out(temporal_green),
        .blue_out(temporal_blue)
    );
    
    always_comb begin
        if ((processing_enable || force_grayscale) && temporal_filter_enable) begin
            // Temporal filter mode: use heatmap RGB outputs
            red_pre_channel   = temporal_red;
            green_pre_channel = temporal_green;
            blue_pre_channel  = temporal_blue;
        end else if (processing_enable || force_grayscale) begin
            // Normal processing mode or grayscale override: replicate the 3-bit value from red channel across all channels
            red_pre_channel   = pixel_red_in;
            green_pre_channel = pixel_red_in;  // Replicate from stored position
            blue_pre_channel  = pixel_red_in;  // Replicate from stored position
        end else begin
            // Color mode: use all channels normally
            red_pre_channel   = pixel_red_in;
            green_pre_channel = pixel_green_in;
            blue_pre_channel  = pixel_blue_in;
        end
    end
    
    // ------------------------------------------------------------
    // Stage 1: Channel Selector
    // ------------------------------------------------------------
    // Selectively displays RGB color channels based on control signals
    channel_selector chan_sel (
        .enable(channel_mode_enable),
        .channel_select(channel_select),
        .red_in(red_pre_channel),
        .green_in(green_pre_channel),
        .blue_in(blue_pre_channel),
        .red_out(red_selected),
        .green_out(green_selected),
        .blue_out(blue_selected)
    );

    // If either override is active, bypass channel selection masks
    always_comb begin
        if (force_color || force_grayscale) begin
            red_after_select   = red_pre_channel;
            green_after_select = green_pre_channel;
            blue_after_select  = blue_pre_channel;
        end else begin
            red_after_select   = red_selected;
            green_after_select = green_selected;
            blue_after_select  = blue_selected;
        end
    end
    
    // ------------------------------------------------------------
    // Stage 2: Text Overlay
    // ------------------------------------------------------------
    // Displays current state name at bottom of screen
    text_overlay txt_overlay (
        .clk(clk),
        .rst_n(rst_n),
        .current_state(current_state),
        .force_color(force_color),
        .force_grayscale(force_grayscale),
        .gesture_code(gesture_code_lat),
        
        // VGA timing
        .draw_x(draw_x),
        .draw_y(draw_y),
        .vde(vde),
        
        // Input pixels (from channel selector)
        .pixel_red_in(red_after_select),
        .pixel_green_in(green_after_select),
        .pixel_blue_in(blue_after_select),
        
        // Output pixels (with text overlay)
        .pixel_red_out(red_with_text),
        .pixel_green_out(green_with_text),
        .pixel_blue_out(blue_with_text)
    );

    // Gesture text already integrated in main text overlay; pass-through
    
    // ------------------------------------------------------------
    // Stage 3: Overlay Manager
    // ------------------------------------------------------------
    // Draws graphical overlays (crosshair, bounding boxes, etc.)
    overlay_manager #(
        .FRAME_WIDTH(FRAME_WIDTH),
        .FRAME_HEIGHT(FRAME_HEIGHT),
        .CROSSHAIR_LENGTH(CROSSHAIR_LENGTH),
        .CROSSHAIR_THICKNESS(CROSSHAIR_THICKNESS),
        .CENTER_GAP(CENTER_GAP)
    ) overlay_mgr (
        .clk(clk),
        .rst_n(rst_n),
        .enable(overlay_enable),
        .bbox_enable(bbox_overlay_enable),
        
        // Centroid coordinates
        .centroid_x(centroid_x),
        .centroid_y(centroid_y),
        .centroid_valid(centroid_valid),
        .bbox_min_x(centroid_bbox_min_x),
        .bbox_min_y(centroid_bbox_min_y),
        .bbox_max_x(centroid_bbox_max_x),
        .bbox_max_y(centroid_bbox_max_y),
        
        // VGA timing and input pixels (from text overlay)
        .draw_x(draw_x),
        .draw_y(draw_y),
        .vde(vde),
        .pixel_red_in(red_with_text),
        .pixel_green_in(green_with_text),
        .pixel_blue_in(blue_with_text),
        
        // Output pixels (with overlay)
        .pixel_red_out(pixel_red_out),
        .pixel_green_out(pixel_green_out),
        .pixel_blue_out(pixel_blue_out)
    );

endmodule

