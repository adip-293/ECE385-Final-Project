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
    input  logic        split_centroid_enable,
    
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
    // Dual centroid (left/right) for split mode
    input  logic [9:0]  left_centroid_x,
    input  logic [9:0]  left_centroid_y,
    input  logic        left_centroid_valid,
    input  logic [9:0]  left_bbox_min_x,
    input  logic [9:0]  left_bbox_min_y,
    input  logic [9:0]  left_bbox_max_x,
    input  logic [9:0]  left_bbox_max_y,
    input  logic [9:0]  right_centroid_x,
    input  logic [9:0]  right_centroid_y,
    input  logic        right_centroid_valid,
    input  logic [9:0]  right_bbox_min_x,
    input  logic [9:0]  right_bbox_min_y,
    input  logic [9:0]  right_bbox_max_x,
    input  logic [9:0]  right_bbox_max_y,
    
    // Gesture flags
    input  logic        gesture_fist,
    input  logic        gesture_open,
    input  logic        gesture_wave,
    
    // Button inputs for pong reset
    input  logic [2:0]  btn_sync,
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
    logic [2:0] red_after_overlay, green_after_overlay, blue_after_overlay;
    // Bypass channel masking when override is active (force_color or force_grayscale)
    logic [2:0] red_after_select, green_after_select, blue_after_select;
    logic [2:0] red_pre_channel, green_pre_channel, blue_pre_channel;
    
    // Pong enable signal
    logic pong_enable;
    assign pong_enable = (current_state == 5'd21);  // PONG state
    
    // Rematch button edge detection for pong (btn[0] in PONG state)
    logic btn0_prev;
    logic pong_rematch_pressed;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn0_prev <= 1'b0;
        end else begin
            btn0_prev <= btn_sync[0];
        end
    end
    
    // Only pass btn[0] press to pong when in PONG state
    assign pong_rematch_pressed = pong_enable && btn_sync[0] && ~btn0_prev;

    // Map centroids to overlay inputs without primary/secondary semantics
    logic [9:0] ov1_cx, ov1_cy; logic ov1_valid;
    logic [9:0] ov1_bminx, ov1_bminy, ov1_bmaxx, ov1_bmaxy;
    logic [9:0] ov2_cx, ov2_cy; logic ov2_valid;
    logic [9:0] ov2_bminx, ov2_bminy, ov2_bmaxx, ov2_bmaxy;

    always_comb begin
        if (split_centroid_enable) begin
            // Dual mode: left on channel 1, right on channel 2
            ov1_cx = left_centroid_x;  ov1_cy = left_centroid_y;  ov1_valid = left_centroid_valid;
            ov1_bminx = left_bbox_min_x; ov1_bminy = left_bbox_min_y; ov1_bmaxx = left_bbox_max_x; ov1_bmaxy = left_bbox_max_y;
            ov2_cx = right_centroid_x; ov2_cy = right_centroid_y; ov2_valid = right_centroid_valid;
            ov2_bminx = right_bbox_min_x; ov2_bminy = right_bbox_min_y; ov2_bmaxx = right_bbox_max_x; ov2_bmaxy = right_bbox_max_y;
        end else begin
            // Single mode: use classic centroid on channel 1 only
            ov1_cx = centroid_x; ov1_cy = centroid_y; ov1_valid = centroid_valid;
            ov1_bminx = centroid_bbox_min_x; ov1_bminy = centroid_bbox_min_y; ov1_bmaxx = centroid_bbox_max_x; ov1_bmaxy = centroid_bbox_max_y;
            ov2_cx = 10'd0; ov2_cy = 10'd0; ov2_valid = 1'b0;
            ov2_bminx = 10'd0; ov2_bminy = 10'd0; ov2_bmaxx = 10'd0; ov2_bmaxy = 10'd0;
        end
    end

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
        
        // Pong score inputs (for large score display)
        .pong_player_a_score(pong_player_a_score),
        .pong_player_b_score(pong_player_b_score),
        .pong_player_a_wins(pong_player_a_wins),
        .pong_player_b_wins(pong_player_b_wins),
        
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
        .enable(overlay_enable && !pong_enable),  // Disable overlay when pong is active
        .bbox_enable(bbox_overlay_enable),
        .use_dual_centroids(split_centroid_enable),
        
        // Centroid coordinates
        .centroid_x(ov1_cx),
        .centroid_y(ov1_cy),
        .centroid_valid(ov1_valid),
        .centroid2_x(ov2_cx),
        .centroid2_y(ov2_cy),
        .centroid2_valid(ov2_valid),
        .bbox_min_x(ov1_bminx),
        .bbox_min_y(ov1_bminy),
        .bbox_max_x(ov1_bmaxx),
        .bbox_max_y(ov1_bmaxy),
        .bbox2_min_x(ov2_bminx),
        .bbox2_min_y(ov2_bminy),
        .bbox2_max_x(ov2_bmaxx),
        .bbox2_max_y(ov2_bmaxy),
        
        // VGA timing and input pixels (from text overlay)
        .draw_x(draw_x),
        .draw_y(draw_y),
        .vde(vde),
        .pixel_red_in(red_with_text),
        .pixel_green_in(green_with_text),
        .pixel_blue_in(blue_with_text),
        
        // Output pixels (with overlay)
        .pixel_red_out(red_after_overlay),
        .pixel_green_out(green_after_overlay),
        .pixel_blue_out(blue_after_overlay)
    );

    // ------------------------------------------------------------
    // Stage 4: Pong Game
    // ------------------------------------------------------------
    // Draws pong game when in PONG state
    logic [2:0] red_after_pong, green_after_pong, blue_after_pong;
    logic [3:0] pong_player_a_score;
    logic [3:0] pong_player_b_score;
    logic       pong_player_a_wins;
    logic       pong_player_b_wins;
    
    pong #(
        .FRAME_WIDTH(FRAME_WIDTH),
        .FRAME_HEIGHT(FRAME_HEIGHT)
    ) pong_game (
        .clk(clk),
        .rst_n(rst_n),
        .enable(pong_enable),
        
        // VGA timing
        .draw_x(draw_x),
        .draw_y(draw_y),
        .vde(vde),
        .frame_start(frame_start),
        
        // Reset/rematch button
        .rematch_pressed(pong_rematch_pressed),
        
        // Centroid inputs for paddle control
        .left_centroid_y(left_centroid_y),
        .left_centroid_valid(left_centroid_valid),
        .right_centroid_y(right_centroid_y),
        .right_centroid_valid(right_centroid_valid),
        
        // Centroid positions for red dot indicators
        .left_centroid_x(left_centroid_x),
        .right_centroid_x(right_centroid_x),
        
        // Input pixels (from overlay manager)
        .pixel_red_in(red_after_overlay),
        .pixel_green_in(green_after_overlay),
        .pixel_blue_in(blue_after_overlay),
        
        // Output pixels
        .pixel_red_out(red_after_pong),
        .pixel_green_out(green_after_pong),
        .pixel_blue_out(blue_after_pong),
        
        // Score outputs
        .player_a_score(pong_player_a_score),
        .player_b_score(pong_player_b_score),
        
        // Win status outputs
        .player_a_wins(pong_player_a_wins),
        .player_b_wins(pong_player_b_wins)
    );
    
    // Final output: pong when enabled, otherwise overlay manager output
    always_comb begin
        if (pong_enable) begin
            pixel_red_out = red_after_pong;
            pixel_green_out = green_after_pong;
            pixel_blue_out = blue_after_pong;
        end else begin
            pixel_red_out = red_after_overlay;
            pixel_green_out = green_after_overlay;
            pixel_blue_out = blue_after_overlay;
        end
    end

endmodule

