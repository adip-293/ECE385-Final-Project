`timescale 1ns / 1ps

/*
 * Gesture Detector Module
 * 
 * Description:
 *   Heuristic-based gesture detection using centroid and blob statistics.
 *   Classifies hand gestures: fist, open hand, wave.
 * 
 * Purpose:
 *   Real-time hand gesture recognition for interactive control.
 *   Uses compactness (blob_area/bbox_area) as primary feature.
 * 
 * Notes:
 *   - Fist: compactness > 0.55 (compact filled blob)
 *   - Open hand: compactness <= 0.55 (fingers spread, less filled)
 *   - Wave: lateral centroid oscillation with direction changes
 *   - Hysteresis prevents flicker (requires 2 consecutive frames to assert)
 */

module gesture_detector (
    // System
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,
    input  logic        frame_start,

    // Blob statistics
    input  logic        centroid_valid,
    input  logic [9:0]  centroid_x,
    input  logic [9:0]  centroid_y,
    input  logic [9:0]  bbox_min_x,
    input  logic [9:0]  bbox_min_y,
    input  logic [9:0]  bbox_max_x,
    input  logic [9:0]  bbox_max_y,
    input  logic [19:0] blob_area,

    // Gesture outputs
    output logic        fist,
    output logic        open_hand,
    output logic        wave
);

    // Parameters for heuristics (tune as needed)
    // SIMPLIFIED: Use ONLY compactness (extent) for fist/palm classification
    // Based on empirical analysis showing compactness is the most reliable feature
    localparam int unsigned MIN_AREA                = 200;   // lower noise floor to accept far hands
    localparam int unsigned MIN_BBOX_W              = 6;     // minimum bbox dimensions to reject speckles
    localparam int unsigned MIN_BBOX_H              = 6;

    // Compactness threshold (from notebook analysis): Use only this for fist vs palm
    // Fist: compactness > threshold (more filled)
    // Palm: compactness <= threshold (less filled, fingers spread)
    localparam int unsigned COMPACTNESS_THRESHOLD_Q8_8 = 141;   // ~0.55 (tuned from real data)

    // Wave detection parameters
    // Detect lateral oscillation of centroid_x with direction changes and high velocity
    localparam int unsigned WAVE_MIN_VELOCITY     = 3;   // pixels/frame required (robust against noise)
    localparam int unsigned WAVE_MIN_DIRECTION_CHANGES = 3;  // direction changes needed (more robust)
    localparam int unsigned WAVE_HYST_ON_FRAMES   = 1;   // quick response
    localparam int unsigned WAVE_HYST_OFF_FRAMES  = 4;   // shorter hold after wave ends

    // Output hysteresis for fist/open (frames to assert/deassert)
    localparam int HYST_ON_FRAMES  = 2;  // require 2 valid samples to assert
    localparam int HYST_OFF_FRAMES = 5;  // require 5 invalid samples to clear

    // -------------------------------------------------------------------------
    // Bbox width/height and area
    // -------------------------------------------------------------------------
    logic [10:0] bbox_width, bbox_height; // allow +1

    always_comb begin
        bbox_width  = (centroid_valid && bbox_max_x >= bbox_min_x) ? (bbox_max_x - bbox_min_x + 11'd1) : 11'd0;
        bbox_height = (centroid_valid && bbox_max_y >= bbox_min_y) ? (bbox_max_y - bbox_min_y + 11'd1) : 11'd0;
    end

    // Bbox area
    logic [21:0] bbox_area; // 11x11 -> up to 22 bits
    always_comb begin
        bbox_area = bbox_width * bbox_height;
    end

    // Q8.8 scaled width/height
    logic [18:0] width_q8_8, height_q8_8; // 11+8 = 19 bits
    always_comb begin
        width_q8_8  = {bbox_width,  8'd0};
        height_q8_8 = {bbox_height, 8'd0};
    end

    // -------------------------------------------------------------------------
    // Fist/Palm detection: Use ONLY compactness (blob_area / bbox_area)
    // Simple, reliable classification based on how filled the bounding box is
    // -------------------------------------------------------------------------
    logic [30:0] area_q8_8;
    assign area_q8_8 = {3'd0, blob_area, 8'd0}; // widen to 31 bits (Q8.8)

    logic [30:0] compactness_threshold;
    logic        fist_detect;
    logic        open_detect;

    always_comb begin
        // Compactness: blob_area / bbox_area
        // Compare: area_q8_8 vs COMPACTNESS_THRESHOLD_Q8_8 * bbox_area
        compactness_threshold = COMPACTNESS_THRESHOLD_Q8_8 * bbox_area;

        // Fist: compactness > threshold (more compact, filled blob)
        fist_detect = centroid_valid &&
                      (blob_area   >= MIN_AREA)   &&
                      (bbox_width  >= MIN_BBOX_W) &&
                      (bbox_height >= MIN_BBOX_H) &&
                      (bbox_area > 22'd0) &&
                      (area_q8_8 > compactness_threshold);

        // Palm: compactness <= threshold (less compact, fingers spread)
        open_detect = centroid_valid &&
                      (blob_area   >= MIN_AREA)   &&
                      (bbox_width  >= MIN_BBOX_W) &&
                      (bbox_height >= MIN_BBOX_H) &&
                      (bbox_area > 22'd0) &&
                      (area_q8_8 <= compactness_threshold);
    end

    // Wave detection signals
    logic [9:0]  prev_x;
    logic signed [10:0] dx_current;   // current frame delta (combinational)
    logic signed [10:0] abs_velocity; // absolute velocity
    logic        current_dir;         // current direction: 0=left, 1=right
    logic        prev_dir;            // previous direction
    logic        prev_dir_valid;      // whether prev_dir has been initialized
    logic [2:0]  wave_decay_timer;    // slow decay timer when velocity is low or centroid invalid
    logic        high_velocity;       // velocity exceeds threshold
    logic [3:0]  direction_change_count;  // count consecutive direction changes
    logic [3:0]  wave_on_count, wave_off_count;
    logic        wave_state;

    // -------------------------------------------------------------------------
    // Hysteresis counters and latched states for outputs
    // -------------------------------------------------------------------------
    logic [3:0] fist_on_count, fist_off_count;
    logic [3:0] open_on_count, open_off_count;
    logic       fist_state, open_state;

    // -------------------------------------------------------------------------
    // Main sequential block
    // -------------------------------------------------------------------------
    // Sample frame_start to align updates once per frame
    logic frame_start_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_start_d <= 1'b0;
        else frame_start_d <= frame_start;
    end

    integer i;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Initialize wave detection
            prev_x                 <= 10'd0;
            prev_dir               <= 1'b0;
            prev_dir_valid         <= 1'b0;
            direction_change_count <= 4'd0;
            wave_on_count          <= 4'd0;
            wave_off_count         <= 4'd0;
            wave_state             <= 1'b0;
            wave_decay_timer       <= 3'd0;

            fist_on_count       <= 4'd0;
            fist_off_count      <= 4'd0;
            fist_state          <= 1'b0;

            open_on_count       <= 4'd0;
            open_off_count      <= 4'd0;
            open_state          <= 1'b0;

            // No wave state
        end else if (frame_start_d) begin
            if (centroid_valid) begin
                // ---------------------------
                // Wave detection update
                // ---------------------------
                // Calculate current frame delta and velocity
                dx_current = $signed({1'b0, centroid_x}) - $signed({1'b0, prev_x});
                abs_velocity = (dx_current < 0) ? -dx_current : dx_current;
                high_velocity = (abs_velocity >= $signed(WAVE_MIN_VELOCITY));
                
                // Determine current direction (0=left/negative, 1=right/positive)
                current_dir = (dx_current >= 0) ? 1'b1 : 1'b0;
                
                // Track direction changes: increment when direction flips with high velocity
                if (high_velocity) begin
                    // Initialize prev_dir on first valid sample to avoid spurious change
                    if (!prev_dir_valid) begin
                        prev_dir <= current_dir;
                        prev_dir_valid <= 1'b1;
                    end
                    if (prev_dir_valid && (current_dir != prev_dir)) begin
                        // Direction changed!
                        direction_change_count <= (direction_change_count < 4'd15) ? 
                                                   (direction_change_count + 4'd1) : 4'd15;
                    end
                    prev_dir <= current_dir;
                    wave_decay_timer <= 3'd0; // reset decay on movement
                end else begin
                    // Low velocity: decay direction change count slowly with a timer
                    if (wave_decay_timer < 3'd7) begin
                        wave_decay_timer <= wave_decay_timer + 3'd1;
                    end else begin
                        wave_decay_timer <= 3'd0;
                        if (direction_change_count > 4'd0) begin
                            direction_change_count <= direction_change_count - 4'd1;
                        end
                    end
                end
                
                // Hysteresis for wave state based on direction change count
                if (direction_change_count >= WAVE_MIN_DIRECTION_CHANGES[3:0]) begin
                    wave_off_count <= 4'd0;
                    if (wave_on_count < WAVE_HYST_ON_FRAMES[3:0]) begin
                        wave_on_count <= wave_on_count + 4'd1;
                        if (wave_on_count + 4'd1 >= WAVE_HYST_ON_FRAMES[3:0])
                            wave_state <= 1'b1;
                    end else begin
                        wave_state <= 1'b1;
                    end
                end else begin
                    wave_on_count <= 4'd0;
                    if (wave_off_count < WAVE_HYST_OFF_FRAMES[3:0]) begin
                        wave_off_count <= wave_off_count + 4'd1;
                        if (wave_off_count + 4'd1 >= WAVE_HYST_OFF_FRAMES[3:0])
                            wave_state <= 1'b0;
                    end else begin
                        wave_state <= 1'b0;
                    end
                end
                
                // Update previous position
                prev_x <= centroid_x;

                // ---------------------------
                // Hysteresis update for fist
                // ---------------------------
                if (fist_detect) begin
                    fist_off_count <= 4'd0;
                    if (fist_on_count < HYST_ON_FRAMES) begin
                        fist_on_count <= fist_on_count + 4'd1;
                        if (fist_on_count + 4'd1 >= HYST_ON_FRAMES)
                            fist_state <= 1'b1;
                    end else begin
                        fist_state <= 1'b1;
                    end
                end else begin
                    fist_on_count <= 4'd0;
                    if (fist_off_count < HYST_OFF_FRAMES) begin
                        fist_off_count <= fist_off_count + 4'd1;
                        if (fist_off_count + 4'd1 >= HYST_OFF_FRAMES)
                            fist_state <= 1'b0;
                    end else begin
                        fist_state <= 1'b0;
                    end
                end

                // ------------------------------
                // Hysteresis update for open hand
                // ------------------------------
                if (open_detect) begin
                    open_off_count <= 4'd0;
                    if (open_on_count < HYST_ON_FRAMES) begin
                        open_on_count <= open_on_count + 4'd1;
                        if (open_on_count + 4'd1 >= HYST_ON_FRAMES)
                            open_state <= 1'b1;
                    end else begin
                        open_state <= 1'b1;
                    end
                end else begin
                    open_on_count <= 4'd0;
                    if (open_off_count < HYST_OFF_FRAMES) begin
                        open_off_count <= open_off_count + 4'd1;
                        if (open_off_count + 4'd1 >= HYST_OFF_FRAMES)
                            open_state <= 1'b0;
                    end else begin
                        open_state <= 1'b0;
                    end
                end

                // Wave detection removed
            end else begin
                // No valid centroid: decay with timer and reset direction initialization
                prev_dir_valid <= 1'b0;
                if (wave_decay_timer < 3'd7) begin
                    wave_decay_timer <= wave_decay_timer + 3'd1;
                end else begin
                    wave_decay_timer <= 3'd0;
                    if (direction_change_count > 4'd0) begin
                        direction_change_count <= direction_change_count - 4'd1;
                    end
                end
                if (wave_off_count < WAVE_HYST_OFF_FRAMES[3:0]) begin
                    wave_off_count <= wave_off_count + 4'd1;
                    if (wave_off_count + 4'd1 >= WAVE_HYST_OFF_FRAMES[3:0])
                        wave_state <= 1'b0;
                end else begin
                    wave_state <= 1'b0;
                end
                wave_on_count <= 4'd0;

                // decay all gesture states when no valid centroid
                if (fist_off_count < HYST_OFF_FRAMES) begin
                    fist_off_count <= fist_off_count + 4'd1;
                    if (fist_off_count + 4'd1 >= HYST_OFF_FRAMES)
                        fist_state <= 1'b0;
                end else begin
                    fist_state <= 1'b0;
                end
                fist_on_count <= 4'd0;

                if (open_off_count < HYST_OFF_FRAMES) begin
                    open_off_count <= open_off_count + 4'd1;
                    if (open_off_count + 4'd1 >= HYST_OFF_FRAMES)
                        open_state <= 1'b0;
                end else begin
                    open_state <= 1'b0;
                end
                open_on_count <= 4'd0;

                // No wave state
            end
        end
    end

    // -------------------------------------------------------------------------
    // One-hot outputs gated by enable (priority: wave > fist > open)
    // Wave is independent of fist/palm state - can wave with either hand shape
    // -------------------------------------------------------------------------
    always_comb begin
        if (enable) begin
            // Wave from detection
            wave = wave_state;
            
            // Fist and palm are mutually exclusive (simple compactness threshold)
            if (fist_state) begin
                fist      = 1'b1;
                open_hand = 1'b0;
            end else if (open_state) begin
                fist      = 1'b0;
                open_hand = 1'b1;
            end else begin
                fist      = 1'b0;
                open_hand = 1'b0;
            end
        end else begin
            wave      = 1'b0;
            fist      = 1'b0;
            open_hand = 1'b0;
        end
    end

endmodule
