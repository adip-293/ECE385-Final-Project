`timescale 1ns / 1ps

/*
 * Gesture Perceptron Module
 * 
 * Description:
 *   Adaptive perceptron-based gesture classifier that learns from user feedback.
 *   Uses blob statistics (area, velocity, compactness, aspect ratio) as features.
 * 
 * Purpose:
 *   Real-time adaptive gesture recognition with online learning.
 *   Classifies gestures: fist, open hand, wave using learned weights.
 * 
 * Usage:
 *   NORMAL MODE (sw[12]=0):
 *   - Buttons cycle through demo states as usual
 *   - Perceptron classifies gestures automatically
 *   
 *   LEARNING MODE (sw[12]=1):
 *   - Buttons become teaching controls:
 *     • BTN[0]: Teach "this is a FIST" (show fist, press btn[0])
 *     • BTN[1]: Teach "this is OPEN HAND" (show open hand, press btn[1])
 *     • BTN[2]: Teach "this is WAVE" (wave, press btn[2])
 *   - Perform gesture clearly, then press corresponding button
 *   - Release button before changing gesture (edge-triggered)
 *   - Learning persists until reset
 *   
 *   To reset weights: Press hardware reset button
 * 
 * Tips:
 *   - Start with clear, exaggerated gestures for initial training
 *   - If classifier is wrong, use switches to correct it immediately
 *   - Wave requires actual motion - move hand left/right rhythmically
 *   - Fist should be compact (fingers curled in)
 *   - Open hand should have fingers spread (lower compactness)
 * 
 * Learning:
 *   - Uses simple perceptron learning rule: w = w + α * error * feature
 *   - Feedback via switches: sw[12:10] for fist/open/wave correction
 *   - Learning rate adjustable, weights persist across frames
 * 
 * Features:
 *   1. Compactness: blob_area / bbox_area (Q8.8)
 *   2. Velocity: abs(dx) per frame
 *   3. Area: blob_area normalized
 *   4. Aspect ratio: width / height
 *   5. Direction changes: oscillation detection
 */

module gesture_perceptron (
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
    
    // Learning mode control
    input  logic        learning_mode,      // sw[12]: Enable learning mode
    input  logic [2:0]  learning_buttons,   // btn[2:0] in learning mode
    // When learning_mode=1:
    //   btn[0]: Teach this as FIST (reinforce fist weights)
    //   btn[1]: Teach this as OPEN HAND (reinforce open weights)
    //   btn[2]: Teach this as WAVE (reinforce wave weights)

    // Gesture outputs
    output logic        fist,
    output logic        open_hand,
    output logic        wave,
    
    // Debug outputs (for LEDs/hex)
    output logic [7:0]  debug_activation_fist,
    output logic [7:0]  debug_activation_open,
    output logic [7:0]  debug_activation_wave
);

    // -------------------------------------------------------------------------
    // Parameters
    // -------------------------------------------------------------------------
    localparam int LEARNING_RATE = 2;  // Q4.4 format: 2 = 0.125
    localparam int MIN_AREA = 200;
    localparam int MIN_BBOX_W = 6;
    localparam int MIN_BBOX_H = 6;
    
    // Number of features
    localparam int NUM_FEATURES = 5;
    
    // Activation threshold (Q8.8 format)
    localparam int ACTIVATION_THRESHOLD = 128;  // 0.5 in Q8.8
    
    // -------------------------------------------------------------------------
    // Feature Extraction
    // -------------------------------------------------------------------------
    logic [10:0] bbox_width, bbox_height;
    logic [21:0] bbox_area;
    logic [9:0]  prev_x;
    logic signed [10:0] velocity;
    logic [9:0]  abs_velocity;
    logic [2:0]  direction_changes;
    
    // Compute bbox dimensions
    always_comb begin
        bbox_width  = (centroid_valid && bbox_max_x >= bbox_min_x) ? 
                      (bbox_max_x - bbox_min_x + 11'd1) : 11'd0;
        bbox_height = (centroid_valid && bbox_max_y >= bbox_min_y) ? 
                      (bbox_max_y - bbox_min_y + 11'd1) : 11'd0;
        bbox_area   = bbox_width * bbox_height;
    end
    
    // Feature 1: Compactness (Q8.8) - blob_area / bbox_area
    logic [7:0] feature_compactness;
    always_comb begin
        if (bbox_area > 0 && centroid_valid) begin
            // Compute (blob_area << 8) / bbox_area to get Q8.8
            // Simplified: use upper bits to approximate
            feature_compactness = (blob_area[19:12] * 8'd255) / bbox_area[21:14];
        end else begin
            feature_compactness = 8'd0;
        end
    end
    
    // Feature 2: Velocity (pixels per frame)
    always_comb begin
        abs_velocity = (velocity < 0) ? -velocity[9:0] : velocity[9:0];
    end
    logic [7:0] feature_velocity;
    assign feature_velocity = abs_velocity[7:0];  // Cap at 255
    
    // Feature 3: Normalized area (0-255 scale)
    logic [7:0] feature_area;
    always_comb begin
        if (blob_area > 20'd5000) begin
            feature_area = 8'd255;  // Max
        end else begin
            feature_area = blob_area[12:5];  // Divide by 32 for rough normalization
        end
    end
    
    // Feature 4: Aspect ratio (width/height, Q8.8)
    logic [7:0] feature_aspect;
    always_comb begin
        if (bbox_height > 0) begin
            // Approximation: (width << 8) / height
            if (bbox_width > bbox_height) begin
                feature_aspect = 8'd200;  // Wide
            end else if (bbox_width < bbox_height) begin
                feature_aspect = 8'd50;   // Tall
            end else begin
                feature_aspect = 8'd128;  // Square
            end
        end else begin
            feature_aspect = 8'd128;
        end
    end
    
    // Feature 5: Direction changes (oscillation indicator)
    logic [7:0] feature_oscillation;
    assign feature_oscillation = {5'd0, direction_changes} << 4;  // Scale up
    
    // -------------------------------------------------------------------------
    // Perceptron Weights (signed Q8.8 format)
    // Three perceptrons: one for each gesture class
    // -------------------------------------------------------------------------
    logic signed [8:0] weights_fist [0:NUM_FEATURES-1];
    logic signed [8:0] weights_open [0:NUM_FEATURES-1];
    logic signed [8:0] weights_wave [0:NUM_FEATURES-1];
    logic signed [8:0] bias_fist, bias_open, bias_wave;
    
    // Initialize weights with heuristic values
    initial begin
        // Fist: high compactness, low velocity, medium area
        weights_fist[0] = 9'sd100;   // Compactness (positive correlation)
        weights_fist[1] = -9'sd20;   // Velocity (negative correlation)
        weights_fist[2] = 9'sd30;    // Area
        weights_fist[3] = 9'sd10;    // Aspect ratio
        weights_fist[4] = -9'sd50;   // Oscillation (negative)
        bias_fist = -9'sd50;
        
        // Open hand: low compactness, low velocity, large area
        weights_open[0] = -9'sd80;   // Compactness (negative)
        weights_open[1] = -9'sd20;   // Velocity
        weights_open[2] = 9'sd50;    // Area (positive)
        weights_open[3] = 9'sd20;    // Aspect ratio
        weights_open[4] = -9'sd50;   // Oscillation (negative)
        bias_open = -9'sd30;
        
        // Wave: medium compactness, high velocity, high oscillation
        weights_wave[0] = 9'sd20;    // Compactness
        weights_wave[1] = 9'sd100;   // Velocity (high positive)
        weights_wave[2] = 9'sd20;    // Area
        weights_wave[3] = 9'sd10;    // Aspect ratio
        weights_wave[4] = 9'sd120;   // Oscillation (high positive)
        bias_wave = -9'sd60;
    end
    
    // -------------------------------------------------------------------------
    // Feature Vector (all normalized to 0-255)
    // -------------------------------------------------------------------------
    logic [7:0] features [0:NUM_FEATURES-1];
    always_comb begin
        features[0] = feature_compactness;
        features[1] = feature_velocity;
        features[2] = feature_area;
        features[3] = feature_aspect;
        features[4] = feature_oscillation;
    end
    
    // -------------------------------------------------------------------------
    // Activation Function (Weighted Sum)
    // -------------------------------------------------------------------------
    logic signed [15:0] activation_fist, activation_open, activation_wave;
    
    always_comb begin
        activation_fist = bias_fist;
        activation_open = bias_open;
        activation_wave = bias_wave;
        
        for (int i = 0; i < NUM_FEATURES; i++) begin
            activation_fist = activation_fist + (weights_fist[i] * $signed({1'b0, features[i]})) / 16;
            activation_open = activation_open + (weights_open[i] * $signed({1'b0, features[i]})) / 16;
            activation_wave = activation_wave + (weights_wave[i] * $signed({1'b0, features[i]})) / 16;
        end
    end
    
    // Debug outputs (saturate to 8-bit unsigned)
    assign debug_activation_fist = (activation_fist < 0) ? 8'd0 : 
                                   (activation_fist > 255) ? 8'd255 : activation_fist[7:0];
    assign debug_activation_open = (activation_open < 0) ? 8'd0 : 
                                   (activation_open > 255) ? 8'd255 : activation_open[7:0];
    assign debug_activation_wave = (activation_wave < 0) ? 8'd0 : 
                                   (activation_wave > 255) ? 8'd255 : activation_wave[7:0];
    
    // -------------------------------------------------------------------------
    // Classification (Winner-Take-All)
    // -------------------------------------------------------------------------
    logic fist_raw, open_raw, wave_raw;
    
    always_comb begin
        // Default: no gesture
        fist_raw = 1'b0;
        open_raw = 1'b0;
        wave_raw = 1'b0;
        
        if (centroid_valid && blob_area >= MIN_AREA && 
            bbox_width >= MIN_BBOX_W && bbox_height >= MIN_BBOX_H) begin
            
            // Winner-take-all: highest activation wins
            if (activation_fist > activation_open && activation_fist > activation_wave && 
                activation_fist > ACTIVATION_THRESHOLD) begin
                fist_raw = 1'b1;
            end else if (activation_open > activation_fist && activation_open > activation_wave && 
                         activation_open > ACTIVATION_THRESHOLD) begin
                open_raw = 1'b1;
            end else if (activation_wave > activation_fist && activation_wave > activation_open && 
                         activation_wave > ACTIVATION_THRESHOLD) begin
                wave_raw = 1'b1;
            end
        end
    end
    
    // -------------------------------------------------------------------------
    // Online Learning (Perceptron Learning Rule with Edge-Triggered Buttons)
    // -------------------------------------------------------------------------
    logic frame_start_d;
    logic learning_trigger;
    logic [2:0] target_gesture;  // One-hot: [wave, open, fist]
    logic [2:0] learning_buttons_prev;
    logic [2:0] button_pressed;  // Edge detection: button went 0->1
    
    // Detect frame start edge
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_start_d <= 1'b0;
            learning_buttons_prev <= 3'b000;
        end else begin
            frame_start_d <= frame_start;
            learning_buttons_prev <= learning_buttons;
        end
    end
    assign learning_trigger = frame_start && !frame_start_d;
    
    // Edge detection for buttons (only trigger on press, not hold)
    assign button_pressed = learning_buttons & ~learning_buttons_prev;
    
    // Learning logic
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset to initial weights (from initial block)
            // Weights already initialized above
        end else if (learning_trigger && enable && centroid_valid && 
                     learning_mode && (button_pressed != 3'b000)) begin
            // User pressed a button in learning mode - update weights
            target_gesture[0] = button_pressed[0];  // BTN[0] = Fist
            target_gesture[1] = button_pressed[1];  // BTN[1] = Open
            target_gesture[2] = button_pressed[2];  // BTN[2] = Wave
            
            // Update fist weights
            if (target_gesture[0] && !fist_raw) begin
                // Should be fist but isn't - increase weights
                for (int i = 0; i < NUM_FEATURES; i++) begin
                    weights_fist[i] <= weights_fist[i] + 
                                       ($signed({1'b0, features[i]}) * LEARNING_RATE) / 16;
                end
                bias_fist <= bias_fist + LEARNING_RATE;
            end else if (!target_gesture[0] && fist_raw) begin
                // Shouldn't be fist but is - decrease weights
                for (int i = 0; i < NUM_FEATURES; i++) begin
                    weights_fist[i] <= weights_fist[i] - 
                                       ($signed({1'b0, features[i]}) * LEARNING_RATE) / 16;
                end
                bias_fist <= bias_fist - LEARNING_RATE;
            end
            
            // Update open weights
            if (target_gesture[1] && !open_raw) begin
                for (int i = 0; i < NUM_FEATURES; i++) begin
                    weights_open[i] <= weights_open[i] + 
                                       ($signed({1'b0, features[i]}) * LEARNING_RATE) / 16;
                end
                bias_open <= bias_open + LEARNING_RATE;
            end else if (!target_gesture[1] && open_raw) begin
                for (int i = 0; i < NUM_FEATURES; i++) begin
                    weights_open[i] <= weights_open[i] - 
                                       ($signed({1'b0, features[i]}) * LEARNING_RATE) / 16;
                end
                bias_open <= bias_open - LEARNING_RATE;
            end
            
            // Update wave weights
            if (target_gesture[2] && !wave_raw) begin
                for (int i = 0; i < NUM_FEATURES; i++) begin
                    weights_wave[i] <= weights_wave[i] + 
                                       ($signed({1'b0, features[i]}) * LEARNING_RATE) / 16;
                end
                bias_wave <= bias_wave + LEARNING_RATE;
            end else if (!target_gesture[2] && wave_raw) begin
                for (int i = 0; i < NUM_FEATURES; i++) begin
                    weights_wave[i] <= weights_wave[i] - 
                                       ($signed({1'b0, features[i]}) * LEARNING_RATE) / 16;
                end
                bias_wave <= bias_wave - LEARNING_RATE;
            end
        end
    end
    
    // -------------------------------------------------------------------------
    // Temporal Tracking (velocity and oscillation detection)
    // -------------------------------------------------------------------------
    logic prev_dir;
    logic prev_dir_valid;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_x <= 10'd0;
            velocity <= 11'sd0;
            direction_changes <= 3'd0;
            prev_dir <= 1'b0;
            prev_dir_valid <= 1'b0;
        end else if (frame_start_d && centroid_valid) begin
            // Update velocity
            velocity <= $signed({1'b0, centroid_x}) - $signed({1'b0, prev_x});
            prev_x <= centroid_x;
            
            // Track direction changes for wave detection
            if (abs_velocity > 10'd2) begin  // Threshold for meaningful movement
                logic current_dir;
                current_dir = (velocity >= 0) ? 1'b1 : 1'b0;
                
                if (!prev_dir_valid) begin
                    prev_dir <= current_dir;
                    prev_dir_valid <= 1'b1;
                end else if (current_dir != prev_dir) begin
                    // Direction changed
                    direction_changes <= (direction_changes < 3'd7) ? 
                                        (direction_changes + 3'd1) : 3'd7;
                    prev_dir <= current_dir;
                end
            end else begin
                // Low velocity - decay direction changes
                if (direction_changes > 3'd0) begin
                    direction_changes <= direction_changes - 3'd1;
                end
            end
        end
    end
    
    // -------------------------------------------------------------------------
    // Hysteresis and Output
    // -------------------------------------------------------------------------
    logic [2:0] fist_count, open_count, wave_count;
    localparam HYST_FRAMES = 2;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            fist <= 1'b0;
            open_hand <= 1'b0;
            wave <= 1'b0;
            fist_count <= 3'd0;
            open_count <= 3'd0;
            wave_count <= 3'd0;
        end else if (frame_start_d && enable) begin
            // Hysteresis for fist
            if (fist_raw) begin
                if (fist_count < HYST_FRAMES) begin
                    fist_count <= fist_count + 3'd1;
                end else begin
                    fist <= 1'b1;
                end
            end else begin
                fist_count <= 3'd0;
                fist <= 1'b0;
            end
            
            // Hysteresis for open hand
            if (open_raw) begin
                if (open_count < HYST_FRAMES) begin
                    open_count <= open_count + 3'd1;
                end else begin
                    open_hand <= 1'b1;
                end
            end else begin
                open_count <= 3'd0;
                open_hand <= 1'b0;
            end
            
            // Hysteresis for wave
            if (wave_raw) begin
                if (wave_count < HYST_FRAMES) begin
                    wave_count <= wave_count + 3'd1;
                end else begin
                    wave <= 1'b1;
                end
            end else begin
                wave_count <= 3'd0;
                wave <= 1'b0;
            end
        end
    end

endmodule
