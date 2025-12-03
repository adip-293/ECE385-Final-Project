`timescale 1ns / 1ps

/*
 * Control Unit Module
 * 
 * Description:
 *   FSM that cycles through image processing demo modes.
 *   Controls all processing stages via enable signals.
 * 
 * Purpose:
 *   Central state machine for demo sequencing and mode selection.
 *   Provides user control over processing pipeline via buttons and switches.
 * 
 * Notes:
 *   - States: RAW, CHANNEL, GRAYSCALE, THRESHOLD, TEMPORAL, MEDIAN, GAUSSIAN, SOBEL,
 *     SHARPEN, EMBOSS, SKIN, SPATIAL, ERODE, DILATE, CENTROID, BLOB, SKIN_MOTION,
 *     COLOR_THRESH, COLOR_TRACK, GESTURE (20 states total)
 *   - btn[0]: toggle override (force color/grayscale based on sw[15])
 *   - btn[1]: next state, btn[2]: previous state
 *   - sw[3:0]: threshold value, pot_in[15:8]: color/skin params
 */

module control_unit (
    // System
    input  logic        clk,
    input  logic        rst_n,
    
    // Control inputs (synchronized)
    input  logic [2:0]  btn_sync,
    input  logic [15:0] sw_sync,
    input  logic [15:0] pot_in,
    
    // Output control signals
    output logic        grayscale_enable,
    output logic        force_color,         // Override flag for btn[0] - forces color if sw[15], grayscale if !sw[15]
    output logic        force_grayscale,     // Force plain grayscale (when btn[0] pressed and !sw[15])
    output logic        channel_mode_enable, // Enable channel selection mode
    output logic [2:0]  channel_select,      // Channel selection: {R, G, B}
    output logic        threshold_enable,    // Enable thresholding
    output logic [3:0]  threshold_value,     // Threshold value (0-15)
    output logic        median_enable,       // Enable median filtering
    output logic        convolution_enable,  // Enable convolution (Gaussian/Sobel)
    output logic [1:0]  kernel_select,       // Kernel type: 0=Gaussian, 1=Sobel, 2=Sharpen, 3=Emboss
    output logic        skin_threshold_enable, // Enable skin thresholding (RGB444 based)
    output logic        spatial_filter_enable, // Enable spatial filter on skin mask
    output logic        erosion_enable,        // Enable erosion on spatial mask
    output logic        dilation_enable,       // Enable dilation on eroded mask
    output logic        centroid_enable,       // Enable centroid detection
    output logic        blob_filter_enable,    // Enable blob filtering around centroid
    output logic        motion_enable,         // Enable skin_motion tracking (motors)
    output logic [3:0]  skin_y_min,            // Skin threshold Y min (tunable)
    output logic [3:0]  skin_y_max,            // Skin threshold Y max (tunable)
    output logic        color_threshold_enable, // Enable color thresholding
    output logic [7:0]  color_select,           // Color selection from potentiometer
    output logic [3:0]  color_threshold_value,  // Hamming distance threshold
    output logic        temporal_filter_enable, // Enable temporal filtering (moving average across frames)
    output logic        gesture_enable,         // Enable gesture detection overlay
    // (reference color/tolerance outputs removed)
    output logic [4:0]  current_state_num    // Current state number for display (5-bit for 17 states)
);

    // ------------------------------------------------------------
    // Demo State Enumeration
    // ------------------------------------------------------------
    // Re-ordered / corrected enumeration to provide unique codes and align
    // with updated text overlay indices (DILATE added, GESTURE last)
    typedef enum logic [4:0] {
        RAW          = 5'd0,   // Raw color output
        CHANNEL_MODE = 5'd1,   // Individual channel display (controlled by sw[2:0])
        GRAYSCALE    = 5'd2,   // Grayscale converted output
        THRESHOLD    = 5'd3,   // Binary threshold on grayscale
        TEMPORAL     = 5'd4,   // Temporal filter: moving average across frames to reduce noise
        MEDIAN       = 5'd5,   // Median filter then threshold
        GAUSSIAN     = 5'd6,   // Gaussian blur (convolution, sw[15] for threshold)
        SOBEL        = 5'd7,   // Sobel edge detection with threshold
        SHARPEN      = 5'd8,   // Sharpen filter (sw[15] for threshold)
        EMBOSS       = 5'd9,   // Emboss 3D relief effect (sw[15] for threshold)
        SKIN         = 5'd10,  // Skin thresholding (binary mask)
        SPATIAL      = 5'd11,  // 3x3 spatial noise filter on skin mask
        ERODE        = 5'd12,  // Erosion: remove small white specs
        DILATE       = 5'd13,  // Dilation (opening visualization after erosion)
        CENTROID     = 5'd14,  // Centroid detection (after opening)
        BLOB         = 5'd15,  // Blob filtering around centroid
        SKIN_MOTION  = 5'd16,  // Skin motion tracking (motors enabled)
        COLOR_THRESH = 5'd17,  // Color thresholding (Hamming distance)
        COLOR_TRACK  = 5'd18,  // Color tracking + centroid + motion
        GESTURE      = 5'd19   // Gesture detection based on blob/centroid pipeline
    } demo_state_t;
    
    demo_state_t current_state, next_state;
    
    // ------------------------------------------------------------
    // Button Edge Detection
    // ------------------------------------------------------------
    // Detect button presses (inputs are already synchronized)
    logic [2:0] btn_prev;
    logic btn_next_pressed, btn_prev_pressed, btn_override_pressed;
    
    // Toggle state for override button
    logic override_toggle;
    
    // Store previous button state for edge detection
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            btn_prev  <= 3'b0;
            override_toggle <= 1'b0;
        end else begin
            btn_prev  <= btn_sync;
            // Toggle override state on button press
            if (btn_override_pressed) begin
                override_toggle <= ~override_toggle;
            end
            // Clear override when changing states so new state's visuals are visible
            if (btn_next_pressed || btn_prev_pressed) begin
                override_toggle <= 1'b0;
            end
        end
    end
    
    // Edge detection - trigger on rising edge
    assign btn_override_pressed = btn_sync[0] & ~btn_prev[0];  // btn[0]
    assign btn_next_pressed     = btn_sync[1] & ~btn_prev[1];  // btn[1]
    assign btn_prev_pressed     = btn_sync[2] & ~btn_prev[2];  // btn[2]
   
    // ------------------------------------------------------------
    // FSM: Next State Logic
    // ------------------------------------------------------------
    always_comb begin
        // Default: maintain current state
        next_state = current_state;
        
        if (btn_next_pressed) begin
            // Cycle to next state
            case (current_state)
                RAW:          next_state = CHANNEL_MODE;
                CHANNEL_MODE: next_state = GRAYSCALE;
                GRAYSCALE:    next_state = THRESHOLD;
                THRESHOLD:    next_state = TEMPORAL;
                TEMPORAL:     next_state = MEDIAN;
                MEDIAN:       next_state = GAUSSIAN;
                GAUSSIAN:     next_state = SOBEL;
                SOBEL:        next_state = SHARPEN;
                SHARPEN:      next_state = EMBOSS;
                EMBOSS:       next_state = SKIN;
                SKIN:         next_state = SPATIAL;
                SPATIAL:      next_state = ERODE;
                ERODE:        next_state = DILATE;
                DILATE:       next_state = CENTROID;
                CENTROID:     next_state = BLOB;
                BLOB:         next_state = SKIN_MOTION;
                SKIN_MOTION:  next_state = COLOR_THRESH;
                COLOR_THRESH: next_state = COLOR_TRACK;
                COLOR_TRACK:  next_state = GESTURE;
                GESTURE:      next_state = RAW;      // Wrap around
                default:      next_state = RAW;
            endcase
        end else if (btn_prev_pressed) begin
            // Cycle to previous state
            case (current_state)
                RAW:          next_state = GESTURE;     // Wrap around
                CHANNEL_MODE: next_state = RAW;
                GRAYSCALE:    next_state = CHANNEL_MODE;
                THRESHOLD:    next_state = GRAYSCALE;
                TEMPORAL:     next_state = THRESHOLD;
                MEDIAN:       next_state = TEMPORAL;
                GAUSSIAN:     next_state = MEDIAN;
                SOBEL:        next_state = GAUSSIAN;
                SHARPEN:      next_state = SOBEL;
                EMBOSS:       next_state = SHARPEN;
                SKIN:         next_state = EMBOSS;
                SPATIAL:      next_state = SKIN;
                ERODE:        next_state = SPATIAL;
                CENTROID:     next_state = DILATE;
                DILATE:       next_state = ERODE;
                BLOB:         next_state = CENTROID;
                SKIN_MOTION:  next_state = BLOB;
                COLOR_THRESH: next_state = SKIN_MOTION;
                COLOR_TRACK:  next_state = COLOR_THRESH;
                GESTURE:      next_state = COLOR_TRACK;
                default:      next_state = RAW;
            endcase
        end
    end
    
    // ------------------------------------------------------------
    // FSM: State Register
    // ------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            current_state <= RAW;  // Default state on reset
        end else begin
            current_state <= next_state;
        end
    end
    
    // ------------------------------------------------------------
    // FSM: Output Logic
    // ------------------------------------------------------------
    always_comb begin
        // Default outputs
        grayscale_enable    = 1'b0;
        // (no ASCII mode)
        force_color         = 1'b0;
        force_grayscale     = 1'b0;
        channel_mode_enable = 1'b0;
        channel_select      = 3'b111;  // All channels enabled by default
        threshold_enable    = 1'b0;
        threshold_value     = 4'h0;
        median_enable       = 1'b0;
        convolution_enable  = 1'b0;
        kernel_select       = 2'b00;    // Default to Gaussian
        skin_threshold_enable = 1'b0;
        spatial_filter_enable = 1'b0;
        erosion_enable      = 1'b0;
        dilation_enable     = 1'b0;
        centroid_enable     = 1'b0;
        blob_filter_enable  = 1'b0;
        motion_enable       = 1'b0;
        skin_y_min          = 4'd2;   // Default: allow darker skin
        skin_y_max          = 4'd14;  // Default: allow lighter skin
        color_threshold_enable = 1'b0;
        color_select        = 8'h00;
        color_threshold_value = 4'h0;
        temporal_filter_enable = 1'b0;
        gesture_enable      = 1'b0;
        // Reference color/tolerance logic removed
        
        // Check for override toggle state
        if (override_toggle) begin
            // Override enabled: force DISPLAY output based on sw[15]
            // sw[15] = 1: display raw color
            // sw[15] = 0: display plain grayscale
            // BUT: still run all processing in background for states that need it (for overlays)
            if (sw_sync[15]) begin
                force_color         = 1'b1;
                force_grayscale     = 1'b0;
            end else begin
                force_color         = 1'b0;
                force_grayscale     = 1'b1;
            end

            // Now apply state-specific processing (same as normal operation)
            // This ensures background computation still happens for overlays
            case (current_state)
                RAW: begin
                    grayscale_enable    = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select       = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end
                
                CHANNEL_MODE: begin
                    grayscale_enable    = 1'b0;
                    channel_mode_enable = 1'b1;
                    channel_select = sw_sync[2:0];
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select       = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end
                
                GRAYSCALE: begin
                    grayscale_enable    = 1'b1;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end
                
                THRESHOLD: begin
                    grayscale_enable    = 1'b1;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12];
                    threshold_enable    = (threshold_value != 4'h0);
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                MEDIAN: begin
                    grayscale_enable    = 1'b1;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12];
                    threshold_enable    = (threshold_value != 4'h0);
                    median_enable       = 1'b1;
                    convolution_enable  = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                GAUSSIAN: begin
                    grayscale_enable    = 1'b1;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12];
                    threshold_enable    = (threshold_value != 4'h0);
                    median_enable       = 1'b1;
                    convolution_enable  = 1'b1;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                SOBEL: begin
                    grayscale_enable    = 1'b1;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12];
                    threshold_enable    = (threshold_value != 4'h0);
                    median_enable       = 1'b1;
                    convolution_enable  = 1'b1;
                    kernel_select         = 2'b01;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                SHARPEN: begin
                    grayscale_enable    = 1'b1;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12];
                    threshold_enable    = (threshold_value != 4'h0);
                    median_enable       = 1'b1;
                    convolution_enable  = 1'b1;
                    kernel_select         = 2'b10;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                EMBOSS: begin
                    grayscale_enable    = 1'b1;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12];
                    threshold_enable    = (threshold_value != 4'h0);
                    median_enable       = 1'b1;
                    convolution_enable  = 1'b1;
                    kernel_select         = 2'b11;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                    temporal_filter_enable = 1'b0;
                end
                
                TEMPORAL: begin
                    grayscale_enable    = 1'b1;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select       = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                    temporal_filter_enable = 1'b1;
                end
                
                SKIN: begin
                    grayscale_enable    = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                    centroid_enable     = 1'b0;
                    skin_y_min          = pot_in[15:12];
                    skin_y_max          = pot_in[11:8];
                end
                
                SPATIAL: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b0;
                    dilation_enable       = 1'b0;
                    centroid_enable       = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                ERODE: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b0;
                    centroid_enable       = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end

                DILATE: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b1;
                    centroid_enable       = 1'b0;
                    blob_filter_enable    = 1'b0;
                    motion_enable         = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                CENTROID: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b1;
                    centroid_enable       = 1'b1;
                    motion_enable         = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                BLOB: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b1;
                    centroid_enable       = 1'b1;
                    blob_filter_enable    = 1'b1;
                    motion_enable         = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                SKIN_MOTION: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b1;
                    centroid_enable       = 1'b1;
                    blob_filter_enable    = 1'b1;
                    motion_enable         = 1'b1;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                    color_threshold_enable = 1'b0;
                    color_select         = 8'h00;
                    color_threshold_value = 4'h0;
                end
                
                COLOR_THRESH: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b0;
                    dilation_enable       = 1'b0;
                    centroid_enable       = 1'b1;
                    motion_enable         = 1'b0;
                    skin_y_min            = 4'd2;
                    skin_y_max            = 4'd14;
                    color_threshold_enable = 1'b1;
                    color_select          = pot_in[15:8];
                    color_threshold_value = sw_sync[3:0];
                end
                
                COLOR_TRACK: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b0;
                    dilation_enable       = 1'b0;
                    centroid_enable       = 1'b1;
                    motion_enable         = 1'b1;
                    skin_y_min            = 4'd2;
                    skin_y_max            = 4'd14;
                    color_threshold_enable = 1'b1;
                    color_select          = pot_in[15:8];
                    color_threshold_value = sw_sync[3:0];
                end
                
                GESTURE: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b1;
                    centroid_enable       = 1'b1;
                    blob_filter_enable    = 1'b1;
                    motion_enable         = 1'b0;
                    gesture_enable        = 1'b1;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                default: begin
                    grayscale_enable      = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable        = 1'b0;
                    dilation_enable       = 1'b0;
                    centroid_enable       = 1'b0;
                    motion_enable         = 1'b0;
                    skin_y_min          = 4'd2;
                    skin_y_max          = 4'd14;
                    color_threshold_enable = 1'b0;
                    color_select         = 8'h00;
                    color_threshold_value = 4'h0;
                end
            endcase
        end else begin
            // Normal state-based output
            case (current_state)
                RAW: begin
                    grayscale_enable    = 1'b0;
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;  // RGB all enabled
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select       = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end
                
                CHANNEL_MODE: begin
                    grayscale_enable    = 1'b0;
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b1;    // Enable channel selector
                    // sw[2:0] = {R_enable, G_enable, B_enable}
                    // 111 = all channels (normal color)
                    // 100 = red only
                    // 010 = green only
                    // 001 = blue only
                    // 110 = red + green (yellow)
                    // 101 = red + blue (magenta)
                    // 011 = green + blue (cyan)
                    // 000 = no channels (black)
                    channel_select = sw_sync[2:0];
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select       = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end
                
                GRAYSCALE: begin
                    grayscale_enable    = 1'b1;
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;  // Not used in grayscale mode
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                // (ASCII state removed)
                
                THRESHOLD: begin
                    grayscale_enable    = 1'b1;    // Need grayscale first
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12];  // Use potentiometer for threshold (0-15)
                    threshold_enable    = (threshold_value != 4'h0); // Disable thresholding if value is 0
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                MEDIAN: begin
                    grayscale_enable    = 1'b1;    // Need grayscale first
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12];  // Use potentiometer for threshold
                    threshold_enable    = (threshold_value != 4'h0);  // Only enable if nonzero
                    median_enable       = 1'b1;    // Enable median filtering
                    convolution_enable  = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                GAUSSIAN: begin
                    grayscale_enable    = 1'b1;    // Need grayscale first
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12]; // Use potentiometer for threshold value
                    threshold_enable    = (threshold_value != 4'h0);  // Only enable if nonzero
                    median_enable       = 1'b1;    // Apply median for denoising
                    convolution_enable  = 1'b1;    // Enable Gaussian convolution
                    kernel_select         = 2'b00;   // 0 = Gaussian kernel
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                SOBEL: begin
                    grayscale_enable    = 1'b1;    // Need grayscale first
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12]; // Use potentiometer for edge threshold
                    threshold_enable    = (threshold_value != 4'h0);  // Only enable if nonzero
                    median_enable       = 1'b1;    // Apply median for denoising
                    convolution_enable  = 1'b1;    // Enable Sobel convolution
                    kernel_select         = 2'b01;   // 1 = Sobel kernel
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                SHARPEN: begin
                    grayscale_enable    = 1'b1;    // Need grayscale first
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12]; // Use potentiometer for threshold value
                    threshold_enable    = (threshold_value != 4'h0);  // Only enable if nonzero
                    median_enable       = 1'b1;    // Apply median for denoising
                    convolution_enable  = 1'b1;    // Enable Sharpen convolution
                    kernel_select         = 2'b10;   // 2 = Sharpen kernel
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                end

                EMBOSS: begin
                    grayscale_enable    = 1'b1;    // Need grayscale first
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_value     = pot_in[15:12]; // Use potentiometer for threshold value
                    threshold_enable    = (threshold_value != 4'h0);  // Only enable if nonzero
                    median_enable       = 1'b1;    // Apply median for denoising
                    convolution_enable  = 1'b1;    // Enable Emboss convolution
                    kernel_select         = 2'b11;   // 3 = Emboss kernel
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                    temporal_filter_enable = 1'b0;
                end
                TEMPORAL: begin
                    // Temporal filter: moving average across frames to reduce noise
                    grayscale_enable    = 1'b1;    // Need grayscale for temporal filtering
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select       = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                    temporal_filter_enable = 1'b1;  // Enable temporal filtering
                end
                SKIN: begin
                    // Skin thresholding: classify RGB444 pixels; display binary mask
                    // pot_in[15:12] = Y_MIN (high-res), pot_in[11:8] = Y_MAX (low-res)
                    grayscale_enable    = 1'b0;
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;  
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                    centroid_enable     = 1'b0;
                    skin_y_min          = pot_in[15:12];   // Use potentiometer for tuning (high-res)
                    skin_y_max          = pot_in[11:8];    // Use potentiometer for tuning (low-res)
                end
                SPATIAL: begin
                    // Apply spatial filter to skin mask
                    // pot_in[15:12] = Y_MIN (high-res), pot_in[11:8] = Y_MAX (low-res)
                    grayscale_enable      = 1'b0;
                    force_color           = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    // Incremental layering: retain skin threshold
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b0;
                    dilation_enable       = 1'b0;
                    centroid_enable       = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                ERODE: begin
                    // Erosion: remove small white noise specs
                    // pot_in[15:12] = Y_MIN (high-res), pot_in[11:8] = Y_MAX (low-res)
                    grayscale_enable      = 1'b0;
                    force_color           = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    // Incremental layering: skin + spatial + erosion
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b0;
                    centroid_enable       = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end

                DILATE: begin
                    // Dilation: expand mask after erosion (opening visualization)
                    // pot_in[15:12] = Y_MIN, pot_in[11:8] = Y_MAX
                    grayscale_enable      = 1'b0;
                    force_color           = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    // Incremental layering: skin + spatial + erosion + dilation
                    skin_threshold_enable = 1'b1;
                    spatial_filter_enable = 1'b1;
                    erosion_enable        = 1'b1;  // keep prior erosion
                    dilation_enable       = 1'b1;  // add dilation
                    centroid_enable       = 1'b0;  // visualization only
                    blob_filter_enable    = 1'b0;
                    motion_enable         = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                CENTROID: begin
                    // Centroid tracking: full pipeline with erosion+dilation+centroid
                    // pot_in[15:12] = Y_MIN (high-res), pot_in[11:8] = Y_MAX (low-res)
                    grayscale_enable      = 1'b0;
                    force_color           = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;  // re-enable skin threshold for full pipeline
                    spatial_filter_enable = 1'b1;  // include spatial denoise upstream
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b1;
                    centroid_enable       = 1'b1;
                    motion_enable         = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                BLOB: begin
                    // Blob filtering: refine mask near previous centroid before further tracking
                    grayscale_enable      = 1'b0;
                    force_color           = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;  // ensure skin classification still active
                    spatial_filter_enable = 1'b1;  // denoise before morphology and blob filtering
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b1;
                    centroid_enable       = 1'b1;
                    blob_filter_enable    = 1'b1;
                    motion_enable         = 1'b0;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                SKIN_MOTION: begin
                    // Skin_motion tracking: same as CENTROID but with motor control enabled
                    // pot_in[15:12] = Y_MIN (high-res), pot_in[11:8] = Y_MAX (low-res)
                    grayscale_enable      = 1'b0;
                    force_color           = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;  // include skin classification
                    spatial_filter_enable = 1'b1;  // and spatial filter for cleaner tracking mask
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b1;
                    centroid_enable       = 1'b1;
                    blob_filter_enable    = 1'b1;
                    motion_enable         = 1'b1;  // Enable motors!
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                    color_threshold_enable = 1'b0;
                    color_select         = 8'h00;
                    color_threshold_value = 4'h0;
                end
                
                COLOR_THRESH: begin
                    // Color thresholding: Hamming distance based color matching
                    // pot_in[15:8] = color_select (8-bit color selection)
                    // sw_sync[3:0] = threshold value (Hamming distance threshold) - use switches for better resolution
                    grayscale_enable      = 1'b0;
                    force_color           = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b1;  // Enable spatial filter on color mask
                    erosion_enable        = 1'b0;
                    dilation_enable       = 1'b0;
                    centroid_enable       = 1'b1;  // Enable centroid overlay
                    motion_enable         = 1'b0;
                    skin_y_min            = 4'd2;
                    skin_y_max            = 4'd14;
                    color_threshold_enable = 1'b1;
                    color_select          = pot_in[15:8];  // 8-bit color selection from potentiometer
                    color_threshold_value = sw_sync[3:0];  // 4-bit Hamming distance threshold from switches
                end
                
                COLOR_TRACK: begin
                    // Color tracking: color threshold + centroid + motion tracking
                    // pot_in[15:8] = color_select (8-bit color selection)
                    // sw_sync[3:0] = threshold value (Hamming distance threshold) - use switches for better resolution
                    grayscale_enable      = 1'b0;
                    force_color           = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b1;  // Enable spatial filter on color mask
                    erosion_enable        = 1'b0;
                    dilation_enable       = 1'b0;
                    centroid_enable       = 1'b1;  // Enable centroid detection
                    motion_enable         = 1'b1;  // Enable motor tracking!
                    skin_y_min            = 4'd2;
                    skin_y_max            = 4'd14;
                    color_threshold_enable = 1'b1;
                    color_select          = pot_in[15:8];  // 8-bit color selection from potentiometer
                    color_threshold_value = sw_sync[3:0];  // 4-bit Hamming distance threshold from switches
                end
                GESTURE: begin
                    // Gesture detection should use the same processed output as BLOB
                    // i.e., skin pipeline → erosion → dilation → centroid → blob_filter
                    grayscale_enable      = 1'b0;
                    force_color           = 1'b0;
                    channel_mode_enable   = 1'b0;
                    channel_select        = 3'b111;
                    threshold_enable      = 1'b0;
                    threshold_value       = 4'h0;
                    median_enable         = 1'b0;
                    convolution_enable    = 1'b0;
                    kernel_select         = 2'b00;
                    skin_threshold_enable = 1'b1;   // activate skin threshold for gesture pipeline
                    spatial_filter_enable = 1'b1;   // include spatial denoise
                    erosion_enable        = 1'b1;
                    dilation_enable       = 1'b1;
                    centroid_enable       = 1'b1;
                    blob_filter_enable    = 1'b1;
                    motion_enable         = 1'b0;
                    gesture_enable        = 1'b1;
                    skin_y_min            = pot_in[15:12];
                    skin_y_max            = pot_in[11:8];
                end
                
                default: begin
                    grayscale_enable    = 1'b0;
                    force_color         = 1'b0;
                    channel_mode_enable = 1'b0;
                    channel_select      = 3'b111;
                    threshold_enable    = 1'b0;
                    threshold_value     = 4'h0;
                    median_enable       = 1'b0;
                    convolution_enable  = 1'b0;
                    kernel_select       = 2'b00;
                    skin_threshold_enable = 1'b0;
                    spatial_filter_enable = 1'b0;
                    erosion_enable      = 1'b0;
                    dilation_enable     = 1'b0;
                    centroid_enable     = 1'b0;
                    motion_enable       = 1'b0;
                    skin_y_min          = 4'd2;
                    skin_y_max          = 4'd14;
                    color_threshold_enable = 1'b0;
                    color_select         = 8'h00;
                    color_threshold_value = 4'h0;
                end
            endcase
        end
    end
    
    // State number output for hex display
    assign current_state_num = current_state;

endmodule
