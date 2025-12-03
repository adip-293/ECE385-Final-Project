`timescale 1ns / 1ps

/*
 * Processor Module
 * 
 * Description:
 *   Top-level processor coordinating camera capture, image processing, and HDMI display.
 *   Integrates all subsystems: camera controller, preprocessor, buffer, postprocessor, motors.
 * 
 * Purpose:
 *   Main system integration and clock domain crossing management.
 *   Routes control signals and data between all major components.
 * 
 * Notes:
 *   - Three clock domains: clk (100MHz), pclk (camera), clk_25m (VGA)
 *   - Reset synchronizers for each domain to prevent metastability
 *   - Control unit provides centralized state machine for demo modes
 *   - Motor controller enables object tracking with pan/tilt servos
 *   - Hex displays show potentiometer value and centroid X coordinate
 */

module processor (
    // System
    input  logic        clk,
    input  logic        rst,
    
    // Control inputs
    input  logic [2:0]  btn,
    input  logic [15:0] sw,
    
    // Analog input
    input logic VP,
    input logic VN,

    // Hex display outputs
    output logic [7:0]  hex_segA,
    output logic [7:0]  hex_segB,
    output logic [3:0]  hex_gridA,
    output logic [3:0]  hex_gridB,
    
    // Status LEDs
    output logic [15:0]  led,
    
    // Camera interface
    input  logic        pclk,
    input  logic [7:0]  pix_byte,
    input  logic        vsync,
    input  logic        href,
    output logic        cam_reset,
    output logic        pwdn,
    output logic        xclk,
    inout  wire         siod,
    output logic        sioc,

    // Motor PWM outputs
    output logic pwm_pan,
    output logic pwm_tilt,

    // HDMI output
    output logic        hdmi_tmds_clk_n,
    output logic        hdmi_tmds_clk_p,
    output logic [2:0]  hdmi_tmds_data_n,
    output logic [2:0]  hdmi_tmds_data_p
);

    // ------------------------------------------------------------
    // Internal Signals
    // ------------------------------------------------------------
    logic clk_25m, clk_125m;
    logic locked;
    logic rst_debounced;
    
    // Reset synchronizer signals
    logic rst_sync_clk, rst_sync_25m, rst_sync_pclk;
    logic r2_rst_clk, r2_rst_25m, r2_rst_pclk;
    
    // Synchronized control inputs
    logic [2:0]  btn_sync;
    logic [15:0] sw_sync;
    
    // Control signals from control unit
    logic grayscale_enable;
    logic force_color;
    logic force_grayscale;
    logic channel_mode_enable;
    logic [2:0] channel_select;
    logic threshold_enable;
    logic [3:0] threshold_value;
    logic median_enable;
    logic convolution_enable;
    logic [1:0] kernel_select;
    logic skin_threshold_enable;
    logic spatial_filter_enable;
    logic temporal_filter_enable;
    logic erosion_enable;
    logic dilation_enable;
    logic centroid_enable;
    logic blob_filter_enable;
    logic motion_enable;
    logic [3:0] skin_y_min;
    logic [3:0] skin_y_max;
    logic        color_threshold_enable;
    logic [7:0]  color_select;
    logic [3:0]  color_threshold_value;
    logic        gesture_enable;
    // (reference color/tolerance logic removed)
    logic [4:0] current_state_num;  // 5-bit to support states incl. GESTURE
    
    // VGA signals
    logic [2:0] vga_red, vga_green, vga_blue;          // Final output to HDMI
    logic [2:0] vga_red_raw, vga_green_raw, vga_blue_raw;  // Raw from frame buffer
    logic       vga_vsync, vga_hsync, vde;
    logic [9:0] draw_x, draw_y;
    logic [1:0] frame_chunk_counter;  // From buffer: temporal filter frame chunk index
    
    // Memory interface signals
    logic [11:0] cam_pix_data;
    logic [18:0] cam_pix_addr;
    logic        cam_pix_write;
    logic [8:0]  vga_pix_data;
    // Line tear-guard signals (derived from href/vsync in pclk domain)
    logic [9:0]  cam_line_y;
    logic        cam_line_ready;
    logic        href_d1;
    logic        vsync_d1, vsync_d2;
    
    // Camera sync signals
    logic vsync_sync;
    
    // Preprocessor outputs
    logic [3:0]  processed_pixel;
    logic [18:0] processed_addr;
    logic        processed_valid;
    
    // Centroid outputs
    logic [9:0]  centroid_x;
    logic [9:0]  centroid_y;
    logic        centroid_valid;
    logic [19:0] blob_area;
    logic [3:0]  motor_direction;
    // Bounding box from centroid
    logic [9:0]  centroid_bbox_min_x, centroid_bbox_min_y;
    logic [9:0]  centroid_bbox_max_x, centroid_bbox_max_y;

    // Gesture detection outputs (used for on-screen overlay only)
    logic gesture_fist, gesture_open, gesture_wave;

    // Potentiometer output
    logic [15:0] pot_out;

    // LEDs: no gesture indicators; keep motor direction on upper LEDs
    assign led[15:12] = motor_direction; // top 4 LEDs show motor direction
    assign led[11:1]  = 11'b0;           // clear LEDs 11..1
    // led[0] is driven by camera cam_done
    
    // ------------------------------------------------------------
    // Clock Generation and Reset Management
    // ------------------------------------------------------------
    
    // Clock wizard - generates 25MHz and 125MHz clocks
    clk_wiz_0 clock_generator (
        .clk_out1(clk_25m),      // 25MHz for VGA/HDMI
        .clk_out2(clk_125m),         // 125MHz for camera
        .clk_out3(xclk),         // 24MHz for camera
        .locked(locked),
        .clk_in1(clk)
    );
    
    // Reset button debouncer
    sync_debounce reset_debouncer (
        .clk(clk),
        .d(~rst),                // Invert for active-low reset
        .q(rst_debounced)
    );
    
    sync_flop rst_clk_synchronizer (
        .clk(clk),
        .d(rst_debounced),
        .q(rst_sync_clk)
    );
    assign r2_rst_clk = rst_sync_clk;
    
    sync_flop rst_25m_synchronizer (
        .clk(clk_25m),
        .d(rst_debounced),
        .q(rst_sync_25m)
    );
    assign r2_rst_25m = rst_sync_25m;
    
    sync_flop rst_pclk_synchronizer (
        .clk(pclk),
        .d(rst_debounced),
        .q(rst_sync_pclk)
    );
    assign r2_rst_pclk = rst_sync_pclk;

    // ------------------------------------------------------------
    // Input Synchronizers
    // ------------------------------------------------------------
    // Button synchronizers with debouncing (array syntax)
    sync_debounce btn_sync_inst [2:0] (
        .clk({3{clk_25m}}),
        .d(btn),
        .q(btn_sync)
    );
    
    // Switch synchronizers (basic flop, no debouncing needed, array syntax)
    sync_flop sw_sync_inst [15:0] (
        .clk({16{clk_25m}}),
        .d(sw),
        .q(sw_sync)
    );
    
    // Vsync synchronizer for line buffer
    sync_flop vsync_sync_inst (
        .clk(pclk),
        .d(vsync),
        .q(vsync_sync)
    );

    // ------------------------------------------------------------
    // Line-ready pulse and line counter in pclk domain
    // Minimal logic to avoid reading the line currently being written
    // ------------------------------------------------------------
    always_ff @(posedge pclk or negedge r2_rst_pclk) begin
        if (!r2_rst_pclk) begin
            href_d1 <= 1'b0;
            vsync_d1 <= 1'b0;
            vsync_d2 <= 1'b0;
            cam_line_y <= 10'd0;
            cam_line_ready <= 1'b0;
        end else begin
            href_d1 <= href;
            vsync_d1 <= vsync;
            vsync_d2 <= vsync_d1;
            cam_line_ready <= 1'b0; // default

            // Reset line counter on frame start (vsync falling edge)
            if (vsync_d2 && ~vsync_d1) begin
                cam_line_y <= 10'd0;
            end

            // Pulse ready and increment line count on href falling edge
            if (href_d1 && ~href) begin
                cam_line_ready <= 1'b1;
                if (cam_line_y == 10'd479)
                    cam_line_y <= 10'd0;
                else
                    cam_line_y <= cam_line_y + 10'd1;
            end
        end
    end

    // ------------------------------------------------------------
    // Control Unit - State machine for demo modes
    // ------------------------------------------------------------
    // btn[0] - Force raw color output (override)
    // btn[1] - Next demo state
    // btn[2] - Previous demo state
    // sw[2:0] - Channel select in CHANNEL_MODE state
    // sw[3:0] - Threshold value in THRESHOLD/MEDIAN states
    // sw[15:4] - Reserved for future features
    
    control_unit ctrl (
        .clk(clk_25m),
        .rst_n(r2_rst_25m),
        // Synchronized inputs
        .btn_sync(btn_sync),
        .sw_sync(sw_sync),
        .pot_in(pot_out),
        // Outputs
        .grayscale_enable(grayscale_enable),
        .force_color(force_color),
        .force_grayscale(force_grayscale),
        .channel_mode_enable(channel_mode_enable),
        .channel_select(channel_select),
        .threshold_enable(threshold_enable),
        .threshold_value(threshold_value),
        .median_enable(median_enable),
        .convolution_enable(convolution_enable),
        .kernel_select(kernel_select),
        .skin_threshold_enable(skin_threshold_enable),
        .spatial_filter_enable(spatial_filter_enable),
        .erosion_enable(erosion_enable),
        .dilation_enable(dilation_enable),
        .centroid_enable(centroid_enable),
        .blob_filter_enable(blob_filter_enable),
        .motion_enable(motion_enable),
        .skin_y_min(skin_y_min),
        .skin_y_max(skin_y_max),
        .color_threshold_enable(color_threshold_enable),
        .color_select(color_select),
        .color_threshold_value(color_threshold_value),
        .temporal_filter_enable(temporal_filter_enable),
        .gesture_enable(gesture_enable),
        // (reference color/tolerance connections removed)
        .current_state_num(current_state_num)
    );

    // ------------------------------------------------------------
    // Hex Drivers - Display state and centroid coordinates
    // ------------------------------------------------------------

    // HexA (left): Display potentiometer value in hex
    hex_driver HexA (
        .clk(clk),
        .reset(~rst_debounced),
        .in({pot_out[15:12], pot_out[11:8], pot_out[7:4], pot_out[3:0]}),
        .hex_seg(hex_segA),
        .hex_grid(hex_gridA)
    );
        // ------------------------------------------------------------
        // Potentiometer Controller
        // ------------------------------------------------------------
        pot_controller pot_ctrl_inst (
            .clk(clk),
            .reset(rst),
            .VP(VP),
            .VN(VN),
            .pot_out(pot_out)
        );
    
    // HexB (right): Display centroid X coordinate in hex
    // Digits from left to right: [0, x[9:6], x[5:2], x[1:0]&valid]
    hex_driver HexB (
        .clk(clk),
        .reset(~rst_debounced),
        .in({4'h0, {centroid_valid,1'b0, centroid_x[9:8]}, centroid_x[7:4], centroid_x[3:0]}),
        .hex_seg(hex_segB),
        .hex_grid(hex_gridB)
    );

    // ------------------------------------------------------------
    // Camera Controller
    // ------------------------------------------------------------
    cam_controller #(
        .CAM_CONFIG_CLK(100_000_000)
    ) camera (
        // System
        .clk(clk_25m),
        .rst_n_clk(r2_rst_clk),
        .rst_n_pclk(r2_rst_pclk),
        
        // Control - auto-start camera after reset
        .cam_start(r2_rst_clk),  // Start camera when system is out of reset
        .cam_done(led[0]),
        
        // Camera physical interface
        .pclk(pclk),
        .pix_byte(pix_byte),
        .vsync(vsync),
        .href(href),
        .cam_reset(cam_reset),
        .cam_pwdn(pwdn),
        
        // SCCB interface
        .sioc(sioc),
        .siod(siod),
        
        // Memory interface
        .pix_data(cam_pix_data),
        .pix_addr(cam_pix_addr),
        .pix_write(cam_pix_write)
    );

    // ------------------------------------------------------------
    // Preprocessor Module - All image processing before frame buffer
    // ------------------------------------------------------------
    preprocessor img_preproc (
        .clk(pclk),
        .rst_n(r2_rst_pclk),
        
        // Camera inputs
        .cam_pix_data(cam_pix_data),
        .cam_pix_addr(cam_pix_addr),
        .cam_pix_write(cam_pix_write),
        .cam_vsync(vsync_sync),
        
        // Processing control
        .grayscale_enable(grayscale_enable || force_grayscale),  // Enable grayscale in forced grayscale mode
        .threshold_enable(threshold_enable && !force_grayscale),  // Disable threshold in forced grayscale mode
        .threshold_value(threshold_value),
        .median_enable(median_enable && !force_grayscale),
        .convolution_enable(convolution_enable && !force_grayscale),
        .kernel_select(kernel_select),
        .skin_threshold_enable(skin_threshold_enable && !force_grayscale),
        .spatial_filter_enable(spatial_filter_enable && !force_grayscale),
        .erosion_enable(erosion_enable && !force_grayscale),
        .dilation_enable(dilation_enable && !force_grayscale),
        .centroid_enable(centroid_enable && !force_grayscale),
        .blob_filter_enable(blob_filter_enable && !force_grayscale),
        .gesture_enable(gesture_enable && !force_grayscale),
        .skin_y_min(skin_y_min),
        .skin_y_max(skin_y_max),
        .color_threshold_enable(color_threshold_enable && !force_grayscale),
        .color_select(color_select),
        .color_threshold_value(color_threshold_value),
        // (reference color/tolerance connections removed)
        
        // Processed outputs
        .processed_pixel(processed_pixel),
        .processed_addr(processed_addr),
        .processed_valid(processed_valid),
        .is_border_pixel(),  // Unused
        
        // Centroid outputs
        .centroid_x(centroid_x),
        .centroid_y(centroid_y),
        .centroid_valid(centroid_valid),
        .blob_area(blob_area),
        .centroid_bbox_min_x(centroid_bbox_min_x),
        .centroid_bbox_min_y(centroid_bbox_min_y),
        .centroid_bbox_max_x(centroid_bbox_max_x),
        .centroid_bbox_max_y(centroid_bbox_max_y),
        .gesture_fist(gesture_fist),
        .gesture_open(gesture_open),
        .gesture_wave(gesture_wave)
    );
    
    // ------------------------------------------------------------
    // Buffer Module (Frame buffer BRAM only)
    // ------------------------------------------------------------
    buffer pixel_buffer (
        // System
        .clk_25m(clk_25m),
        .rst_n(r2_rst_25m),
        .pclk(pclk),
        
        // VGA timing inputs
        .draw_x(draw_x),
        .draw_y(draw_y),
        .vde(vde),
        
        // Preprocessed pixel inputs
        .processed_pixel(processed_pixel),
        .processed_addr(processed_addr),
        .processed_valid(processed_valid),
        
        // Processing mode control
        // In forced grayscale mode: enable processing (grayscale conversion only)
        // In forced color mode: disable processing (use camera data directly)
        // Otherwise: use normal processing pipeline
        .processing_enable(((grayscale_enable || threshold_enable || median_enable || convolution_enable || skin_threshold_enable || spatial_filter_enable || erosion_enable || dilation_enable || centroid_enable || blob_filter_enable || color_threshold_enable || temporal_filter_enable) && !force_color) || force_grayscale),
        // Temporal filter enable
        .temporal_filter_enable(temporal_filter_enable),        
        // Original camera data (for color mode)
        .cam_pix_data(cam_pix_data),
        .cam_pix_addr(cam_pix_addr),
        .cam_pix_write(cam_pix_write),
        .cam_vsync(vsync_sync),  // Camera vsync for frame counter
        .cam_line_y(cam_line_y),
        .cam_line_ready(cam_line_ready),
        // Compare mode: sw[13] toggles left processed/right raw split (moved off ch. select)
        .compare_mode(sw_sync[13]),
        
        // VGA read interface
        .vga_pix_data(vga_pix_data),
        .frame_chunk_counter(frame_chunk_counter)  // Frame chunk counter (for temporal filter)
    );

    // ------------------------------------------------------------
    // VGA Controller
    // ------------------------------------------------------------
    vga_controller vga_ctrl (
        .pixel_clk(clk_25m),
        .reset(~r2_rst_25m),
        .hs(vga_hsync),
        .vs(vga_vsync),
        .active_nblank(vde),
        .sync(),
        .drawX(draw_x),
        .drawY(draw_y)
    );

    // ------------------------------------------------------------
    // RGB Generation - Direct mapping from pixel data
    // ------------------------------------------------------------
    assign vga_red_raw   = vga_pix_data[8:6];
    assign vga_green_raw = vga_pix_data[5:3];
    assign vga_blue_raw  = vga_pix_data[2:0];
    
    // ------------------------------------------------------------
    // Postprocessor - Channel selection and overlay rendering
    // ------------------------------------------------------------
    postprocessor #(
        .FRAME_WIDTH(640),
        .FRAME_HEIGHT(480),
        .CROSSHAIR_LENGTH(20),
        .CROSSHAIR_THICKNESS(2),
        .CENTER_GAP(5)
    ) postproc (
        .clk(clk_25m),
        .rst_n(r2_rst_25m),
        
        // Control signals
        .channel_mode_enable(channel_mode_enable),
        .channel_select(channel_select),
        .overlay_enable(centroid_enable || motion_enable),  // Crosshair overlay
        .bbox_overlay_enable(sw_sync[14] && (centroid_enable || blob_filter_enable)), // Switch 14 toggles bbox in centroid/blob
        .processing_enable(((grayscale_enable || threshold_enable || median_enable || convolution_enable || skin_threshold_enable || spatial_filter_enable || erosion_enable || dilation_enable || centroid_enable || blob_filter_enable || color_threshold_enable || temporal_filter_enable) && !force_color) || force_grayscale),  // Processing mode: replicate 3-bit grayscale (including forced grayscale)
        .temporal_filter_enable(temporal_filter_enable),  // Enable temporal filtering
        .frame_chunk_counter(frame_chunk_counter),  // Frame chunk counter for temporal filter
        .force_color(force_color),  // Force color override
        .force_grayscale(force_grayscale),  // Force grayscale override
        .current_state(current_state_num), // Current state for text display
        // Gesture overlay enable comes from control unit's gesture state
        .gesture_enable(gesture_enable),   // Enable gesture text overlay
        
        // VGA timing
        .draw_x(draw_x),
        .draw_y(draw_y),
        .vde(vde),
        
        // Centroid information for overlay
        .centroid_x(centroid_x),
        .centroid_y(centroid_y),
        .centroid_valid(centroid_valid),
        .centroid_bbox_min_x(centroid_bbox_min_x),
        .centroid_bbox_min_y(centroid_bbox_min_y),
        .centroid_bbox_max_x(centroid_bbox_max_x),
        .centroid_bbox_max_y(centroid_bbox_max_y),
        .gesture_fist(gesture_fist),
        .gesture_open(gesture_open),
        // Use only detected wave gesture (no switch override)
        .gesture_wave(gesture_wave),
        
        // Input pixels from frame buffer
        .pixel_red_in(vga_red_raw),
        .pixel_green_in(vga_green_raw),
        .pixel_blue_in(vga_blue_raw),
        
        // Output pixels to HDMI
        .pixel_red_out(vga_red),
        .pixel_green_out(vga_green),
        .pixel_blue_out(vga_blue)
    );

    // ------------------------------------------------------------
    // HDMI Output
    // ------------------------------------------------------------
    hdmi_tx_0 hdmi_transmitter (
        // Clocking and reset
        .pix_clk(clk_25m),
        .pix_clkx5(clk_125m),
        .pix_clk_locked(locked),
        .rst(~r2_rst_25m),        // Active high reset
        
        // Video signals
        .red(vga_red),
        .green(vga_green),
        .blue(vga_blue),
        .hsync(vga_hsync),
        .vsync(vga_vsync),
        .vde(vde),
        
        // Auxiliary data (unused)
        .aux0_din(4'b0),
        .aux1_din(4'b0),
        .aux2_din(4'b0),
        .ade(1'b0),
        
        // HDMI differential outputs
        .TMDS_CLK_P(hdmi_tmds_clk_p),
        .TMDS_CLK_N(hdmi_tmds_clk_n),
        .TMDS_DATA_P(hdmi_tmds_data_p),
        .TMDS_DATA_N(hdmi_tmds_data_n)
    );

    // ------------------------------------------------------------
    // Motor Driver - Auto-centering based on centroid
    // ------------------------------------------------------------
    motor_controller motor_ctrl (
        .clk(clk_25m),
        .rst_n(r2_rst_25m),
        // Prevent runaway when centroid is not valid by requiring a valid target
        .enable(motion_enable && centroid_valid),
        .centroid_x(centroid_x),
        .centroid_y(centroid_y),
        .centroid_valid(centroid_valid),
        .pwm_pan(pwm_pan),
        .pwm_tilt(pwm_tilt),
        .motor_direction(motor_direction),
        .error_x(),  // Output port (unused at top level)
        .error_y()   // Output port (unused at top level)
    );

endmodule