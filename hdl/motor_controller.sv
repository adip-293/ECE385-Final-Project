`timescale 1ns / 1ps

/*
 * Motor Controller Module
 * 
 * Description:
 *   PID-style motor controller for pan/tilt servos.
 *   Centers detected object (centroid) in camera frame by commanding motor movements.
 * 
 * Purpose:
 *   High-level control logic for object tracking with servos.
 *   Computes direction and step sizes based on centroid position error.
 * 
 * Notes:
 *   - Deadzone: +/-60px horizontal, +/-50px vertical (prevents jitter near center)
 *   - Constant step size (200) for smooth motion without oscillation
 *   - Direction output: [3]=Left, [2]=Down, [1]=Up, [0]=Right
 *   - When disabled: motors idle (no movement commands)
 */

module motor_controller (
    // System
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,

    // Centroid tracking input
    input  logic [9:0]  centroid_x,
    input  logic [9:0]  centroid_y,
    input  logic        centroid_valid,

    // PWM outputs
    output logic        pwm_pan,
    output logic        pwm_tilt,
    
    // Status outputs
    output logic [3:0]  motor_direction,
    output logic [9:0]  error_x,
    output logic [9:0]  error_y
);

    // Frame center coordinates
    localparam CENTER_X = 10'd320;  // 640/2
    localparam CENTER_Y = 10'd240;  // 480/2
    
    // Deadzone - don't move if centroid is within this range of center
    localparam DEADZONE_X = 10'd60;  // ±60 pixels horizontal (increased further for stability)
    localparam DEADZONE_Y = 10'd40;  // ±50 pixels vertical (increased further for stability)
    
    // Constant step sizing for snappy response
    localparam integer CONST_STEP = 200;      // Fixed step size for both pan and tilt
    
    // Remove PD and filtering for simplicity
    
    // Return-to-center parameters (when motion disabled)
    localparam integer CENTER_STEP = 30;  // Slow step size for returning to center
    localparam integer CENTER_DEADZONE = 10; // Small deadzone to stop near center
    
    // Calculate offset from center
    logic signed [10:0] offset_x;  // Signed for direction
    logic signed [10:0] offset_y;
    
    assign offset_x = $signed({1'b0, centroid_x}) - $signed({1'b0, CENTER_X});
    assign offset_y = $signed({1'b0, centroid_y}) - $signed({1'b0, CENTER_Y});
    
    // Absolute values for deadzone comparison
    logic [9:0] abs_offset_x;
    logic [9:0] abs_offset_y;
    
    assign abs_offset_x = (offset_x[10]) ? -offset_x[9:0] : offset_x[9:0];
    assign abs_offset_y = (offset_y[10]) ? -offset_y[9:0] : offset_y[9:0];
    
    // Step size calculation (constant)
    logic [18:0] pan_step;
    logic [18:0] tilt_step;
    always_comb begin
        //Range of movements:
        
        //Horizonatal:
        ///0-60 Deadzone
        //60-320 -> Movement

        //61-100 100 step/clk
        //101-165 125 step/clk
        //166-200 150 step/clk
        //201-250 175 step/clk
        //251-320 200 step/clk


        //Vertical
        //0-40 Deadzone
        //40`-240 -> Movement

        //41-100 100
        //101-145 125
        //146-190 150
        //191-240 175


        if (abs_offset_x < 10'd100) begin
            pan_step = 19'd100;
        end
        else if (abs_offset_x < 10'd165) begin
            pan_step = 19'd125;
        end
        else if (abs_offset_x < 10'd200) begin
            pan_step = 19'd150;
        end
        else if (abs_offset_x < 10'd250) begin
            pan_step = 19'd175;
        end
        else begin
            pan_step = 19'd200;
        end

        if (abs_offset_y < 10'd100) begin
            tilt_step = 19'd100;
        end
        else if (abs_offset_y < 10'd145) begin
            tilt_step = 19'd125;
        end
        else if (abs_offset_y < 10'd190) begin
            tilt_step = 19'd150;
        end
        else begin
            tilt_step = 19'd175;
        end

        // pan_step  = abs_offset_x>>3;
        // tilt_step = abs_offset_y>>3;
    end
    
    logic [3:0] direction;
    logic driver_rst;
    
    // Current motor positions (read from motor_driver)
    logic [18:0] current_pan_threshold;
    logic [18:0] current_tilt_threshold;
    
    // Return-to-center mode
    logic return_to_center_mode;
    
    // No filtering logic needed for constant step
    
    always_comb begin
        direction = 4'b0000;  // Default: no movement
        driver_rst = 1'b0;
        return_to_center_mode = 1'b0;
        
        if (enable && centroid_valid) begin
            // Active tracking mode
            // Horizontal control
            if (abs_offset_x > DEADZONE_X) begin
                if (offset_x > 0) begin
                    direction[0] = 1'b1;  // Right
                end else begin
                    direction[3] = 1'b1;  // Left
                end
            end
            // Vertical control
            if (abs_offset_y > DEADZONE_Y) begin
                if (offset_y > 0) begin
                    direction[2] = 1'b1;  // Down
                end else begin
                    direction[1] = 1'b1;  // Up
                end
            end
        end
        // If not enabled or no valid centroid, direction stays 4'b0000
        // Motor driver's timeout will handle returning to center
    end

    // Instantiate motor_driver
    motor_driver u_motor_driver (
        .clk(clk),
        .rst(~rst_n),
        .enable(enable),
        .direction(direction),
        .pan_step(pan_step),
        .tilt_step(tilt_step),
        .pwm_pan(pwm_pan),
        .pwm_tilt(pwm_tilt),
        .pan_threshold(current_pan_threshold),   // Read current position
        .tilt_threshold(current_tilt_threshold)  // Read current position
    );

    // Export direction and error to top level
    assign motor_direction = direction;
    assign error_x = abs_offset_x;
    assign error_y = abs_offset_y;

endmodule
