/*
 * Motor Driver Module
 * 
 * Description:
 *   PWM motor driver for pan/tilt servos.
 *   Adjusts servo positions based on direction commands and step sizes.
 * 
 * Purpose:
 *   Low-level servo control with PWM signal generation.
 *   Maps direction commands to threshold updates for smooth servo motion.
 * 
 * Notes:
 *   - PWM cycle: 20ms (50Hz), pulse width: 1-2ms for 0-180 degrees
 *   - Direction encoding: [3]=Left, [2]=Down, [1]=Up, [0]=Right
 *   - When disabled: holds current position and forces PWM low
 */

module motor_driver(
    // System
    input  logic        clk,
    input  logic        rst,
    input  logic        enable,
    
    // Control inputs
    input  logic [3:0]  direction,
    input  logic [18:0] pan_step,
    input  logic [18:0] tilt_step,
    
    // PWM outputs
    output logic        pwm_pan,
    output logic        pwm_tilt,
    
    // Position feedback
    output logic [18:0] pan_threshold,
    output logic [18:0] tilt_threshold
);

    // Input Clock is 25MHz -> 40ns period
    // PWM Clock is 50Hz -> 20ms period
    // There are 20ms/40ns = 500_000 clock pulses in one PWM cycle
    // Pulse should be between 1ms to 2ms for 0 to 180 degrees
    // 1ms = 25_000 clock pulses
    // 2ms = 50_000 clock pulses

    localparam integer PWM_CYCLE = 500_000;

    // localparam integer PWM_TILT_MIN = 19_300; //0967
    // localparam integer PWM_TILT_CENTER = 28_000;
    // localparam integer PWM_TILT_MAX = 36_700; //11EB

    // localparam integer PWM_PAN_MIN =11_300; //05E8
    // localparam integer PWM_PAN_CENTER = 36_650;
    // localparam integer PWM_PAN_MAX = 62_000; //1E39

    localparam integer PWM_TILT_MIN = 9_000;
    localparam integer PWM_TILT_CENTER = 15_000;
    localparam integer PWM_TILT_MAX = 22_500;

    localparam integer PWM_PAN_MIN = 16_000;
    localparam integer PWM_PAN_CENTER = 40_500;
    localparam integer PWM_PAN_MAX = 65_000;


    logic [18:0] counter;
    logic pwm_clk;
    logic pwm_clk_prev;
    logic pwm_clk_edge;
    
    // Timeout counter to detect when disabled for too long (return to center)
    logic [23:0] disable_counter;  // ~16 seconds at 1MHz update rate
    localparam DISABLE_TIMEOUT = 24'd1000000;  // 1 second at 1MHz
    
    //TODO
    // [X] Modify to adjust simulataneous pan and tilt
    // [X] Remove lower and upper bounds to detect true range
    // [X] Find threshold reset bug - Fixed with proper pwm_clk edge detection
    // [X] Speed up motor
    // [X] Add timeout to return to center when stuck
    
    // Detect positive edge of pwm_clk for threshold updates
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            pwm_clk_prev <= 1'b0;
        end else begin
            pwm_clk_prev <= pwm_clk;
        end
    end
    
    assign pwm_clk_edge = pwm_clk && !pwm_clk_prev;
    
    //Input Direction -> Threshold Mapping
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            pan_threshold  <= PWM_PAN_CENTER;
            tilt_threshold <= PWM_TILT_CENTER;
            disable_counter <= 24'd0;
        end else if (pwm_clk_edge) begin  // Update on pwm_clk edge only
            if (enable && (direction != 4'b0000)) begin
                // Reset disable counter when actively moving
                disable_counter <= 24'd0;
                
                //Left and Right control - Pan
                if(direction[3] == 1)begin
                    pan_threshold <= (pan_threshold >= PWM_PAN_MAX - pan_step) ? PWM_PAN_MAX : pan_threshold + pan_step;
                end else if(direction[0] == 1)begin
                    pan_threshold <= (pan_threshold <= PWM_PAN_MIN + pan_step) ? PWM_PAN_MIN : pan_threshold - pan_step;
                end else begin
                    pan_threshold <= pan_threshold; 
                end
                
                //Up and Down control - Tilt
                if(direction[2] == 1) begin
                    tilt_threshold <= (tilt_threshold <= PWM_TILT_MIN + tilt_step) ? PWM_TILT_MIN : tilt_threshold - tilt_step;
                end
                else if(direction[1] == 1) begin
                    tilt_threshold <= (tilt_threshold >= PWM_TILT_MAX - tilt_step) ? PWM_TILT_MAX : tilt_threshold + tilt_step;
                end
                else begin
                    tilt_threshold <= tilt_threshold;
                end
            end else begin
                // When disabled or no direction command, increment timeout counter
                if (disable_counter < DISABLE_TIMEOUT) begin
                    disable_counter <= disable_counter + 24'd1;
                end else begin
                    // Timeout reached - slowly return to center to unstick
                    if (pan_threshold > PWM_PAN_CENTER) begin
                        pan_threshold <= pan_threshold - 19'd100;  // Slow drift to center
                    end else if (pan_threshold < PWM_PAN_CENTER) begin
                        pan_threshold <= pan_threshold + 19'd100;
                    end
                    
                    if (tilt_threshold > PWM_TILT_CENTER) begin
                        tilt_threshold <= tilt_threshold - 19'd100;
                    end else if (tilt_threshold < PWM_TILT_CENTER) begin
                        tilt_threshold <= tilt_threshold + 19'd100;
                    end
                end
            end
        end
    end

    //Counter Logic
    always_ff @(posedge clk or posedge rst) begin 
        if(rst) begin
            counter <= 19'b0;
            pwm_clk <= 1'b0;
        end else if(counter == PWM_CYCLE-1) begin
            counter <= 19'b0;
            pwm_clk <= ~pwm_clk;
        end
        else begin
            //Increment counter(unsigned addition)
            counter <= counter + 1;
        end
    end

    //PWM Output via Counter
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            pwm_pan  <= 1'b0;
            pwm_tilt <= 1'b0;
        end else if (!enable) begin
            pwm_pan  <= 1'b0;
            pwm_tilt <= 1'b0;
        end else begin
            pwm_pan  <= (counter < pan_threshold) ? 1'b1 : 1'b0;
            pwm_tilt <= (counter < tilt_threshold) ? 1'b1 : 1'b0;
        end
    end


   
endmodule