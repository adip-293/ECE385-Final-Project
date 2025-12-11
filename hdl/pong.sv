`timescale 1ns / 1ps

/*
 * Pong Game Module
 * 
 * Description:
 *   Implements a two-player pong game where hand positions control paddles.
 *   Left and right centroids control left and right paddles respectively.
 *   Centroid positions are displayed as red dots for visual feedback.
 * 
 * Purpose:
 *   Interactive game demonstrating dual centroid tracking and real-time control.
 *   Hand tracking via split centroid detection controls paddle movement.
 * 
 * Notes:
 *   - Ball physics: bounces off walls and paddles with angle-based deflection
 *   - Speed increases based on paddle collision count (every N collisions)
 *   - Paddle control: follows centroid Y position when valid
 *   - Score tracking: players score when ball passes opponent's paddle
 *   - Visual feedback: red dots show centroid positions, white paddles, green ball
 */

module pong #(
    parameter FRAME_WIDTH           = 640,
    parameter FRAME_HEIGHT          = 480,
    parameter PADDLE_WIDTH          = 5,
    parameter PADDLE_HEIGHT         = 80,
    parameter BALL_SIZE             = 8,
    parameter DOT_SIZE              = 6,
    parameter PADDLE_MARGIN         = 5,   // Distance from edge
    parameter CENTER_LINE_DOT_SPACING = 20, // Space between center line dots
    parameter CENTER_LINE_DOT_SIZE  = 4,
    parameter BALL_SPEED_X          = 3,    // Base pixels per frame
    parameter BALL_SPEED_Y          = 2,    // Base pixels per frame
    parameter SPEED_WAVE_INTERVAL  = 2,    // Increase speed every N paddle collisions
    parameter MAX_SPEED_X           = 8,    // Maximum X speed
    parameter MAX_SPEED_Y           = 6     // Maximum Y speed
)(
    // System
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,

    // VGA timing
    input  logic [9:0]  draw_x,
    input  logic [9:0]  draw_y,
    input  logic        vde,
    input  logic        frame_start,  // Pulse at start of new frame
    
    // Reset/rematch button
    input  logic        rematch_pressed,  // btn[0] pressed for rematch
    
    // Random number input (from potentiometer noise)
    input  logic [3:0]  rng,              // 4-bit random value from pot[3:0]

    // Centroid inputs for paddle control
    input  logic [9:0]  left_centroid_y,   // Left player paddle Y position
    input  logic        left_centroid_valid,
    input  logic [9:0]  right_centroid_y,  // Right player paddle Y position
    input  logic        right_centroid_valid,

    // Centroid positions for red dot indicators
    input  logic [9:0]  left_centroid_x,
    input  logic [9:0]  right_centroid_x,

    // Input pixels
    input  logic [2:0]  pixel_red_in,
    input  logic [2:0]  pixel_green_in,
    input  logic [2:0]  pixel_blue_in,

    // Output pixels
    output logic [2:0]  pixel_red_out,
    output logic [2:0]  pixel_green_out,
    output logic [2:0]  pixel_blue_out,
    
    // Score outputs for display
    output logic [3:0]  player_a_score,
    output logic [3:0]  player_b_score,
    
    // Win status outputs
    output logic        player_a_wins,
    output logic        player_b_wins
);

    // ------------------------------------------------------------
    // Local Parameters
    // ------------------------------------------------------------
    localparam logic [9:0] LEFT_PADDLE_X  = PADDLE_MARGIN;
    localparam logic [9:0] RIGHT_PADDLE_X = FRAME_WIDTH - PADDLE_MARGIN;
    localparam logic [9:0] LEFT_LOSS_X    = 10;   // Ball passes here = player B wins
    localparam logic [9:0] RIGHT_LOSS_X   = 630;  // Ball passes here = player A wins
    localparam logic [9:0] CENTER_LINE_R  = 323;
    localparam logic [9:0] CENTER_LINE_L  = 317;
    localparam logic [9:0] BALL_X_CENTER  = 320;
    localparam logic [9:0] BALL_Y_CENTER  = 240;
    localparam logic [9:0] BALL_X_MIN     = 0;
    localparam logic [9:0] BALL_X_MAX     = 639;
    localparam logic [9:0] BALL_Y_MIN     = 0;
    localparam logic [9:0] BALL_Y_MAX     = 479;

    // ------------------------------------------------------------
    // Game State Registers
    // ------------------------------------------------------------
    logic [9:0] ball_x, ball_y;
    logic [9:0] left_paddle_center;
    logic [9:0] right_paddle_center;
    logic signed [10:0] Ball_X_Motion;  // Signed for direction
    logic signed [10:0] Ball_Y_Motion;  // Signed for direction
    logic       next_turn;
    logic       player_a_win;
    logic       player_b_win;
    logic [3:0] player_a_score;
    logic [3:0] player_b_score;
    logic       player_a_wins;
    logic       player_b_wins;
    
    // Speed tracking system
    logic [4:0] paddle_collision_count;  // Count total paddle collisions
    logic [3:0] current_speed_level;      // Current speed wave (0-15)
    logic [9:0] current_speed_x;          // Current X speed magnitude
    logic [9:0] current_speed_y;          // Current Y speed magnitude
    logic       paddle_collision_this_frame;  // Collision detected this frame

    // ------------------------------------------------------------
    // Frame Start Edge Detection
    // ------------------------------------------------------------
    logic frame_start_prev;
    logic frame_start_edge;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_start_prev <= 1'b0;
        end else begin
            frame_start_prev <= frame_start;
        end
    end
    
    assign frame_start_edge = frame_start & ~frame_start_prev;

    // ------------------------------------------------------------
    // Speed Calculation
    // ------------------------------------------------------------
    // Compute current speed magnitude based on collision count
    always_comb begin
        // Speed level increases every SPEED_WAVE_INTERVAL collisions
        current_speed_level = paddle_collision_count / SPEED_WAVE_INTERVAL;
        
        // Calculate speed magnitude: base speed + level increment, clamped to maximum
        if ((BALL_SPEED_X + current_speed_level) > MAX_SPEED_X) begin
            current_speed_x = MAX_SPEED_X;
        end else begin
            current_speed_x = BALL_SPEED_X + current_speed_level;
        end
        
        if ((BALL_SPEED_Y + current_speed_level) > MAX_SPEED_Y) begin
            current_speed_y = MAX_SPEED_Y;
        end else begin
            current_speed_y = BALL_SPEED_Y + current_speed_level;
        end
    end

    // ------------------------------------------------------------
    // Ball Physics: Motion Calculation
    // ------------------------------------------------------------
    logic signed [10:0] Ball_X_Motion_next;
    logic signed [10:0] Ball_Y_Motion_next;
    logic [9:0] Ball_X_next;
    logic [9:0] Ball_Y_next;
    
    // Paddle collision detection with angle-based deflection
    logic signed [10:0] paddle_relative_y;  // Ball Y relative to paddle center
    logic signed [10:0] angle_factor;       // Calculated deflection factor
    logic signed [10:0] deflection_y;       // Y velocity adjustment based on hit position
    logic signed [10:0] paddle_half_height;  // Half paddle height for calculations
    
    // Random initial ball motion calculation
    logic signed [10:0] random_x_motion;
    logic signed [10:0] random_y_motion;
    logic [9:0] speed_y_plus_one;  // BALL_SPEED_Y + 1 for variation
    
    assign speed_y_plus_one = BALL_SPEED_Y + 1;
    
    always_comb begin
        // Randomize X direction using rng[0]
        if (rng[0]) begin
            random_x_motion = $signed({1'b0, BALL_SPEED_X[9:0]});
        end else begin
            random_x_motion = -$signed({1'b0, BALL_SPEED_X[9:0]});
        end
        
        // Randomize Y direction and add slight speed variation using rng[3:1]
        case (rng[2:1])
            2'b00: begin
                random_y_motion = -$signed({1'b0, BALL_SPEED_Y});
            end
            2'b01: begin
                random_y_motion = $signed({1'b0, BALL_SPEED_Y});
            end
            2'b10: begin
                if (rng[3]) begin
                    random_y_motion = -$signed({1'b0, speed_y_plus_one});
                end else begin
                    random_y_motion = -$signed({1'b0, BALL_SPEED_Y});
                end
            end
            2'b11: begin
                if (rng[3]) begin
                    random_y_motion = $signed({1'b0, speed_y_plus_one});
                end else begin
                    random_y_motion = $signed({1'b0, BALL_SPEED_Y});
                end
            end
        endcase
    end

    always_comb begin
        // Default: maintain current motion
        Ball_X_Motion_next = Ball_X_Motion;
        Ball_Y_Motion_next = Ball_Y_Motion;
        paddle_collision_this_frame = 1'b0;
        deflection_y = 11'sd0;

        // Top wall collision
        if ((ball_y - BALL_SIZE) <= BALL_Y_MIN) begin
            Ball_Y_Motion_next = $signed({1'b0, current_speed_y[9:0]});
        end
        // Bottom wall collision
        else if ((ball_y + BALL_SIZE) >= BALL_Y_MAX) begin
            Ball_Y_Motion_next = -$signed({1'b0, current_speed_y[9:0]});
        end
        // Right paddle collision with angle-based deflection
        else if ((ball_x + BALL_SIZE) >= (RIGHT_PADDLE_X - PADDLE_WIDTH) &&
                 (ball_x - BALL_SIZE) <= (RIGHT_PADDLE_X + PADDLE_WIDTH) &&
                 (ball_y > right_paddle_center - (PADDLE_HEIGHT >> 1) - BALL_SIZE) &&
                 (ball_y < right_paddle_center + (PADDLE_HEIGHT >> 1) + BALL_SIZE) &&
                 (Ball_X_Motion > 0)) begin  // Only if moving right
            
            paddle_collision_this_frame = 1'b1;
            
            // Calculate relative position on paddle (-PADDLE_HEIGHT/2 to +PADDLE_HEIGHT/2)
            paddle_relative_y = $signed({1'b0, ball_y}) - $signed({1'b0, right_paddle_center});
            
            // Calculate deflection based on hit position (simplified, no division)
            // Map relative_y (-PADDLE_HEIGHT/2 to +PADDLE_HEIGHT/2) to deflection
            // Use multiplication and shift instead of division for efficiency
            paddle_half_height = $signed({1'b0, (PADDLE_HEIGHT >> 1)});
            
            // Scale relative_y to deflection range: multiply by speed, divide by paddle_half
            // Approximate: deflection = (relative_y * current_speed_y) / paddle_half_height
            // Use: deflection = (relative_y * current_speed_y) >> log2(paddle_half_height)
            // For PADDLE_HEIGHT=80, half=40, log2(40)≈5.3, use 5
            if (paddle_relative_y > 0) begin
                // Hit on bottom half - deflect downward
                angle_factor = (paddle_relative_y * $signed({1'b0, current_speed_y})) >>> 5;  // Approximate division by 40
                if (angle_factor > $signed({1'b0, current_speed_y})) begin
                    angle_factor = $signed({1'b0, current_speed_y});
                end
                deflection_y = angle_factor;
            end else begin
                // Hit on top half - deflect upward
                angle_factor = (paddle_relative_y * $signed({1'b0, current_speed_y})) >>> 5;
                if (angle_factor < -$signed({1'b0, current_speed_y})) begin
                    angle_factor = -$signed({1'b0, current_speed_y});
                end
                deflection_y = angle_factor;
            end
            
            // Reverse X direction and add deflection
            Ball_X_Motion_next = -$signed(current_speed_x[9:0]);
            // Add angle-based Y deflection (preserve some existing Y motion for spin effect)
            Ball_Y_Motion_next = deflection_y + (Ball_Y_Motion >>> 1);  // Mix old and new
            
            // Clamp Y motion to max speed
            if (Ball_Y_Motion_next > $signed({1'b0, current_speed_y})) begin
                Ball_Y_Motion_next = $signed({1'b0, current_speed_y});
            end else if (Ball_Y_Motion_next < -$signed({1'b0, current_speed_y})) begin
                Ball_Y_Motion_next = -$signed({1'b0, current_speed_y});
            end
        end
        // Left paddle collision with angle-based deflection
        else if ((ball_x - BALL_SIZE) <= (LEFT_PADDLE_X + PADDLE_WIDTH) &&
                 (ball_x + BALL_SIZE) >= (LEFT_PADDLE_X - PADDLE_WIDTH) &&
                 (ball_y > left_paddle_center - (PADDLE_HEIGHT >> 1) - BALL_SIZE) &&
                 (ball_y < left_paddle_center + (PADDLE_HEIGHT >> 1) + BALL_SIZE) &&
                 (Ball_X_Motion < 0)) begin  // Only if moving left
            
            paddle_collision_this_frame = 1'b1;
            
            // Calculate relative position on paddle
            paddle_relative_y = $signed({1'b0, ball_y}) - $signed({1'b0, left_paddle_center});
            
            // Calculate deflection (same as right paddle)
            paddle_half_height = $signed({1'b0, (PADDLE_HEIGHT >> 1)});
            
            if (paddle_relative_y > 0) begin
                angle_factor = (paddle_relative_y * $signed({1'b0, current_speed_y})) >>> 5;
                if (angle_factor > $signed({1'b0, current_speed_y})) begin
                    angle_factor = $signed({1'b0, current_speed_y});
                end
                deflection_y = angle_factor;
            end else begin
                angle_factor = (paddle_relative_y * $signed({1'b0, current_speed_y})) >>> 5;
                if (angle_factor < -$signed({1'b0, current_speed_y})) begin
                    angle_factor = -$signed({1'b0, current_speed_y});
                end
                deflection_y = angle_factor;
            end
            
            // Reverse X direction and add deflection
            Ball_X_Motion_next = $signed(current_speed_x[9:0]);
            Ball_Y_Motion_next = deflection_y + (Ball_Y_Motion >>> 1);
            
            // Clamp Y motion
            if (Ball_Y_Motion_next > $signed({1'b0, current_speed_y})) begin
                Ball_Y_Motion_next = $signed({1'b0, current_speed_y});
            end else if (Ball_Y_Motion_next < -$signed({1'b0, current_speed_y})) begin
                Ball_Y_Motion_next = -$signed({1'b0, current_speed_y});
            end
        end
    end

    // Update ball position with signed motion and wall clamping
    always_comb begin
        // X position update
        if (Ball_X_Motion_next[10]) begin  // Negative (moving left)
            Ball_X_next = ball_x - (~Ball_X_Motion_next[9:0] + 1'b1);
        end else begin
            Ball_X_next = ball_x + Ball_X_Motion_next[9:0];
        end
        
        // Clamp X to screen bounds
        if (Ball_X_next < BALL_SIZE) begin
            Ball_X_next = BALL_SIZE;
        end else if (Ball_X_next > (BALL_X_MAX - BALL_SIZE)) begin
            Ball_X_next = BALL_X_MAX - BALL_SIZE;
        end
        
        // Y position update
        if (Ball_Y_Motion_next[10]) begin  // Negative (moving up)
            Ball_Y_next = ball_y - (~Ball_Y_Motion_next[9:0] + 1'b1);
        end else begin
            Ball_Y_next = ball_y + Ball_Y_Motion_next[9:0];
        end
        
        // Clamp Y to screen bounds (prevents ball from vanishing at top/bottom)
        if (Ball_Y_next < BALL_SIZE) begin
            Ball_Y_next = BALL_SIZE;
        end else if (Ball_Y_next > (BALL_Y_MAX - BALL_SIZE)) begin
            Ball_Y_next = BALL_Y_MAX - BALL_SIZE;
        end
    end

    // ------------------------------------------------------------
    // Rematch button edge detection
    // ------------------------------------------------------------
    logic rematch_pressed_prev;
    logic rematch_edge;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rematch_pressed_prev <= 1'b0;
        end else begin
            rematch_pressed_prev <= rematch_pressed;
        end
    end
    
    assign rematch_edge = rematch_pressed & ~rematch_pressed_prev;

    // ------------------------------------------------------------
    // Game State Update (on frame start)
    // ------------------------------------------------------------
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n || rematch_edge) begin
            // Reset to initial positions
            ball_x <= BALL_X_CENTER;
            ball_y <= BALL_Y_CENTER;
            left_paddle_center <= FRAME_HEIGHT >> 1;
            right_paddle_center <= FRAME_HEIGHT >> 1;
            
            // Randomize initial ball direction using RNG
            Ball_X_Motion <= random_x_motion;
            Ball_Y_Motion <= random_y_motion;
            player_a_win <= 1'b0;
            player_b_win <= 1'b0;
            player_a_score <= 4'd0;
            player_b_score <= 4'd0;
            player_a_wins <= 1'b0;
            player_b_wins <= 1'b0;
            next_turn <= 1'b0;
            paddle_collision_count <= 5'd0;
            current_speed_level <= 4'd0;
        end else if (enable && frame_start_edge) begin
            // Increment collision counter when paddle hit detected
            if (paddle_collision_this_frame) begin
                paddle_collision_count <= paddle_collision_count + 5'd1;
            end
            
            // Check for score conditions
            if (ball_x > RIGHT_LOSS_X) begin
                next_turn <= 1'b1;
                player_a_win <= 1'b1;
                // Reset collision count on score (speed resets)
                paddle_collision_count <= 5'd0;
            end else if (ball_x < LEFT_LOSS_X) begin
                next_turn <= 1'b1;
                player_b_win <= 1'b1;
                // Reset collision count on score
                paddle_collision_count <= 5'd0;
            end

            // Handle new turn (reset ball after score)
            if (next_turn) begin
                ball_x <= BALL_X_CENTER;
                ball_y <= BALL_Y_CENTER;
                next_turn <= 1'b0;

                // Update scores (check before resetting win flags)
                if (player_a_win) begin
                    if (player_a_score < 4'd7) begin
                        player_a_score <= player_a_score + 1'b1;
                        // Check for win condition
                        if ((player_a_score + 1'b1) >= 4'd7) begin
                            player_a_wins <= 1'b1;
                        end
                    end
                    player_a_win <= 1'b0;
                    // Randomize ball motion for new turn
                    Ball_X_Motion <= random_x_motion;
                    Ball_Y_Motion <= random_y_motion;
                end else if (player_b_win) begin
                    if (player_b_score < 4'd7) begin
                        player_b_score <= player_b_score + 1'b1;
                        // Check for win condition
                        if ((player_b_score + 1'b1) >= 4'd7) begin
                            player_b_wins <= 1'b1;
                        end
                    end
                    player_b_win <= 1'b0;
                    // Randomize ball motion for new turn
                    Ball_X_Motion <= random_x_motion;
                    Ball_Y_Motion <= random_y_motion;
                end
            end else begin
                // Normal game update
                if (!player_a_wins && !player_b_wins) begin
                    Ball_X_Motion <= Ball_X_Motion_next;
			        Ball_Y_Motion <= Ball_Y_Motion_next; 
                    ball_x <= Ball_X_next;
                    ball_y <= Ball_Y_next;
                end
                else begin
                    Ball_X_Motion <= 0;
                    Ball_Y_Motion <= 0;
                    ball_x <= BALL_X_CENTER;
                    ball_y <= BALL_Y_CENTER;
                end
                // Update paddle positions from centroids (mirrored control)
                // Left player's hand controls right paddle
                if (left_centroid_valid) begin
                    right_paddle_center <= left_centroid_y;
                end
                // Right player's hand controls left paddle
                if (right_centroid_valid) begin
                    left_paddle_center <= right_centroid_y;
                end
            end
		end  
    end

    // ------------------------------------------------------------
    // Rendering Logic
    // ------------------------------------------------------------
    logic dot_pixel;
    logic paddle_pixel;
    logic center_line_pixel;
    logic ball_pixel;
    
    // Mirrored centroid positions for display
    logic [9:0] left_centroid_x_mirrored;
    logic [9:0] right_centroid_x_mirrored;
    
    always_comb begin
        // Mirror X coordinates: left side appears on right, right side appears on left
        left_centroid_x_mirrored  = FRAME_WIDTH - left_centroid_x;
        right_centroid_x_mirrored = FRAME_WIDTH - right_centroid_x;
    end

    always_ff @(posedge clk) begin
        if (!enable || !vde) begin
            dot_pixel <= 1'b0;
            paddle_pixel <= 1'b0;
            center_line_pixel <= 1'b0;
            ball_pixel <= 1'b0;
        end else begin
            // Red dots for centroids (mirrored positions)
            // Left player's hand displayed on right side
            dot_pixel <= (left_centroid_valid &&
                          (draw_x < left_centroid_x_mirrored + DOT_SIZE) &&
                          (draw_x > left_centroid_x_mirrored - DOT_SIZE) &&
                          (draw_y < left_centroid_y + DOT_SIZE) &&
                          (draw_y > left_centroid_y - DOT_SIZE)) ||
                         // Right player's hand displayed on left side
                         (right_centroid_valid &&
                          (draw_x < right_centroid_x_mirrored + DOT_SIZE) &&
                          (draw_x > right_centroid_x_mirrored - DOT_SIZE) &&
                          (draw_y < right_centroid_y + DOT_SIZE) &&
                          (draw_y > right_centroid_y - DOT_SIZE));

            // White paddles
            paddle_pixel <= ((draw_x < LEFT_PADDLE_X + PADDLE_WIDTH) &&
                             (draw_x > LEFT_PADDLE_X - PADDLE_WIDTH) &&
                             (draw_y < left_paddle_center + (PADDLE_HEIGHT >> 1)) &&
                             (draw_y > left_paddle_center - (PADDLE_HEIGHT >> 1))) ||
                            ((draw_x < RIGHT_PADDLE_X + PADDLE_WIDTH) &&
                             (draw_x > RIGHT_PADDLE_X - PADDLE_WIDTH) &&
                             (draw_y < right_paddle_center + (PADDLE_HEIGHT >> 1)) &&
                             (draw_y > right_paddle_center - (PADDLE_HEIGHT >> 1)));

            // Red center line (dashed pattern)
            center_line_pixel <= (draw_x < CENTER_LINE_R) &&
                                 (draw_x > CENTER_LINE_L) &&
                                 (draw_y[4] == 1'b1);  // Every 16 pixels

            // Green ball (circular approximation with square)
            ball_pixel <= (draw_x < ball_x + BALL_SIZE) &&
                          (draw_x > ball_x - BALL_SIZE) &&
                          (draw_y < ball_y + BALL_SIZE) &&
                          (draw_y > ball_y - BALL_SIZE);
        end
    end

    // ------------------------------------------------------------
    // Pixel Output Mux
    // ------------------------------------------------------------
    always_comb begin
        if (!enable || !vde) begin
            // Pass through input when disabled
            pixel_red_out   = pixel_red_in;
            pixel_green_out = pixel_green_in;
            pixel_blue_out  = pixel_blue_in;
        end else if (ball_pixel) begin
            // White ball (highest priority - render last)
            pixel_red_out   = 3'b111;
            pixel_green_out = 3'b111;
            pixel_blue_out  = 3'b111;
        end else if (dot_pixel) begin
            // Red dots for centroids
            pixel_red_out   = 3'b111;
            pixel_green_out = 3'b000;
            pixel_blue_out  = 3'b000;
        end else if (paddle_pixel && !dot_pixel) begin
            // White paddles
            pixel_red_out   = 3'b111;
            pixel_green_out = 3'b111;
            pixel_blue_out  = 3'b111;
        end else if (center_line_pixel) begin
            // Gray center line
            pixel_red_out   = 3'b011;
            pixel_green_out = 3'b011;
            pixel_blue_out  = 3'b011;
        end else begin
            // Pass through background (including score text)
            pixel_red_out   = pixel_red_in;
            pixel_green_out = pixel_green_in;
            pixel_blue_out  = pixel_blue_in;
        end
end

endmodule

