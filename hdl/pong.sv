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
 *   - Ball physics: bounces off top/bottom walls and paddles
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
    parameter BALL_SPEED_X          = 3,    // Pixels per frame
    parameter BALL_SPEED_Y          = 2     // Pixels per frame
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
    logic [9:0] Ball_X_Motion;
    logic [9:0] Ball_Y_Motion;
    logic       next_turn;
    logic       player_a_win;
    logic       player_b_win;
    logic [3:0] player_a_score;
    logic [3:0] player_b_score;
    logic       player_a_wins;
    logic       player_b_wins;

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
    // Ball Physics: Motion Calculation
    // ------------------------------------------------------------
    logic [9:0] Ball_X_Motion_next;
    logic [9:0] Ball_Y_Motion_next;
    logic [9:0] Ball_X_next;
    logic [9:0] Ball_Y_next;

    always_comb begin
        // Default: maintain current motion
        Ball_X_Motion_next = Ball_X_Motion;
        Ball_Y_Motion_next = Ball_Y_Motion;

        // Top wall collision
        if ((ball_y - BALL_SIZE) <= BALL_Y_MIN) begin
            Ball_Y_Motion_next = BALL_SPEED_Y[9:0];
        end
        // Bottom wall collision
        else if ((ball_y + BALL_SIZE) >= BALL_Y_MAX) begin
            Ball_Y_Motion_next = (~(BALL_SPEED_Y[9:0]) + 1'b1);  // Negate via 2's complement
        end

        // Right paddle collision
        if (((ball_x + BALL_SIZE) < (RIGHT_PADDLE_X - PADDLE_WIDTH) + BALL_SPEED_X) &&
            ((ball_x + BALL_SIZE) > (RIGHT_PADDLE_X - PADDLE_WIDTH) - BALL_SPEED_X) &&
            (ball_y > right_paddle_center - (PADDLE_HEIGHT >> 1)) &&
            (ball_y < right_paddle_center + (PADDLE_HEIGHT >> 1))) begin
            Ball_X_Motion_next = (~(BALL_SPEED_X[9:0]) + 1'b1);  // Bounce left
        end
        // Left paddle collision
        else if (((ball_x - BALL_SIZE) < (LEFT_PADDLE_X + PADDLE_WIDTH) + BALL_SPEED_X) &&
                 ((ball_x - BALL_SIZE) > (LEFT_PADDLE_X + PADDLE_WIDTH) - BALL_SPEED_X) &&
                 (ball_y > left_paddle_center - (PADDLE_HEIGHT >> 1)) &&
                 (ball_y < left_paddle_center + (PADDLE_HEIGHT >> 1))) begin
            Ball_X_Motion_next = BALL_SPEED_X[9:0];  // Bounce right
        end
    end

    assign Ball_X_next = ball_x + Ball_X_Motion_next;
    assign Ball_Y_next = ball_y + Ball_Y_Motion_next;

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
            Ball_X_Motion <= BALL_SPEED_X[9:0];
            Ball_Y_Motion <= BALL_SPEED_Y[9:0];
            player_a_win <= 1'b0;
            player_b_win <= 1'b0;
            player_a_score <= 4'd0;
            player_b_score <= 4'd0;
            player_a_wins <= 1'b0;
            player_b_wins <= 1'b0;
            next_turn <= 1'b0;
        end else if (enable && frame_start_edge) begin
            // Check for score conditions
            if (ball_x > RIGHT_LOSS_X) begin
                next_turn <= 1'b1;
                player_a_win <= 1'b1;
            end else if (ball_x < LEFT_LOSS_X) begin
                next_turn <= 1'b1;
                player_b_win <= 1'b1;
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
                    // Reset ball motion
                    Ball_X_Motion <= BALL_SPEED_X[9:0];
                    Ball_Y_Motion <= BALL_SPEED_Y[9:0];
                end else if (player_b_win) begin
                    if (player_b_score < 4'd7) begin
                        player_b_score <= player_b_score + 1'b1;
                        // Check for win condition
                        if ((player_b_score + 1'b1) >= 4'd7) begin
                            player_b_wins <= 1'b1;
                        end
                    end
                    player_b_win <= 1'b0;
                    // Reset ball motion
                    Ball_X_Motion <= BALL_SPEED_X[9:0];
                    Ball_Y_Motion <= BALL_SPEED_Y[9:0];
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

            // Green ball
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
