/*
 * Text Overlay Module
 * 
 * Description:
 *   Displays the current processing state name on the bottom row of the screen
 *   using the IBM Code Page 437 font (8x16 pixels per character).
 * 
 * Purpose:
 *   - Render state name text at bottom of display (row 29 of 30)
 *   - Handle character ROM lookups for each glyph
 *   - Overlay white text (3'b111) on passthrough background
 * 
 * Notes:
 *   - Text displayed at y >= 464 (last glyph row)
 *   - Supports state names, gesture indicators, and mode flags
 *   - Background pixels pass through when not rendering text
 */
module text_overlay (
    input  logic        clk,
    input  logic        rst_n,
    input  logic [4:0]  current_state,    // 5-bit state (0-19)
    input  logic        force_color,      // Force color override
    input  logic        force_grayscale,  // Force grayscale override
    input  logic [1:0]  gesture_code,     // 0=none,1=FIST,2=OPEN,3=WAVE (latched per frame)
    input  logic [3:0]  pong_player_a_score,  // Player A score (0-7) for large display
    input  logic [3:0]  pong_player_b_score,  // Player B score (0-7) for large display
    input  logic        pong_player_a_wins,   // Player A wins flag
    input  logic        pong_player_b_wins,   // Player B wins flag
    
    // VGA timing
    input  logic [9:0]  draw_x,
    input  logic [9:0]  draw_y,
    input  logic        vde,
    
    // Input pixels
    input  logic [2:0]  pixel_red_in,
    input  logic [2:0]  pixel_green_in,
    input  logic [2:0]  pixel_blue_in,
    
    // Output pixels
    output logic [2:0]  pixel_red_out,
    output logic [2:0]  pixel_green_out,
    output logic [2:0]  pixel_blue_out
);

    // Text overlay parameters
    localparam TEXT_ROW_START = 464;    // Start of last glyph row (480 - 16)
    localparam TEXT_COL_START = 0;      // Left margin (8 pixels)
    localparam CHAR_WIDTH = 8;          // Character width in pixels
    localparam CHAR_HEIGHT = 16;        // Character height in pixels
    localparam MAX_TEXT_LEN = 16;       // Maximum text length
    
    // Text color (white)
    localparam logic [2:0] TEXT_RED   = 3'b111;
    localparam logic [2:0] TEXT_GREEN = 3'b111;
    localparam logic [2:0] TEXT_BLUE  = 3'b111;
    
    // State name strings (as ASCII codes)
    // Reordered to place DILATE before CENTROID for sequential morphology
    // Indices must match control_unit enumeration values (0..21)
    logic [7:0] state_text [0:21][0:MAX_TEXT_LEN-1];
    
    // Initialize state text strings
    initial begin
        // State 0: RAW
        state_text[0] = '{8'd82, 8'd65, 8'd87, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "RAW             "
        // State 1: CHANNEL_MODE
        state_text[1] = '{8'd67, 8'd72, 8'd65, 8'd78, 8'd78, 8'd69, 8'd76, 8'd32, 8'd77, 8'd79, 8'd68, 8'd69, 8'd32, 8'd32, 8'd32, 8'd32};  // "CHANNEL MODE    "
        // State 2: GRAYSCALE
        state_text[2] = '{8'd71, 8'd82, 8'd65, 8'd89, 8'd83, 8'd67, 8'd65, 8'd76, 8'd69, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "GRAYSCALE       "
        // State 3: THRESHOLD
        state_text[3] = '{8'd84, 8'd72, 8'd82, 8'd69, 8'd83, 8'd72, 8'd79, 8'd76, 8'd68, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "THRESHOLD       "
        // State 4: TEMPORAL
        state_text[4] = '{8'd84, 8'd69, 8'd77, 8'd80, 8'd79, 8'd82, 8'd65, 8'd76, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "TEMPORAL        "
        // State 5: MEDIAN
        state_text[5] = '{8'd77, 8'd69, 8'd68, 8'd73, 8'd65, 8'd78, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "MEDIAN          "
        // State 6: GAUSSIAN
        state_text[6] = '{8'd71, 8'd65, 8'd85, 8'd83, 8'd83, 8'd73, 8'd65, 8'd78, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "GAUSSIAN        "
        // State 7: SOBEL
        state_text[7] = '{8'd83, 8'd79, 8'd66, 8'd69, 8'd76, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "SOBEL           "
        // State 8: SHARPEN
        state_text[8] = '{8'd83, 8'd72, 8'd65, 8'd82, 8'd80, 8'd69, 8'd78, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "SHARPEN         "
        // State 9: EMBOSS
        state_text[9] = '{8'd69, 8'd77, 8'd66, 8'd79, 8'd83, 8'd83, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "EMBOSS          "
        // State 10: SKIN
        state_text[10] = '{8'd83, 8'd75, 8'd73, 8'd78, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32};  // "SKIN            "
        // State 11: SPATIAL
        state_text[11] = '{8'd83, 8'd80, 8'd65, 8'd84, 8'd73, 8'd65, 8'd76, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32}; // "SPATIAL         "
        // State 12: ERODE
        state_text[12] = '{8'd69, 8'd82, 8'd79, 8'd68, 8'd69, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32}; // "ERODE           "
        // State 13: DILATE
        state_text[13] = '{8'd68, 8'd73, 8'd76, 8'd65, 8'd84, 8'd69, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32}; // "DILATE          "
        // State 14: CENTROID
        state_text[14] = '{8'd67, 8'd69, 8'd78, 8'd84, 8'd82, 8'd79, 8'd73, 8'd68, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32}; // "CENTROID        "
        // State 15: BLOB FILTER
        state_text[15] = '{8'd66, 8'd76, 8'd79, 8'd66, 8'd32, 8'd70, 8'd73, 8'd76, 8'd84, 8'd69, 8'd82, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32}; // "BLOB FILTER     "
        // State 16: SKIN MOTION
        state_text[16] = '{8'd83, 8'd75, 8'd73, 8'd78, 8'd32, 8'd77, 8'd79, 8'd84, 8'd73, 8'd79, 8'd78, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32}; // "SKIN MOTION     "
        // State 17: COLOR THRESH
        state_text[17] = '{8'd67, 8'd79, 8'd76, 8'd79, 8'd82, 8'd32, 8'd84, 8'd72, 8'd82, 8'd69, 8'd83, 8'd72, 8'd32, 8'd32, 8'd32, 8'd32}; // "COLOR THRESH    "
        // State 18: COLOR TRACK
        state_text[18] = '{8'd67, 8'd79, 8'd76, 8'd79, 8'd82, 8'd32, 8'd84, 8'd82, 8'd65, 8'd67, 8'd75, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32}; // "COLOR TRACK     "
        // State 19: GESTURE
        state_text[19] = '{8'd71, 8'd69, 8'd83, 8'd84, 8'd85, 8'd82, 8'd69, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32}; // "GESTURE         "
        // State 20: DUAL CENTROID
        state_text[20] = '{8'd68, 8'd85, 8'd65, 8'd76, 8'd32, 8'd67, 8'd69, 8'd78, 8'd84, 8'd82, 8'd79, 8'd73, 8'd68, 8'd32, 8'd32, 8'd32}; // "DUAL CENTROID   "
        // State 21: PONG
        state_text[21] = '{8'd80, 8'd79, 8'd78, 8'd71, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32, 8'd32}; // "PONG            "
    end
    
    // Font ROM interface
    logic [10:0] font_addr;
    logic [7:0]  font_data;
    
    font_rom font (
        .addr(font_addr),
        .data(font_data)
    );
    
    // Calculate text rendering parameters
    logic in_text_region;
    logic [3:0] char_index;       // Which character in the string (0-15)
    logic [2:0] pixel_col;        // Which pixel column within character (0-7)
    logic [3:0] pixel_row;        // Which pixel row within character (0-15)
    logic [7:0] current_char;     // ASCII code of current character
    logic       text_pixel_on;    // Whether current pixel is part of text
    
    // Determine if we're in the text region
    assign in_text_region = (draw_y >= TEXT_ROW_START) && (draw_y < TEXT_ROW_START + CHAR_HEIGHT) && 
                           (draw_x >= TEXT_COL_START) && (draw_x < TEXT_COL_START + (MAX_TEXT_LEN * CHAR_WIDTH));
    
    // Calculate position within text
    assign pixel_row = draw_y - TEXT_ROW_START;
    assign char_index = (draw_x - TEXT_COL_START) / CHAR_WIDTH;
    assign pixel_col = (draw_x - TEXT_COL_START) % CHAR_WIDTH;
    
    // Determine which state text to display
    // If override is active, show "RAW" or "GRAYSCALE" instead of current state
    logic [4:0] display_state;
    always_comb begin
        if (force_color) begin
            display_state = 5'd0;  // Show "RAW"
        end else if (force_grayscale) begin
            display_state = 5'd2;  // Show "GRAYSCALE"
        end else begin
            display_state = current_state;  // Show current state
        end
    end
    
    // Gesture words for bottom-right overlay (FIST, OPEN, WAVE)
    logic [7:0] gesture_word [0:3][0:3];
    initial begin
        gesture_word[0] = '{8'd78, 8'd79, 8'd78, 8'd69}; // NONE
        gesture_word[1] = '{8'd70, 8'd73, 8'd83, 8'd84}; // FIST
        gesture_word[2] = '{8'd79, 8'd80, 8'd69, 8'd78}; // OPEN
        gesture_word[3] = '{8'd87, 8'd65, 8'd86, 8'd69}; // WAVE
    end

    // Safe state index
    logic [4:0] safe_display_state;
    assign safe_display_state = (display_state > 5'd21) ? 5'd0 : display_state;

    // Always take characters from state text row (no inline gesture word insertion now)
    always_comb begin
        current_char = state_text[safe_display_state][char_index];
    end

    // Bottom-right gesture overlay region (4 chars wide)
    localparam GESTURE_CHARS = 4;
    localparam integer SCREEN_WIDTH = 640; // fixed VGA width
    localparam [9:0] GESTURE_X_START = SCREEN_WIDTH - (GESTURE_CHARS * CHAR_WIDTH); // 640 - 32 = 608
    logic in_gesture_region;
    logic [2:0] gesture_pixel_col;
    logic [2:0] gesture_char_index; // 0..3
    logic [7:0] gesture_char;
    logic [10:0] gesture_font_addr;
    logic [7:0]  gesture_font_data;
    logic        gesture_text_on;

    // Check gesture region (same row as main text)
    assign in_gesture_region = (draw_y >= TEXT_ROW_START) && (draw_y < TEXT_ROW_START + CHAR_HEIGHT) &&
                               (draw_x >= GESTURE_X_START) && (draw_x < GESTURE_X_START + (GESTURE_CHARS * CHAR_WIDTH));
    assign gesture_char_index = (draw_x - GESTURE_X_START) / CHAR_WIDTH;
    assign gesture_pixel_col  = (draw_x - GESTURE_X_START) % CHAR_WIDTH;
    assign gesture_char       = gesture_word[gesture_code][gesture_char_index];
    assign gesture_font_addr  = {gesture_char[6:0], pixel_row};

    // Reuse same font ROM (shared) - sequential access ok for combinational read of one address per pixel
    // We'll mux address: priority is scores > gesture > main text
    // NOTE: Because font ROM is 1-port, pick address based on priority
    logic [10:0] final_font_addr;
    always_comb begin
        if (use_score_font) begin
            final_font_addr = score_font_addr_use;
        end else if ((gesture_code != 2'd0) && in_gesture_region) begin
            final_font_addr = gesture_font_addr;
        end else begin
            final_font_addr = {current_char[6:0], pixel_row};
        end
    end
    assign font_addr = final_font_addr;

    // font_data already driven via final_font_addr; derive per-region pixel bits
    assign text_pixel_on     = font_data[7 - pixel_col];
    assign gesture_text_on   = font_data[7 - gesture_pixel_col];
    // (gesture_text_on computed above; no second font_addr assignment to avoid overriding gesture region)
    
    // Black background for all states except RAW (0) and GRAYSCALE (2)
    logic black_background;
    always_comb begin
        black_background = (safe_display_state != 5'd0) && (safe_display_state != 5'd2);
    end
    
    // ------------------------------------------------------------
    // Large Score Display for PONG State
    // ------------------------------------------------------------
    localparam integer SCORE_SCALE = 16;  // 16x scale factor (128x256 pixels per digit)
    localparam integer SCORE_SCALE_LOG2 = 4;  // log2(16) for right shift
    localparam [9:0] SCORE_CHAR_WIDTH = CHAR_WIDTH * SCORE_SCALE;   // 128 pixels
    localparam [9:0] SCORE_CHAR_HEIGHT = CHAR_HEIGHT * SCORE_SCALE; // 256 pixels
    localparam [9:0] LEFT_HALF_CENTER_X = 160;   // Center of left half (0-319)
    localparam [9:0] RIGHT_HALF_CENTER_X = 480;  // Center of right half (320-639)
    localparam [9:0] SCORE_CENTER_Y = 240;       // Vertical center
    
    // Calculate if we're in large score regions
    logic in_score_a_region, in_score_b_region;
    logic [9:0] score_a_x_offset, score_a_y_offset;
    logic [9:0] score_b_x_offset, score_b_y_offset;
    logic [2:0] score_font_col;  // Which column in 8-pixel font (0-7)
    logic [3:0] score_font_row;  // Which row in 16-pixel font (0-15)
    logic [7:0] score_a_char, score_b_char;  // ASCII character for each score
    logic [10:0] score_a_font_addr, score_b_font_addr;
    logic [7:0] score_a_font_data, score_b_font_data;
    logic score_a_pixel_on, score_b_pixel_on;
    logic in_pong_state;
    
    assign in_pong_state = (safe_display_state == 5'd21);
    
    // Player A (left half) score region
    localparam [9:0] SCORE_CHAR_HALF_WIDTH = SCORE_CHAR_WIDTH >> 1;   // 64 pixels
    localparam [9:0] SCORE_CHAR_HALF_HEIGHT = SCORE_CHAR_HEIGHT >> 1; // 128 pixels
    assign score_a_x_offset = (draw_x >= (LEFT_HALF_CENTER_X - SCORE_CHAR_HALF_WIDTH)) &&
                               (draw_x < (LEFT_HALF_CENTER_X + SCORE_CHAR_HALF_WIDTH)) ?
                               draw_x - (LEFT_HALF_CENTER_X - SCORE_CHAR_HALF_WIDTH) : 10'd0;
    assign score_a_y_offset = (draw_y >= (SCORE_CENTER_Y - SCORE_CHAR_HALF_HEIGHT)) &&
                               (draw_y < (SCORE_CENTER_Y + SCORE_CHAR_HALF_HEIGHT)) ?
                               draw_y - (SCORE_CENTER_Y - SCORE_CHAR_HALF_HEIGHT) : 10'd0;
    assign in_score_a_region = in_pong_state &&
                                (draw_x >= (LEFT_HALF_CENTER_X - SCORE_CHAR_HALF_WIDTH)) &&
                                (draw_x < (LEFT_HALF_CENTER_X + SCORE_CHAR_HALF_WIDTH)) &&
                                (draw_y >= (SCORE_CENTER_Y - SCORE_CHAR_HALF_HEIGHT)) &&
                                (draw_y < (SCORE_CENTER_Y + SCORE_CHAR_HALF_HEIGHT));
    
    // Player B (right half) score region
    assign score_b_x_offset = (draw_x >= (RIGHT_HALF_CENTER_X - SCORE_CHAR_HALF_WIDTH)) &&
                               (draw_x < (RIGHT_HALF_CENTER_X + SCORE_CHAR_HALF_WIDTH)) ?
                               draw_x - (RIGHT_HALF_CENTER_X - SCORE_CHAR_HALF_WIDTH) : 10'd0;
    assign score_b_y_offset = (draw_y >= (SCORE_CENTER_Y - SCORE_CHAR_HALF_HEIGHT)) &&
                               (draw_y < (SCORE_CENTER_Y + SCORE_CHAR_HALF_HEIGHT)) ?
                               draw_y - (SCORE_CENTER_Y - SCORE_CHAR_HALF_HEIGHT) : 10'd0;
    assign in_score_b_region = in_pong_state &&
                                (draw_x >= (RIGHT_HALF_CENTER_X - SCORE_CHAR_HALF_WIDTH)) &&
                                (draw_x < (RIGHT_HALF_CENTER_X + SCORE_CHAR_HALF_WIDTH)) &&
                                (draw_y >= (SCORE_CENTER_Y - SCORE_CHAR_HALF_HEIGHT)) &&
                                (draw_y < (SCORE_CENTER_Y + SCORE_CHAR_HALF_HEIGHT));
    
    // Scale down to font coordinates (right shift by SCORE_SCALE_LOG2 = 4)
    // score_font_col: 3 bits for 0-7, take bits [6:4] after 16x downscale
    // score_font_row: 4 bits for 0-15, take bits [7:4] after 16x downscale
    assign score_font_col = in_score_a_region ? score_a_x_offset[6:4] :
                            in_score_b_region ? score_b_x_offset[6:4] : 3'd0;
    assign score_font_row = in_score_a_region ? score_a_y_offset[7:4] :
                            in_score_b_region ? score_b_y_offset[7:4] : 4'd0;
    
    // Convert score (0-7) to ASCII character ('0' = 48), or W/L if someone won
    // 'W' = 87, 'L' = 76
    always_comb begin
        if (pong_player_a_wins) begin
            score_a_char = 8'd87;  // 'W'
        end else if (pong_player_b_wins) begin
            score_a_char = 8'd76;  // 'L'
        end else begin
            score_a_char = 8'd48 + {4'd0, pong_player_a_score};  // Score digit
        end
        
        if (pong_player_b_wins) begin
            score_b_char = 8'd87;  // 'W'
        end else if (pong_player_a_wins) begin
            score_b_char = 8'd76;  // 'L'
        end else begin
            score_b_char = 8'd48 + {4'd0, pong_player_b_score};  // Score digit
        end
    end
    
    // Font ROM addresses for scores
    assign score_a_font_addr = {score_a_char[6:0], score_font_row};
    assign score_b_font_addr = {score_b_char[6:0], score_font_row};
    
    // Second font ROM instance for score (or we can time-multiplex, but let's use a separate lookup)
    // Actually, we need to handle this carefully - font ROM is single port
    // For now, let's use the same font ROM but prioritize scores
    logic [10:0] score_font_addr_use;
    logic use_score_font;
    assign use_score_font = in_score_a_region || in_score_b_region;
    assign score_font_addr_use = in_score_a_region ? score_a_font_addr : score_b_font_addr;
    
    // Check if scaled pixel should be on (using font_data from ROM)
    assign score_a_pixel_on = in_score_a_region && (font_data[7 - score_font_col]);
    assign score_b_pixel_on = in_score_b_region && (font_data[7 - score_font_col]);
    
    // Check if we're in the text row at bottom
    logic in_text_row;
    assign in_text_row = (draw_y >= TEXT_ROW_START) && (draw_y < TEXT_ROW_START + CHAR_HEIGHT);
    
    // Output logic with separate bottom-right gesture region and large score display
    always_comb begin
        // In PONG state, black out everything (including text row) to hide hands
        if (vde && in_pong_state) begin
            // Black out entire screen (hides hands and text)
            pixel_red_out   = 3'b000;
            pixel_green_out = 3'b000;
            pixel_blue_out  = 3'b000;
            
            // Override with large score display (rendered on black background)
            if (in_score_a_region || in_score_b_region) begin
                if (score_a_pixel_on || score_b_pixel_on) begin
                    // Gray score/win letters
                    pixel_red_out   = 3'b011;
                    pixel_green_out = 3'b011;
                    pixel_blue_out  = 3'b011;
                end
                // Background stays black (hands and text hidden)
            end
        end else begin
            // Default passthrough (normal mode)
            pixel_red_out   = pixel_red_in;
            pixel_green_out = pixel_green_in;
            pixel_blue_out  = pixel_blue_in;

            // Regular text overlay at bottom (only when not in PONG state)
            if (vde && in_text_row) begin
                // Main state text region (left)
                if (in_text_region) begin
                    if (text_pixel_on && current_char != 8'd32) begin
                        pixel_red_out   = TEXT_RED;
                        pixel_green_out = TEXT_GREEN;
                        pixel_blue_out  = TEXT_BLUE;
                    end else if (black_background && current_char != 8'd32) begin
                        // Only draw black background for non-space characters
                        pixel_red_out   = 3'b000;
                        pixel_green_out = 3'b000;
                        pixel_blue_out  = 3'b000;
                    end
                end
                // Gesture word region (right): render whenever a gesture is active, independent of state overrides
                if ((gesture_code != 2'd0) && in_gesture_region) begin
                    if (gesture_text_on && gesture_char != 8'd32) begin
                        pixel_red_out   = TEXT_RED;
                        pixel_green_out = TEXT_GREEN;
                        pixel_blue_out  = TEXT_BLUE;
                    end else begin
                        // Force black background box even for spaces for consistency
                        pixel_red_out   = 3'b000;
                        pixel_green_out = 3'b000;
                        pixel_blue_out  = 3'b000;
                    end
                end
            end
        end
    end

endmodule

