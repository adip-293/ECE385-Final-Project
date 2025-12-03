/*
 * Temporal Filter Module (Median-of-Three)
 * 
 * Description:
 *   Computes the median of the current and two previous 3-bit frame values
 *   stored in a single 9-bit word. Median-of-three is robust to outliers and
 *   introduces minimal lag compared to averaging.
 * 
 * Algorithm:
 *   - Extract current, prev1, prev2 based on frame_chunk_counter
 *   - Output median(current, prev1, prev2)
 * 
 * Notes:
 *   - 9-bit memory word stores 3 frames (3 bits each): [8:6], [5:3], [2:0]
 *   - frame_chunk_counter indicates which chunk is the current frame
 */

module temporal_filter (
    // Input channels (3 bits each from 9-bit word)
    input  logic [2:0]  pixel_red_in,
    input  logic [2:0]  pixel_green_in,
    input  logic [2:0]  pixel_blue_in,
    input  logic [1:0]  frame_chunk_counter,
    
    // Output
    //output logic [2:0]  temporal_avg
    output logic [2:0] red_out,
    output logic [2:0] blue_out,
    output logic [2:0] green_out
);

    // Reconstruct 9-bit value from separate RGB channels
    logic [8:0] full_9bit_pixel;
    assign full_9bit_pixel = {pixel_red_in, pixel_green_in, pixel_blue_in};
    
    // Extract the three frames based on frame_chunk_counter
    // frame_chunk_counter tells us which chunk is the current frame
    logic [2:0] current_frame, prev_frame1, prev_frame2;
    
    always_comb begin
        case (frame_chunk_counter)
            2'b00: begin  // Current frame is in [8:6]
                current_frame = full_9bit_pixel[8:6];
                prev_frame1 = full_9bit_pixel[5:3];
                prev_frame2 = full_9bit_pixel[2:0];
            end
            2'b01: begin  // Current frame is in [5:3]
                current_frame = full_9bit_pixel[5:3];
                prev_frame1 = full_9bit_pixel[8:6];
                prev_frame2 = full_9bit_pixel[2:0];
            end
            2'b10: begin  // Current frame is in [2:0]
                current_frame = full_9bit_pixel[2:0];
                prev_frame1 = full_9bit_pixel[5:3];
                prev_frame2 = full_9bit_pixel[8:6];
            end
            default: begin
                current_frame = full_9bit_pixel[8:6];
                prev_frame1 = full_9bit_pixel[5:3];
                prev_frame2 = full_9bit_pixel[2:0];
            end
        endcase
    end
    
    // Median-of-three computation: median = a + b + c - min(a,b,c) - max(a,b,c)
    logic [2:0] a, b, c;
    // logic [2:0] min_ab, max_ab;
    // logic [2:0] min_abc, max_abc;
    // logic [4:0] sum_abc; // up to 21
    //logic [4:0] median_ext; // extended-width median before truncation
    logic [2:0] diff1, diff2;
    logic [3:0] motion;
    always_comb begin
        a = current_frame;
        b = prev_frame1;
        c = prev_frame2;

        // Pairwise min/max for a and b
        // min_ab = (a < b) ? a : b;
        // max_ab = (a > b) ? a : b;
        // // Min of (min_ab, c), Max of (max_ab, c)
        // min_abc = (min_ab < c) ? min_ab : c;
        // max_abc = (max_ab > c) ? max_ab : c;
        
        // // Sum and compute median
        // sum_abc   = {2'b00, a} + {2'b00, b} + {2'b00, c};
        // median_ext = sum_abc - {2'b00, min_abc} - {2'b00, max_abc};
        //temporal_avg = median_ext[2:0];
        if(a>=b)begin
            diff1 = a-b;
        end else begin
            diff1 = b-a;
        end

        if(b>=c) begin
            diff2 = b-c;
        end else begin
            diff2 = c-b;
        end
        
        // Apply deadzone to filter out camera noise (ignore diff of 0 or 1)
        if (diff1 <= 3'd2) diff1 = 3'd0;
        if (diff2 <= 3'd2) diff2 = 3'd0;
        
        // Piecewise linear for better control
        motion = diff1 + diff2;
        if (motion == 4'd0) begin
            // No motion: Deep blue/black
            red_out   = 3'b000;
            green_out = 3'b000;
            blue_out  = 3'b100;
        end else if (motion < 4'd3) begin
            // Low motion: Blue
            red_out   = 3'b000;
            green_out = motion[1:0];
            blue_out  = 3'b111;
        end else if (motion < 4'd6) begin
            // Medium motion: cyan → green → yellow
            red_out   = motion[2:0] - 3'd3;
            green_out = 3'b111;
            blue_out  = 3'd6 - motion[2:0];
        end else begin
            // High motion: yellow → orange → red
            red_out   = 3'b111;
            green_out = (motion > 4'd12) ? 3'b001 : (4'd12 - motion[2:0]);
            blue_out  = 3'b000;
        end
    end



endmodule

