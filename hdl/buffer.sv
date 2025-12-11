`timescale 1ns / 1ps

/*
 * Buffer Module
 * 
 * Description:
 *   Frame buffer with dual-port BRAM for video storage.
 *   Port A: writes from camera/preprocessor (pclk), Port B: reads for VGA (clk_25m).
 * 
 * Purpose:
 *   Bridge between camera pixel clock and VGA pixel clock domains.
 *   Supports temporal filtering with read-modify-write for 3-frame history.
 * 
 * Notes:
 *   - Temporal mode: 9-bit stores 3 frames (3 bits each): [8:6], [5:3], [2:0]
 *   - Normal mode: stores single 3-bit value in [8:6], [5:0]=0
 *   - Tear guard: avoids reading line currently being written by camera
 *   - Border clamping: blanks first/last 2 rows/columns to prevent artifacts
 */

module buffer (
    // System clocks
    input  logic        clk_25m,
    input  logic        rst_n,
    input  logic        pclk,

    // VGA timing
    input  logic [9:0]  draw_x,
    input  logic [9:0]  draw_y,
    input  logic        vde,

    // Preprocessed pixel write interface
    input  logic [2:0]  processed_pixel,
    input  logic [18:0] processed_addr,
    input  logic        processed_valid,
    
    // Processing control
    input  logic        processing_enable,
    input  logic        temporal_filter_enable,
    input  logic        dither_enable,         // Enable dithered color compression
    input  logic        dither_type,           // Dither type: 0=simple (x+y)%3, 1=Bayer matrix
    input  logic        force_color,           // Force color override - disables dithering when active
    
    // Original camera data (for color mode)
    input  logic [11:0] cam_pix_data,
    input  logic [18:0] cam_pix_addr,
    input  logic        cam_pix_write,
    input  logic        cam_vsync,
    input  logic [9:0]  cam_line_y,
    input  logic        cam_line_ready,

    // Compare mode: show processed on left half, raw(grayscale) on right half
    input  logic        compare_mode,
    
    // Transition trigger for ripple effect (override toggles only)
    input  logic        transition_trigger,     // Pulse on override toggle (from clk_25m domain)

    // VGA output
    output logic [8:0]  vga_pix_data,
    output logic [1:0]  frame_chunk_counter
);

    // -----------------------------
    // Ripple Transition Controller (in pclk domain)
    // -----------------------------
    // All transition logic handled here - control_unit only provides trigger pulse
    logic transition_trigger_sync1, transition_trigger_sync2, transition_trigger_pclk;
    logic transition_trigger_pulse;  // Edge detected trigger
    
    // Synchronize transition_trigger pulse from clk_25m to pclk domain
    always_ff @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            transition_trigger_sync1 <= 1'b0;
            transition_trigger_sync2 <= 1'b0;
            transition_trigger_pclk <= 1'b0;
        end else begin
            transition_trigger_sync1 <= transition_trigger;
            transition_trigger_sync2 <= transition_trigger_sync1;
            transition_trigger_pclk <= transition_trigger_sync2;
        end
    end
    
    // Edge detect the synchronized trigger
    logic transition_trigger_prev;
    always_ff @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            transition_trigger_prev <= 1'b0;
        end else begin
            transition_trigger_prev <= transition_trigger_pclk;
        end
    end
    assign transition_trigger_pulse = transition_trigger_pclk & ~transition_trigger_prev;
    
    // Ripple transition state machine (in pclk domain, uses camera frame timing)
    localparam [9:0] RIPPLE_MAX_RADIUS = 10'd400;  // Maximum radius to cover 640x480 screen
    localparam [9:0] RIPPLE_INCREMENT = 10'd50;    // Pixels to expand per frame (fast: ~8 frames to complete)
    
    logic transition_active_pclk;
    logic [9:0] ripple_radius_pclk;
    
    always_ff @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            transition_active_pclk <= 1'b0;
            ripple_radius_pclk <= 10'd0;
        end else begin
            if (transition_trigger_pulse) begin
                // Start new transition on trigger pulse
                transition_active_pclk <= 1'b1;
                ripple_radius_pclk <= 10'd0;
            end else if (transition_active_pclk && frame_start_pclk) begin
                // Expand ripple on each camera frame start
                if (ripple_radius_pclk < RIPPLE_MAX_RADIUS) begin
                    ripple_radius_pclk <= ripple_radius_pclk + RIPPLE_INCREMENT;
                end else begin
                    // Transition complete
                    transition_active_pclk <= 1'b0;
                    ripple_radius_pclk <= 10'd0;
                end
            end
        end
    end

    // -----------------------------
    // Frame Counter (modulo 3) - tracks which 3-bit chunk is current frame
    // -----------------------------
    logic [1:0] frame_counter;  // 0, 1, or 2 - which chunk we're writing to
    logic vsync_sync_pclk, vsync_prev_pclk;
    logic frame_start_pclk;
    
    // Synchronize vsync to pclk domain
    always_ff @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_sync_pclk <= 1'b0;
            vsync_prev_pclk <= 1'b0;
        end else begin
            vsync_sync_pclk <= cam_vsync;
            vsync_prev_pclk <= vsync_sync_pclk;
        end
    end
    
    // Detect frame start (falling edge of vsync)
    assign frame_start_pclk = ~vsync_sync_pclk & vsync_prev_pclk;
    
    // Frame counter increments modulo 3 on each frame start
    always_ff @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            frame_counter <= 2'b0;
        end else if (frame_start_pclk) begin
            if (frame_counter == 2'b10)
                frame_counter <= 2'b0;
            else
                frame_counter <= frame_counter + 1'b1;
        end
    end
    
    // Synchronize frame_counter to clk_25m and latch per VGA frame
    // We hold a constant chunk index for the entire VGA scan to prevent
    // mid-scan changes that cause a visible "scrubbing line".
    logic [1:0] frame_counter_sync_25m, frame_counter_sync_25m_q;
    logic [1:0] frame_chunk_counter_latched;
    always_ff @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            frame_counter_sync_25m_q <= 2'b0;
            frame_chunk_counter_latched <= 2'b0;
            frame_chunk_counter <= 2'b0;
        end else begin
            // Simple multi-bit sampling; updated once per VGA frame below
            frame_counter_sync_25m_q <= frame_counter_sync_25m;

            // Latch chunk index at the start of each VGA frame (top-left pixel)
            if ((draw_x == 10'd0) && (draw_y == 10'd0)) begin
                // Use the last COMPLETED camera frame to avoid showing the
                // chunk currently being written by the camera (prevents
                // mid-frame scrubbing).
                case (frame_counter_sync_25m_q)
                    2'b00: frame_chunk_counter_latched <= 2'b10; // previous of 0 is 2
                    2'b01: frame_chunk_counter_latched <= 2'b00; // previous of 1 is 0
                    2'b10: frame_chunk_counter_latched <= 2'b01; // previous of 2 is 1
                    default: frame_chunk_counter_latched <= 2'b10;
                endcase
            end

            frame_chunk_counter <= frame_chunk_counter_latched;
        end
    end
    assign frame_counter_sync_25m = frame_counter;  // CDC: updated slowly; latched per VGA frame
    
    // -----------------------------
    // Determine what to write to BRAM
    // In color modes: write original camera data
    // In processing modes: write preprocessed grayscale (3-bit only)
    // In temporal filter mode: read-modify-write to pack into correct chunk
    // -----------------------------
    logic [8:0]  cam_pix_data_9bit;
    logic [8:0]  bram_read_data;  // Data read from BRAM for read-modify-write
    logic [18:0] bram_write_addr, bram_read_addr;
    logic [2:0]  write_pixel_pipe;  // Pipeline pixel value for write after read
    logic [18:0] write_addr_pipe;   // Pipeline address for write after read
    logic [1:0]  write_counter_pipe; // Pipeline frame counter for write after read
    logic        bram_write_enable;
    logic        bram_read_stage;  // Stage 1: read, Stage 2: write

    // Locals used in compare mode
    logic [9:0] cam_x, proc_x;
    logic [2:0] cam_gray3;
    
    // Dithering logic signals
    logic [9:0] pix_x, pix_y;
    logic [1:0] dither_pattern;
    logic [1:0] dither_r, dither_g, dither_b;
    logic [1:0] dither_r_simple, dither_g_simple, dither_b_simple;
    logic [1:0] dither_r_bayer, dither_g_bayer, dither_b_bayer;
    logic [1:0] bayer_x, bayer_y;
    logic [3:0] bayer_index;
    logic [3:0] bayer_value, bayer_g, bayer_b;
    logic [4:0] r_with_dither, g_with_dither, b_with_dither;
    logic [3:0] r_clamped, g_clamped, b_clamped;
    logic [2:0] r_out, g_out, b_out;
    
    // Pipelined read-modify-write for temporal filter
    always_ff @(posedge pclk or negedge rst_n) begin
        if (!rst_n) begin
            write_pixel_pipe <= 3'b0;
            write_addr_pipe <= 19'b0;
            write_counter_pipe <= 2'b0;
            bram_read_stage <= 1'b0;
        end else begin
            if (processing_enable && temporal_filter_enable && processed_valid) begin
                // Stage 1: Initiate read
                bram_read_addr <= processed_addr;
                write_pixel_pipe <= processed_pixel;
                write_addr_pipe <= processed_addr;
                write_counter_pipe <= frame_counter;
                bram_read_stage <= 1'b1;  // Next cycle will be write stage
            end else if (bram_read_stage) begin
                // Stage 2: Use read data to modify and write
                bram_read_stage <= 1'b0;
            end
        end
    end
    
    // Read data is available after 1 cycle latency
    assign bram_read_data = bram_dout_a;
    
    always_comb begin
        // Compare mode: processed left, raw grayscale right
        if (compare_mode) begin
            // Determine X positions from linear addresses
            cam_x  = cam_pix_addr % 19'd640;
            proc_x = processed_addr % 19'd640;
            // Priority: processed-left when valid; else raw-right when valid; else no write
            if (processed_valid && (proc_x < 10'd320)) begin
                cam_pix_data_9bit  = {processed_pixel, 6'b000000};
                bram_write_addr    = processed_addr;
                bram_write_enable  = 1'b1;
            end else if (cam_pix_write && (cam_x >= 10'd320)) begin
                // Right-half grayscale: replicate camera green channel top 3 bits
                cam_gray3          = cam_pix_data[7:5];
                cam_pix_data_9bit  = {cam_gray3, cam_gray3, cam_gray3};
                bram_write_addr    = cam_pix_addr;
                bram_write_enable  = 1'b1;
            end else begin
                cam_pix_data_9bit  = 9'b0;
                bram_write_addr    = 19'b0;
                bram_write_enable  = 1'b0;
            end
        end else if (processing_enable && temporal_filter_enable && bram_read_stage) begin
            // Temporal filter mode: pack 3-bit value into appropriate chunk based on frame_counter
            // Use read data from BRAM (1 cycle latency)
            case (write_counter_pipe)
                2'b00: cam_pix_data_9bit = {write_pixel_pipe, bram_read_data[5:0]};  // Write to [8:6]
                2'b01: cam_pix_data_9bit = {bram_read_data[8:6], write_pixel_pipe, bram_read_data[2:0]};  // Write to [5:3]
                2'b10: cam_pix_data_9bit = {bram_read_data[8:3], write_pixel_pipe};  // Write to [2:0]
                default: cam_pix_data_9bit = {write_pixel_pipe, bram_read_data[5:0]};
            endcase
            bram_write_addr = write_addr_pipe;
            bram_write_enable = 1'b1;
        end else if (processing_enable) begin
            // Normal processing mode: store only 3-bit value in [8:6], leave [5:0] as 0
            cam_pix_data_9bit = {processed_pixel, 6'b000000};
            bram_write_addr = processed_addr;
            bram_write_enable = processed_valid;
        end else begin
            // Color modes: use original camera data
            // Only apply dithering if enabled AND not forcing raw color (override)
            if (dither_enable && !force_color) begin
                // Dithered color compression: RGB444 -> RGB333 with rounding + dithering
                // Extract pixel coordinates for dither pattern
                pix_x = cam_pix_addr % 19'd640;
                pix_y = cam_pix_addr / 19'd640;
                
                // Select dithering method based on dither_type
                if (dither_type == 1'b0) begin
                    // Simple position-based dither pattern: (x+y) % 3
                    // Maps to: 0 -> -1, 1 -> 0, 2 -> +1
                    dither_pattern = (pix_x + pix_y) % 3;
                    
                    // Convert pattern to dither value (-1, 0, +1)
                    dither_r_simple = (dither_pattern == 2'd0) ? 2'b11 : (dither_pattern == 2'd2) ? 2'b01 : 2'b00;  // -1, 0, +1
                    dither_g_simple = dither_r_simple;  // Apply same dither to all channels
                    dither_b_simple = dither_r_simple;
                    
                    dither_r = dither_r_simple;
                    dither_g = dither_g_simple;
                    dither_b = dither_b_simple;
                end else begin
                    // Bayer matrix 4x4 ordered dithering
                    // Bayer matrix: [0  8  2 10]
                    //              [12 4 14  6]
                    //              [3 11  1  9]
                    //              [15 7 13  5]
                    // Values 0-15, we'll use to create -1, 0, or +1 dither
                    
                    // Get position in 4x4 Bayer tile
                    bayer_x = pix_x[1:0];
                    bayer_y = pix_y[1:0];
                    bayer_index = {bayer_y, bayer_x};  // 4-bit index (0-15)
                    
                    // 4x4 Bayer matrix lookup (values 0-15)
                    case (bayer_index)
                        4'd0:  bayer_value = 4'd0;
                        4'd1:  bayer_value = 4'd8;
                        4'd2:  bayer_value = 4'd2;
                        4'd3:  bayer_value = 4'd10;
                        4'd4:  bayer_value = 4'd12;
                        4'd5:  bayer_value = 4'd4;
                        4'd6:  bayer_value = 4'd14;
                        4'd7:  bayer_value = 4'd6;
                        4'd8:  bayer_value = 4'd3;
                        4'd9:  bayer_value = 4'd11;
                        4'd10: bayer_value = 4'd1;
                        4'd11: bayer_value = 4'd9;
                        4'd12: bayer_value = 4'd15;
                        4'd13: bayer_value = 4'd7;
                        4'd14: bayer_value = 4'd13;
                        4'd15: bayer_value = 4'd5;
                        default: bayer_value = 4'd0;
                    endcase
                    
                    // Convert Bayer value (0-15) to dither (-1, 0, +1)
                    // Thresholds: 0-4 = -1, 5-10 = 0, 11-15 = +1
                    // This creates balanced dithering
                    dither_r_bayer = (bayer_value < 5) ? 2'b11 : (bayer_value < 11) ? 2'b00 : 2'b01;  // -1, 0, +1
                    // Offset Bayer for different channels to reduce correlation
                    bayer_g = (bayer_value + 4'd5) % 16;   // Offset by 5
                    bayer_b = (bayer_value + 4'd10) % 16;  // Offset by 10
                    dither_g_bayer = (bayer_g < 5) ? 2'b11 : (bayer_g < 11) ? 2'b00 : 2'b01;
                    dither_b_bayer = (bayer_b < 5) ? 2'b11 : (bayer_b < 11) ? 2'b00 : 2'b01;
                    
                    dither_r = dither_r_bayer;
                    dither_g = dither_g_bayer;
                    dither_b = dither_b_bayer;
                end
                
                // Add dither to the 4-bit value (affects LSB), then take [3:1] like normal conversion
                // This preserves bit mapping: same bit positions as raw, just with dither noise
                // Normal conversion: takes [11:9], [7:5], [3:1] = drops LSB (bits 8, 4, 0)
                // Dithered: add dither to full 4-bit value, clamp to prevent wrapping, then take [3:1]
                
                // Add dither to the full 4-bit value (sign-extend dither for addition)
                r_with_dither = {1'b0, cam_pix_data[11:8]} + {{3{dither_r[1]}}, dither_r};
                g_with_dither = {1'b0, cam_pix_data[7:4]} + {{3{dither_g[1]}}, dither_g};
                b_with_dither = {1'b0, cam_pix_data[3:0]} + {{3{dither_b[1]}}, dither_b};
                
                // Clamp to valid 4-bit range (0-15) - prevent wrapping at boundaries
                // Prevent underflow: if value is 0 and dither is -1, clamp to 0 (don't wrap to 15)
                // Prevent overflow: if value is 15 and dither is +1, clamp to 15
                // dither_r[1]==1 means -1 (2'b11), dither_r[1]==0 means 0 (2'b00) or +1 (2'b01)
                r_clamped = ((cam_pix_data[11:8] == 4'd0) && (dither_r == 2'b11)) ? 4'd0 :  // Underflow protection
                           ((cam_pix_data[11:8] == 4'd15) && (dither_r == 2'b01)) ? 4'd15 : // Overflow protection
                           (r_with_dither > 5'd15) ? 4'd15 : r_with_dither[3:0];
                g_clamped = ((cam_pix_data[7:4] == 4'd0) && (dither_g == 2'b11)) ? 4'd0 :
                           ((cam_pix_data[7:4] == 4'd15) && (dither_g == 2'b01)) ? 4'd15 :
                           (g_with_dither > 5'd15) ? 4'd15 : g_with_dither[3:0];
                b_clamped = ((cam_pix_data[3:0] == 4'd0) && (dither_b == 2'b11)) ? 4'd0 :
                           ((cam_pix_data[3:0] == 4'd15) && (dither_b == 2'b01)) ? 4'd15 :
                           (b_with_dither > 5'd15) ? 4'd15 : b_with_dither[3:0];
                
                // Take [3:1] bits (same as normal conversion) - preserves bit mapping
                r_out = r_clamped[3:1];
                g_out = g_clamped[3:1];
                b_out = b_clamped[3:1];
                
                cam_pix_data_9bit = {r_out, g_out, b_out};
            end else begin
                // Simple truncation: Pack 12-bit RGB444 to 9-bit RGB333 (take top 3 bits of each channel)
                cam_pix_data_9bit = {cam_pix_data[11:9], cam_pix_data[7:5], cam_pix_data[3:1]};
            end
            bram_write_addr = cam_pix_addr;
            bram_write_enable = cam_pix_write;
        end
    end

    // -----------------------------
    // Ripple Transition: Write Masking (Perfect Circle)
    // -----------------------------
    // Extract pixel coordinates from write address
    logic [9:0] write_x, write_y;
    logic [19:0] pixel_distance_squared;
    logic [19:0] radius_squared;
    logic write_inside_ripple;
    
    always_comb begin
        // Extract coordinates from write address
        write_x = bram_write_addr % 19'd640;
        write_y = bram_write_addr / 19'd640;
        
        // Calculate distance squared from screen center (320, 240)
        pixel_distance_squared = distance_squared(write_x, write_y, 10'd320, 10'd240);
        
        // Calculate radius squared for comparison
        radius_squared = ripple_radius_pclk * ripple_radius_pclk;
        
        // Generate write mask: allow writes inside ripple radius (perfect circle)
        if (transition_active_pclk) begin
            // Only allow writes inside the expanding circle
            // Compare squared distances to avoid sqrt
            write_inside_ripple = (pixel_distance_squared < radius_squared);
        end else begin
            // No transition: allow all writes
            write_inside_ripple = 1'b1;
        end
    end
    
    // Apply ripple mask to write enable
    logic bram_write_enable_masked;
    assign bram_write_enable_masked = bram_write_enable && write_inside_ripple;

    // -----------------------------
    // Helpers
    // -----------------------------
    localparam int H_ACTIVE = 640;
    localparam int V_ACTIVE = 480;
    localparam int H_TOTAL  = 800;
    localparam int V_TOTAL  = 525;

    // Address mapping: addr = y*640 + x
    function automatic [18:0] xy_to_addr(input logic [9:0] x, input logic [9:0] y);
        xy_to_addr = ({9'b0, y} << 9) + ({9'b0, y} << 7) + x;
    endfunction
    
    // -----------------------------
    // Ripple Transition: Perfect Circle Distance Calculation
    // -----------------------------
    // Perfect Euclidean distance: sqrt(dx² + dy²)
    // For comparison, we use distance squared to avoid sqrt
    // Center: (320, 240) - screen center
    function automatic [19:0] distance_squared(
        input logic [9:0] x,
        input logic [9:0] y,
        input logic [9:0] center_x,
        input logic [9:0] center_y
    );
        logic [9:0] dx, dy;
        logic [19:0] dx_squared, dy_squared;
        
        // Calculate absolute differences from center
        dx = (x >= center_x) ? (x - center_x) : (center_x - x);
        dy = (y >= center_y) ? (y - center_y) : (center_y - y);
        
        // Calculate squares (10-bit input → 20-bit output max: 640² = 409600)
        dx_squared = dx * dx;
        dy_squared = dy * dy;
        
        // Return distance squared
        distance_squared = dx_squared + dy_squared;
    endfunction

    // -----------------------------
    // BRAM Read Addressing (latency-aware)
    // -----------------------------
    // Use a registered address aligned to the current pixel (not next)
    logic [18:0] prefetch_addr;
    logic [18:0] prefetch_addr_q;
    // Tear guard: avoid reading the line currently being written by camera.
    // Synchronize cam_line_ready pulse into clk_25m domain and track a safe line number.
    logic cam_line_ready_sync1, cam_line_ready_sync2;
    logic [9:0] safe_line_y; // last completed line as seen in clk_25m domain

    // Simple 2-flop sync for single-bit pulse
    always_ff @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            cam_line_ready_sync1 <= 1'b0;
            cam_line_ready_sync2 <= 1'b0;
        end else begin
            cam_line_ready_sync1 <= cam_line_ready;
            cam_line_ready_sync2 <= cam_line_ready_sync1;
        end
    end

    // Latch safe_line_y when a line completes; cross cam_line_y through a small register bank
    // Note: cam_line_y is multi-bit; sampling it only when line_ready pulse is synchronized
    // reduces CDC hazards because cam_line_y is stable near the falling edge of href.
    logic [9:0] cam_line_y_sampled;
    always_ff @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            safe_line_y <= 10'd0;
            cam_line_y_sampled <= 10'd0;
        end else if (cam_line_ready_sync2) begin
            cam_line_y_sampled <= cam_line_y;
            safe_line_y <= cam_line_y_sampled; // one-cycle delayed sample
        end
    end
    
    always_comb begin
        // Current intended read coordinates
        logic [9:0] read_x, read_y;
        read_x = draw_x;
        read_y = draw_y;

        if (draw_y < V_ACTIVE) begin
            // Guard the currently written line only for the first few pixels to avoid left-edge wrap
            if ((draw_y == safe_line_y) && (draw_y > 10'd0) && (draw_x < 10'd2)) begin
                read_y = draw_y - 10'd1;
            end

            // Clamp leftmost pixel to current coordinates (no lookahead)
            prefetch_addr = xy_to_addr(read_x, read_y);
        end else begin
            // Vertical blank
            prefetch_addr = xy_to_addr(10'd0, 10'd0);
        end
    end

    // Register the address to align with BRAM 1-cycle latency
    always_ff @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            prefetch_addr_q <= 19'd0;
        end else begin
            prefetch_addr_q <= prefetch_addr;
        end
    end

    // -----------------------------
    // Dual-port BRAM
    // Port A: writes on pclk (camera/preprocessor)
    //   - In temporal mode: read-modify-write (reads existing, writes modified)
    // Port B: reads on clk_25m (VGA)
    // -----------------------------
    logic [8:0] bram_dout_q;
    logic [8:0] bram_dout_a;  // Port A read data (for read-modify-write)

    // Address mux: read address during read stage, write address during write stage
    logic [18:0] bram_addr_a;
    assign bram_addr_a = (processing_enable && temporal_filter_enable && processed_valid && !bram_read_stage) ? bram_read_addr : bram_write_addr;
    
    blk_mem_gen_0 pixel_memory (
        // Port A (write from camera/preprocessor, with read for read-modify-write)
        .addra (bram_addr_a),
        .clka  (pclk),
        .dina  (cam_pix_data_9bit),
        .douta (bram_dout_a),  // Read existing data for read-modify-write
        .ena   (1'b1),
        .wea   (bram_write_enable_masked && (bram_read_stage || !temporal_filter_enable || !processing_enable)),

        // Port B (read for VGA, 1-cycle latency): use registered address
        .addrb (prefetch_addr_q),
        .clkb  (clk_25m),
        .dinb  (9'b0),
        .doutb (bram_dout_q),
        .enb   (1'b1),
        .web   (1'b0)
    );

    // -----------------------------
    // VSYNC start-of-frame clamp: blank first 2 pixels of each line
    // (prevents any residual edge artifacts at frame boundaries)
    // -----------------------------
    logic cam_vsync_25m_d1, cam_vsync_25m_d2;
    always_ff @(posedge clk_25m or negedge rst_n) begin
        if (!rst_n) begin
            cam_vsync_25m_d1 <= 1'b0;
            cam_vsync_25m_d2 <= 1'b0;
        end else begin
            cam_vsync_25m_d1 <= cam_vsync;
            cam_vsync_25m_d2 <= cam_vsync_25m_d1;
        end
    end

    wire frame_start_25m = (~cam_vsync_25m_d1) & cam_vsync_25m_d2; // falling edge

    // Final pixel output with comprehensive edge clamping:
    // 1. Blank first 2 pixels when the current line is being written (tearing protection)
    // 2. Blank top 2 and bottom 2 rows to prevent edge artifacts
    // 3. Blank leftmost 2 and rightmost 2 columns to prevent wrapping artifacts
    wire left_edge_clamp = vde && (draw_x < 10'd2) && (draw_y == safe_line_y);
    wire border_clamp = vde && ((draw_x < 10'd2) || (draw_x >= 10'd638) ||
                                (draw_y < 10'd2) || (draw_y >= 10'd478));
    assign vga_pix_data = (left_edge_clamp || border_clamp) ? 9'd0 : (vde ? bram_dout_q : 9'd0);

endmodule

