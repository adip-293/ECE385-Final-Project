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
    
    // Original camera data (for color mode)
    input  logic [11:0] cam_pix_data,
    input  logic [18:0] cam_pix_addr,
    input  logic        cam_pix_write,
    input  logic        cam_vsync,
    input  logic [9:0]  cam_line_y,
    input  logic        cam_line_ready,

    // Compare mode: show processed on left half, raw(grayscale) on right half
    input  logic        compare_mode,

    // VGA output
    output logic [8:0]  vga_pix_data,
    output logic [1:0]  frame_chunk_counter
);

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
            // Pack 12-bit RGB444 to 9-bit RGB333 (take top 3 bits of each channel)
            cam_pix_data_9bit = {cam_pix_data[11:9], cam_pix_data[7:5], cam_pix_data[3:1]};
            bram_write_addr = cam_pix_addr;
            bram_write_enable = cam_pix_write;
        end
    end

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
        .wea   (bram_write_enable && (bram_read_stage || !temporal_filter_enable || !processing_enable)),

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
