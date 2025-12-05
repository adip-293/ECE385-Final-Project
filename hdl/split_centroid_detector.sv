/*
 * Split Centroid Detector Module
 *
 * Description:
 *   Computes two centroids (left and right halves) from a binary mask stream.
 *   Uses separate spatial moment accumulators and bounding boxes per half.
 *
 * Interface matches centroid_detector style, but outputs duplicated per half.
 */

module split_centroid_detector #(
    parameter FRAME_WIDTH   = 640,
    parameter FRAME_HEIGHT  = 480,
    parameter MIN_BLOB_SIZE = 300
)(
    input  logic        clk,
    input  logic        rst_n,
    input  logic        enable,

    // Pixel stream input
    input  logic [3:0]  mask_pixel,
    input  logic [9:0]  pixel_x,
    input  logic [9:0]  pixel_y,
    input  logic        pixel_valid,

    // Frame synchronization
    input  logic        frame_start,

    // Left centroid outputs
    output logic [9:0]  left_centroid_x,
    output logic [9:0]  left_centroid_y,
    output logic        left_centroid_valid,
    output logic [19:0] left_blob_area,
    output logic [9:0]  left_bbox_min_x,
    output logic [9:0]  left_bbox_min_y,
    output logic [9:0]  left_bbox_max_x,
    output logic [9:0]  left_bbox_max_y,

    // Right centroid outputs
    output logic [9:0]  right_centroid_x,
    output logic [9:0]  right_centroid_y,
    output logic        right_centroid_valid,
    output logic [19:0] right_blob_area,
    output logic [9:0]  right_bbox_min_x,
    output logic [9:0]  right_bbox_min_y,
    output logic [9:0]  right_bbox_max_x,
    output logic [9:0]  right_bbox_max_y
);

    localparam logic [9:0] HALF_WIDTH  = FRAME_WIDTH[9:0] >> 1; // 320
    localparam logic [9:0] FRAME_MAX_X = FRAME_WIDTH - 1;
    localparam logic [9:0] FRAME_MAX_Y = FRAME_HEIGHT - 1;

    // Accumulators for left half
    logic [19:0] sum1_L;      // M00
    logic [28:0] sumx_L;      // M10
    logic [28:0] sumy_L;      // M01

    // Accumulators for right half
    logic [19:0] sum1_R;
    logic [28:0] sumx_R;
    logic [28:0] sumy_R;

    // Next-state versions
    logic [19:0] sum1_L_n, sum1_R_n;
    logic [28:0] sumx_L_n, sumx_R_n;
    logic [28:0] sumy_L_n, sumy_R_n;

    // Bounding boxes accumulators
    logic [9:0] minx_L, miny_L, maxx_L, maxy_L;
    logic [9:0] minx_R, miny_R, maxx_R, maxy_R;
    logic [9:0] minx_L_n, miny_L_n, maxx_L_n, maxy_L_n;
    logic [9:0] minx_R_n, miny_R_n, maxx_R_n, maxy_R_n;

    // Edge detect for frame_start
    logic frame_start_d;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) frame_start_d <= 1'b0; else frame_start_d <= frame_start;
    end
    logic frame_complete;
    assign frame_complete = frame_start & ~frame_start_d;

    // Split classification
    logic is_left, is_right;
    always_comb begin
        is_left  = (pixel_x < HALF_WIDTH);
        is_right = ~is_left; // pixels at x>=HALF_WIDTH go right
    end

    // Accumulation
    always_comb begin
        // Defaults: hold
        sum1_L_n = sum1_L; sumx_L_n = sumx_L; sumy_L_n = sumy_L;
        sum1_R_n = sum1_R; sumx_R_n = sumx_R; sumy_R_n = sumy_R;
        // BBox defaults
        minx_L_n = minx_L; miny_L_n = miny_L; maxx_L_n = maxx_L; maxy_L_n = maxy_L;
        minx_R_n = minx_R; miny_R_n = miny_R; maxx_R_n = maxx_R; maxy_R_n = maxy_R;

        if (enable && pixel_valid && (|mask_pixel)) begin
            if (is_left) begin
                sum1_L_n = sum1_L + 20'd1;
                sumx_L_n = sumx_L + {19'd0, pixel_x};
                sumy_L_n = sumy_L + {19'd0, pixel_y};
                // bbox
                minx_L_n = (pixel_x < minx_L) ? pixel_x : minx_L;
                miny_L_n = (pixel_y < miny_L) ? pixel_y : miny_L;
                maxx_L_n = (pixel_x > maxx_L) ? pixel_x : maxx_L;
                maxy_L_n = (pixel_y > maxy_L) ? pixel_y : maxy_L;
            end else begin
                sum1_R_n = sum1_R + 20'd1;
                sumx_R_n = sumx_R + {19'd0, pixel_x};
                sumy_R_n = sumy_R + {19'd0, pixel_y};
                // bbox
                minx_R_n = (pixel_x < minx_R) ? pixel_x : minx_R;
                miny_R_n = (pixel_y < miny_R) ? pixel_y : miny_R;
                maxx_R_n = (pixel_x > maxx_R) ? pixel_x : maxx_R;
                maxy_R_n = (pixel_y > maxy_R) ? pixel_y : maxy_R;
            end
        end
    end

    // Registers
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum1_L <= 20'd0; sumx_L <= 29'd0; sumy_L <= 29'd0;
            sum1_R <= 20'd0; sumx_R <= 29'd0; sumy_R <= 29'd0;
            minx_L <= FRAME_MAX_X; miny_L <= FRAME_MAX_Y; maxx_L <= 10'd0; maxy_L <= 10'd0;
            minx_R <= FRAME_MAX_X; miny_R <= FRAME_MAX_Y; maxx_R <= 10'd0; maxy_R <= 10'd0;
        end else if (frame_complete && enable) begin
            // reset on frame boundary when enabled
            sum1_L <= 20'd0; sumx_L <= 29'd0; sumy_L <= 29'd0;
            sum1_R <= 20'd0; sumx_R <= 29'd0; sumy_R <= 29'd0;
            minx_L <= FRAME_MAX_X; miny_L <= FRAME_MAX_Y; maxx_L <= 10'd0; maxy_L <= 10'd0;
            minx_R <= FRAME_MAX_X; miny_R <= FRAME_MAX_Y; maxx_R <= 10'd0; maxy_R <= 10'd0;
        end else if (enable) begin
            sum1_L <= sum1_L_n; sumx_L <= sumx_L_n; sumy_L <= sumy_L_n;
            sum1_R <= sum1_R_n; sumx_R <= sumx_R_n; sumy_R <= sumy_R_n;
            minx_L <= minx_L_n; miny_L <= miny_L_n; maxx_L <= maxx_L_n; maxy_L <= maxy_L_n;
            minx_R <= minx_R_n; miny_R <= miny_R_n; maxx_R <= maxx_R_n; maxy_R <= maxy_R_n;
        end
    end

    // Compute and latch results at frame completion
    logic [9:0] cxL_calc, cyL_calc; logic vL_calc;
    logic [9:0] cxR_calc, cyR_calc; logic vR_calc;

    always_comb begin
        if (sum1_L >= MIN_BLOB_SIZE[19:0]) begin
            cxL_calc = sumx_L / sum1_L; cyL_calc = sumy_L / sum1_L; vL_calc = 1'b1;
        end else begin
            cxL_calc = 10'd0; cyL_calc = 10'd0; vL_calc = 1'b0;
        end
        if (sum1_R >= MIN_BLOB_SIZE[19:0]) begin
            cxR_calc = sumx_R / sum1_R; cyR_calc = sumy_R / sum1_R; vR_calc = 1'b1;
        end else begin
            cxR_calc = 10'd0; cyR_calc = 10'd0; vR_calc = 1'b0;
        end
    end

    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            left_centroid_x <= 10'd0; left_centroid_y <= 10'd0; left_centroid_valid <= 1'b0;
            right_centroid_x <= 10'd0; right_centroid_y <= 10'd0; right_centroid_valid <= 1'b0;
            left_blob_area <= 20'd0; right_blob_area <= 20'd0;
            left_bbox_min_x <= 10'd0; left_bbox_min_y <= 10'd0; left_bbox_max_x <= 10'd0; left_bbox_max_y <= 10'd0;
            right_bbox_min_x <= 10'd0; right_bbox_min_y <= 10'd0; right_bbox_max_x <= 10'd0; right_bbox_max_y <= 10'd0;
        end else if (frame_complete && enable) begin
            left_centroid_x <= cxL_calc; left_centroid_y <= cyL_calc; left_centroid_valid <= vL_calc;
            right_centroid_x <= cxR_calc; right_centroid_y <= cyR_calc; right_centroid_valid <= vR_calc;
            left_blob_area <= sum1_L; right_blob_area <= sum1_R;
            if (vL_calc) begin
                left_bbox_min_x <= minx_L; left_bbox_min_y <= miny_L; left_bbox_max_x <= maxx_L; left_bbox_max_y <= maxy_L;
            end else begin
                left_bbox_min_x <= 10'd0; left_bbox_min_y <= 10'd0; left_bbox_max_x <= 10'd0; left_bbox_max_y <= 10'd0;
            end
            if (vR_calc) begin
                right_bbox_min_x <= minx_R; right_bbox_min_y <= miny_R; right_bbox_max_x <= maxx_R; right_bbox_max_y <= maxy_R;
            end else begin
                right_bbox_min_x <= 10'd0; right_bbox_min_y <= 10'd0; right_bbox_max_x <= 10'd0; right_bbox_max_y <= 10'd0;
            end
        end
    end

endmodule
