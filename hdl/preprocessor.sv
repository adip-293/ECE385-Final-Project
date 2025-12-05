`timescale 1ns / 1ps

/*
 * Preprocessor Module
 * 
 * Description:
 *   Complete image preprocessing pipeline before frame buffer.
 *   Camera RGB -> Grayscale -> Spatial Filters -> Morphology -> Centroid -> Gesture.
 * 
 * Purpose:
 *   All real-time image processing for camera feed.
 *   Outputs processed pixels and tracking metadata (centroid, gestures).
 * 
 * Notes:
 *   - Pipeline stages: grayscale, line buffer, median, convolution, threshold
 *   - Skin pipeline: skin_threshold -> spatial_filter -> erosion -> dilation
 *   - Color pipeline: color_threshold -> spatial_filter -> centroid
 *   - Border handling: excludes first/last 2 rows/columns from processing
 *   - Supports multiple processing modes via enable signals
 */

module preprocessor (
    // System
    input  logic        clk,
    input  logic        rst_n,
    
    // Camera inputs
    input  logic [11:0] cam_pix_data,
    input  logic [18:0] cam_pix_addr,
    input  logic        cam_pix_write,
    input  logic        cam_vsync,
    
    // Processing control
    input  logic        grayscale_enable,
    input  logic        threshold_enable,
    input  logic [3:0]  threshold_value,
    input  logic        median_enable,
    input  logic        convolution_enable,
    input  logic [1:0]  kernel_select,
    input  logic        skin_threshold_enable,
    input  logic        spatial_filter_enable,
    input  logic        erosion_enable,
    input  logic        dilation_enable,
    input  logic        centroid_enable,
    input  logic        blob_filter_enable,
    input  logic        gesture_enable,
    input  logic        split_centroid_enable,
    input  logic [3:0]  skin_y_min,
    input  logic [3:0]  skin_y_max,
    input  logic        color_threshold_enable,
    input  logic [7:0]  color_select,
    input  logic [3:0]  color_threshold_value,
    input  logic        learning_mode,      // sw[13]: Enable learning mode
    input  logic [2:0]  learning_buttons,   // btn[2:0] for teaching in learning mode
    
    // Processed outputs
    output logic [2:0]  processed_pixel,
    output logic [18:0] processed_addr,
    output logic        processed_valid,
    output logic        is_border_pixel,
    
    // Centroid outputs
    output logic [9:0]  centroid_x,
    output logic [9:0]  centroid_y,
    output logic        centroid_valid,
    output logic [19:0] blob_area,
    output logic [9:0]  centroid_bbox_min_x,
    output logic [9:0]  centroid_bbox_min_y,
    output logic [9:0]  centroid_bbox_max_x,
    output logic [9:0]  centroid_bbox_max_y,

    // Split centroid outputs (left/right halves)
    output logic [9:0]  left_centroid_x,
    output logic [9:0]  left_centroid_y,
    output logic        left_centroid_valid,
    output logic [9:0]  left_bbox_min_x,
    output logic [9:0]  left_bbox_min_y,
    output logic [9:0]  left_bbox_max_x,
    output logic [9:0]  left_bbox_max_y,
    output logic [9:0]  right_centroid_x,
    output logic [9:0]  right_centroid_y,
    output logic        right_centroid_valid,
    output logic [9:0]  right_bbox_min_x,
    output logic [9:0]  right_bbox_min_y,
    output logic [9:0]  right_bbox_max_x,
    output logic [9:0]  right_bbox_max_y,
    
    // Gesture outputs
    output logic        gesture_fist,
    output logic        gesture_open,
    output logic        gesture_wave
);

    //=============================================================================
    // STAGE 1: GRAYSCALE CONVERSION
    //=============================================================================
    logic [11:0] gray_pix_data;
    logic [3:0]  gray_4bit;
    logic [2:0]  gray_3bit;  // 3-bit grayscale for output
    // Skin mask (binary) from RGB444
    logic [3:0]  skin_mask_pixel;
    logic [2:0]  skin_mask_3bit;
    // Spatially filtered skin mask
    logic [3:0]  spatial_mask_pixel;
    logic [2:0]  spatial_mask_3bit;
    // Eroded skin mask
    logic [3:0]  eroded_mask_pixel;
    logic [2:0]  eroded_mask_3bit;
    // Dilated skin mask (after erosion)
    logic [3:0]  dilated_mask_pixel;
    logic [2:0]  dilated_mask_3bit;
    // Color threshold mask (binary) from RGB444
    logic [3:0]  color_mask_pixel;
    logic [2:0]  color_mask_3bit;
    logic [2:0]  spatial_color_mask_3bit;
    logic [3:0]  blob_mask_pixel;
    logic [2:0]  blob_mask_3bit;
    
    grayscale_converter gray_conv (
        .clk(clk),
        .enable(grayscale_enable),
        .rgb_in(cam_pix_data),
        .data_out(gray_pix_data)
    );
    
    // Extract 4-bit grayscale from green channel, then convert to 3-bit
    assign gray_4bit = gray_pix_data[7:4];
    assign gray_3bit = gray_4bit[3:1];  // Take top 3 bits, discard LSB
    
    // Convert all mask pixels from 4-bit to 3-bit
    assign skin_mask_3bit = skin_mask_pixel[3:1];
    assign spatial_mask_3bit = spatial_mask_pixel[3:1];
    assign eroded_mask_3bit = eroded_mask_pixel[3:1];
    assign dilated_mask_3bit = dilated_mask_pixel[3:1];
    assign color_mask_3bit = color_mask_pixel[3:1];
    assign spatial_color_mask_3bit = spatial_color_mask_pixel[3:1];
    assign blob_mask_3bit = blob_mask_pixel[3:1];

    // Skin thresholding directly from RGB444
    skin_threshold skin_thr (
        .clk(clk),
        .enable(skin_threshold_enable || spatial_filter_enable || erosion_enable || dilation_enable || centroid_enable),
        .r_in(cam_pix_data[11:8]),
        .g_in(cam_pix_data[7:4]),
        .b_in(cam_pix_data[3:0]),
        .y_min_tune(skin_y_min),
        .y_max_tune(skin_y_max),
        .mask_pixel(skin_mask_pixel)
    );

    // Color thresholding directly from RGB444
    color_threshold color_thr (
        .clk(clk),
        .enable(color_threshold_enable),
        .r_in(cam_pix_data[11:8]),
        .g_in(cam_pix_data[7:4]),
        .b_in(cam_pix_data[3:0]),
        .color_select(color_select),
        .threshold(color_threshold_value),
        .mask_pixel(color_mask_pixel)
    );

    // Spatially filtered color mask
    logic [3:0]  spatial_color_mask_pixel;

    // Line buffer for color mask when applying spatial filter
    logic [3:0] c00, c01, c02, c10, c11, c12, c20, c21, c22;
    logic       color_mask_neighborhood_valid;
    logic [9:0] color_mask_out_x, color_mask_out_y;

    line_buffer color_mask_line_buf (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_in(color_mask_pixel),
        .pixel_valid(cam_pix_write),
        .frame_start(frame_start),
        .p00(c00), .p01(c01), .p02(c02),
        .p10(c10), .p11(c11), .p12(c12),
        .p20(c20), .p21(c21), .p22(c22),
        .neighborhood_valid(color_mask_neighborhood_valid),
        .out_x(color_mask_out_x),
        .out_y(color_mask_out_y)
    );

    spatial_filter color_spatial_filt (
        .clk(clk),
        .enable(spatial_filter_enable && color_threshold_enable),
        .p00(c00), .p01(c01), .p02(c02),
        .p10(c10), .p11(c11), .p12(c12),
        .p20(c20), .p21(c21), .p22(c22),
        .pixel_out(spatial_color_mask_pixel)
    );

    // Line buffer for skin mask when applying spatial filter
    logic [3:0] m00, m01, m02, m10, m11, m12, m20, m21, m22;
    logic       mask_neighborhood_valid;
    logic [9:0] mask_out_x, mask_out_y;

    line_buffer mask_line_buf (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_in(skin_mask_pixel),
        .pixel_valid(cam_pix_write),
        .frame_start(frame_start),
        .p00(m00), .p01(m01), .p02(m02),
        .p10(m10), .p11(m11), .p12(m12),
        .p20(m20), .p21(m21), .p22(m22),
        .neighborhood_valid(mask_neighborhood_valid),
        .out_x(mask_out_x),
        .out_y(mask_out_y)
    );

    spatial_filter spatial_filt (
        .clk(clk),
        .enable(spatial_filter_enable),
        .p00(m00), .p01(m01), .p02(m02),
        .p10(m10), .p11(m11), .p12(m12),
        .p20(m20), .p21(m21), .p22(m22),
        .pixel_out(spatial_mask_pixel)
    );

    // Line buffer for erosion (operates on spatial output)
    logic [3:0] e00, e01, e02, e10, e11, e12, e20, e21, e22;
    logic       erosion_neighborhood_valid;
    logic [9:0] erosion_out_x, erosion_out_y;

    line_buffer erosion_line_buf (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_in(spatial_filter_enable ? spatial_mask_pixel : skin_mask_pixel),
        .pixel_valid(cam_pix_write),
        .frame_start(frame_start),
        .p00(e00), .p01(e01), .p02(e02),
        .p10(e10), .p11(e11), .p12(e12),
        .p20(e20), .p21(e21), .p22(e22),
        .neighborhood_valid(erosion_neighborhood_valid),
        .out_x(erosion_out_x),
        .out_y(erosion_out_y)
    );

    morphological_filter morph_erode (
        .clk(clk),
        .erosion_enable(erosion_enable),
        .dilation_enable(1'b0),
        .p00(e00), .p01(e01), .p02(e02),
        .p10(e10), .p11(e11), .p12(e12),
        .p20(e20), .p21(e21), .p22(e22),
        .pixel_out(eroded_mask_pixel)
    );

    // Line buffer for dilation (operates on erosion output)
    logic [3:0] d00, d01, d02, d10, d11, d12, d20, d21, d22;
    logic       dilation_neighborhood_valid;
    logic [9:0] dilation_out_x, dilation_out_y;

    line_buffer dilation_line_buf (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_in(erosion_enable ? eroded_mask_pixel : (spatial_filter_enable ? spatial_mask_pixel : skin_mask_pixel)),
        .pixel_valid(cam_pix_write),
        .frame_start(frame_start),
        .p00(d00), .p01(d01), .p02(d02),
        .p10(d10), .p11(d11), .p12(d12),
        .p20(d20), .p21(d21), .p22(d22),
        .neighborhood_valid(dilation_neighborhood_valid),
        .out_x(dilation_out_x),
        .out_y(dilation_out_y)
    );

    morphological_filter morph_dilate (
        .clk(clk),
        .erosion_enable(1'b0),
        .dilation_enable(dilation_enable),
        .p00(d00), .p01(d01), .p02(d02),
        .p10(d10), .p11(d11), .p12(d12),
        .p20(d20), .p21(d21), .p22(d22),
        .pixel_out(dilated_mask_pixel)
    );

    //=============================================================================
    // CENTROID DETECTION (operates on dilated mask or color mask)
    //=============================================================================
    logic [9:0] current_x, current_y;
    
    // Extract current pixel coordinates from address
    assign current_x = cam_pix_addr % 19'd640;
    assign current_y = cam_pix_addr / 19'd640;

    // Blob filter applied before centroid detection when enabled
    logic [9:0] prev_bbox_min_x, prev_bbox_min_y, prev_bbox_max_x, prev_bbox_max_y;
    logic        prev_bbox_valid;

    blob_filter blob_filter_inst (
        .clk(clk),
        .rst_n(rst_n),
        .enable(blob_filter_enable),
        .mask_pixel(is_border_centroid ? 4'h0 : dilated_mask_pixel),  // Force border to black
        .pixel_x(current_x),
        .pixel_y(current_y),
        .bbox_min_x(prev_bbox_min_x),
        .bbox_min_y(prev_bbox_min_y),
        .bbox_max_x(prev_bbox_max_x),
        .bbox_max_y(prev_bbox_max_y),
        .bbox_valid(prev_bbox_valid),
        .frame_start(frame_start),
        .pixel_out(blob_mask_pixel)
    );
    
    // Select mask source for centroid detection
    logic [3:0] centroid_mask_source;
    always_comb begin
        if (blob_filter_enable)
            centroid_mask_source = blob_mask_pixel;  // Use blob-filtered skin mask
        else if (color_threshold_enable && spatial_filter_enable)
            centroid_mask_source = spatial_color_mask_pixel;  // Use spatially filtered color mask
        else if (color_threshold_enable)
            centroid_mask_source = color_mask_pixel;  // Use color mask for color threshold/track
        else
            centroid_mask_source = dilated_mask_pixel;  // Use dilated mask for skin tracking
    end

    // Select coordinates and valid signal for centroid detection
    logic [9:0] centroid_x_coord, centroid_y_coord;
    logic       centroid_pixel_valid;
    logic       is_border_centroid;  // Border check for centroid
    
    // Border detection for centroid (exclude first/last 2 rows and columns to be safe)
    assign is_border_centroid = (current_x < 10'd2) || (current_x >= 10'd638) ||
                                (current_y < 10'd2) || (current_y >= 10'd478);
    
    always_comb begin
        if (color_threshold_enable && spatial_filter_enable) begin
            // Use color mask buffer coordinates when spatial filtering is enabled
            centroid_x_coord = color_mask_out_x;
            centroid_y_coord = color_mask_out_y;
            centroid_pixel_valid = cam_pix_write && color_mask_neighborhood_valid && !is_border_color_mask;
        end else if (dilation_enable) begin
            // Use dilation buffer coordinates
            centroid_x_coord = dilation_out_x;
            centroid_y_coord = dilation_out_y;
            centroid_pixel_valid = cam_pix_write && dilation_neighborhood_valid && !is_border_dilation;
        end else if (erosion_enable) begin
            // Use erosion buffer coordinates
            centroid_x_coord = erosion_out_x;
            centroid_y_coord = erosion_out_y;
            centroid_pixel_valid = cam_pix_write && erosion_neighborhood_valid && !is_border_erosion;
        end else if (spatial_filter_enable) begin
            // Use spatial filter buffer coordinates
            centroid_x_coord = mask_out_x;
            centroid_y_coord = mask_out_y;
            centroid_pixel_valid = cam_pix_write && mask_neighborhood_valid && !is_border_mask;
        end else begin
            // Use camera coordinates for direct color mask or skin mask
            // EXCLUDE BORDER PIXELS to prevent edge artifacts from affecting centroid
            centroid_x_coord = current_x;
            centroid_y_coord = current_y;
            centroid_pixel_valid = cam_pix_write && !is_border_centroid;
        end
    end
    
    centroid_detector #(
        .FRAME_WIDTH(640),
        .FRAME_HEIGHT(480),
        .MIN_BLOB_SIZE(300)  // Minimum 300 pixels for valid blob
    ) centroid_det (
        .clk(clk),
        .rst_n(rst_n),
        .enable(centroid_enable),
        
        // Input mask (use spatially filtered color mask, color mask, or dilated mask for skin)
        .mask_pixel(centroid_mask_source),
        .pixel_x(centroid_x_coord),
        .pixel_y(centroid_y_coord),
        .pixel_valid(centroid_pixel_valid),
        
        // Frame sync
        .frame_start(frame_start),
        
        // Centroid outputs
        .centroid_x(centroid_x),
        .centroid_y(centroid_y),
        .centroid_valid(centroid_valid),
        .blob_area(blob_area),
        .bbox_min_x(centroid_bbox_min_x),
        .bbox_min_y(centroid_bbox_min_y),
        .bbox_max_x(centroid_bbox_max_x),
        .bbox_max_y(centroid_bbox_max_y)
    );

    // Split centroid detector for multiplayer (left/right halves)
    split_centroid_detector #(
        .FRAME_WIDTH(640),
        .FRAME_HEIGHT(480),
        .MIN_BLOB_SIZE(300)
    ) split_centroid_det (
        .clk(clk),
        .rst_n(rst_n),
        .enable(split_centroid_enable),
        .mask_pixel(centroid_mask_source),
        .pixel_x(centroid_x_coord),
        .pixel_y(centroid_y_coord),
        .pixel_valid(centroid_pixel_valid),
        .frame_start(frame_start),
        // left
        .left_centroid_x(left_centroid_x),
        .left_centroid_y(left_centroid_y),
        .left_centroid_valid(left_centroid_valid),
        .left_blob_area(),
        .left_bbox_min_x(left_bbox_min_x),
        .left_bbox_min_y(left_bbox_min_y),
        .left_bbox_max_x(left_bbox_max_x),
        .left_bbox_max_y(left_bbox_max_y),
        // right
        .right_centroid_x(right_centroid_x),
        .right_centroid_y(right_centroid_y),
        .right_centroid_valid(right_centroid_valid),
        .right_blob_area(),
        .right_bbox_min_x(right_bbox_min_x),
        .right_bbox_min_y(right_bbox_min_y),
        .right_bbox_max_x(right_bbox_max_x),
        .right_bbox_max_y(right_bbox_max_y)
    );
    
    // Keep the last valid bounding box for blob filtering
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prev_bbox_min_x <= 10'd0;
            prev_bbox_min_y <= 10'd0;
            prev_bbox_max_x <= 10'd0;
            prev_bbox_max_y <= 10'd0;
            prev_bbox_valid <= 1'b0;
        end else begin
            if (centroid_valid) begin
                prev_bbox_min_x <= centroid_bbox_min_x;
                prev_bbox_min_y <= centroid_bbox_min_y;
                prev_bbox_max_x <= centroid_bbox_max_x;
                prev_bbox_max_y <= centroid_bbox_max_y;
                prev_bbox_valid <= 1'b1;
            end else if (frame_start) begin
                prev_bbox_valid <= 1'b0;
            end
        end
    end

    //=============================================================================
    // STAGE 2: LINE BUFFER (3x3 NEIGHBORHOOD)
    //=============================================================================
    logic [3:0] p00, p01, p02;  // Top row
    logic [3:0] p10, p11, p12;  // Middle row (p11 is center)
    logic [3:0] p20, p21, p22;  // Bottom row
    logic       neighborhood_valid;
    logic [9:0] spatial_out_x, spatial_out_y;
    
    // Vsync edge detection for frame start
    logic vsync_prev, frame_start;
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            vsync_prev <= 1'b0;
        else
            vsync_prev <= cam_vsync;
    end
    assign frame_start = cam_vsync & ~vsync_prev;
    
    line_buffer line_buf (
        .clk(clk),
        .rst_n(rst_n),
        .pixel_in(gray_4bit),
        .pixel_valid(cam_pix_write),
        .frame_start(frame_start),
        .p00(p00), .p01(p01), .p02(p02),
        .p10(p10), .p11(p11), .p12(p12),
        .p20(p20), .p21(p21), .p22(p22),
        .neighborhood_valid(neighborhood_valid),
        .out_x(spatial_out_x),
        .out_y(spatial_out_y)
    );

    //=============================================================================
    // STAGE 3: SPATIAL FILTERING (Median or Convolution)
    //=============================================================================
    
    // Median Filter
    logic [3:0] median_out;
    median_filter median_filt (
        .clk(clk),
        .enable(median_enable),
        .p00(p00), .p01(p01), .p02(p02),
        .p10(p10), .p11(p11), .p12(p12),
        .p20(p20), .p21(p21), .p22(p22),
        .pixel_out(median_out)
    );
    
    // Convolution Kernel (Gaussian/Sobel/Sharpen/Emboss)
    logic [3:0] conv_out;
    convolution_kernel conv_kern (
        .clk(clk),
        .enable(convolution_enable),
        .kernel_select(kernel_select),
        .p00(p00), .p01(p01), .p02(p02),
        .p10(p10), .p11(p11), .p12(p12),
        .p20(p20), .p21(p21), .p22(p22),
        .pixel_out(conv_out)
    );

    //=============================================================================
    // STAGE 4: THRESHOLDING
    //=============================================================================
    
    // Determine pixel to threshold (after median/conv or raw center pixel)
    // Keep as 4-bit for threshold module, then convert to 3-bit
    logic [3:0] pre_threshold_pixel_4bit;
    logic [2:0] pre_threshold_pixel;
    logic [2:0] median_out_3bit, conv_out_3bit, p11_3bit;
    
    always_comb begin
        if (color_threshold_enable && spatial_filter_enable)
            pre_threshold_pixel_4bit = spatial_color_mask_pixel; // spatially filtered color mask
        else if (color_threshold_enable)
            pre_threshold_pixel_4bit = color_mask_pixel; // binary color mask
        else if (dilation_enable)
            pre_threshold_pixel_4bit = dilated_mask_pixel; // dilated mask (after erosion)
        else if (erosion_enable)
            pre_threshold_pixel_4bit = eroded_mask_pixel; // eroded mask
        else if (spatial_filter_enable)
            pre_threshold_pixel_4bit = spatial_mask_pixel; // spatially filtered skin mask
        else if (skin_threshold_enable)
            pre_threshold_pixel_4bit = skin_mask_pixel; // binary skin mask
        else if (convolution_enable)
            pre_threshold_pixel_4bit = conv_out;
        else if (median_enable)
            pre_threshold_pixel_4bit = median_out;
        else
            pre_threshold_pixel_4bit = p11;
    end
    
    // Convert to 3-bit
    assign pre_threshold_pixel = pre_threshold_pixel_4bit[3:1];
    assign median_out_3bit = median_out[3:1];
    assign conv_out_3bit = conv_out[3:1];
    assign p11_3bit = p11[3:1];
    
    // Threshold module (works on 12-bit RGB, so replicate 4-bit across channels)
    logic [11:0] threshold_in, threshold_out;
    logic [3:0]  thresholded_pixel_4bit;
    logic [2:0]  thresholded_pixel;
    
    assign threshold_in = {pre_threshold_pixel_4bit, pre_threshold_pixel_4bit, pre_threshold_pixel_4bit};
    
    threshold threshold_inst (
        .enable(threshold_enable),
        .threshold_value(threshold_value),
        .rgb_in(threshold_in),
        .rgb_out(threshold_out)
    );
    
    assign thresholded_pixel_4bit = threshold_out[7:4];  // Extract 4-bit from green channel
    assign thresholded_pixel = thresholded_pixel_4bit[3:1];  // Convert to 3-bit

    //=============================================================================
    // STAGE 5: OUTPUT MUX & ADDRESS GENERATION
    //=============================================================================
    
    // Determine if any spatial processing is active
    logic spatial_active;
    assign spatial_active = median_enable || threshold_enable || convolution_enable || skin_threshold_enable || spatial_filter_enable || erosion_enable || dilation_enable || blob_filter_enable || color_threshold_enable;
    
    // Final processed pixel selection (3-bit)
    logic [2:0] spatial_pixel;
    assign spatial_pixel = threshold_enable ? thresholded_pixel : pre_threshold_pixel;
    
    // Border detection: First row + last row + first column + last column
    // Use spatial_out_x/spatial_out_y when neighborhood_valid to align with processed pixel center
    // Fall back to camera address during initial fill
    logic is_border;
    logic [9:0] x_cam, y_cam; // raw cam coordinates
    logic [9:0] x_proc, y_proc; // processed center coordinates
    logic [9:0] x_use, y_use;
    always_comb begin
        x_cam = cam_pix_addr % 19'd640;
        y_cam = cam_pix_addr / 19'd640;
        x_proc = spatial_out_x;
        y_proc = spatial_out_y;

        // Determine which coordinate space to use
        if (neighborhood_valid)
            {x_use, y_use} = {x_proc, y_proc};
        else
            {x_use, y_use} = {x_cam, y_cam};

        // Single-pixel border (true 3x3 kernel interior starts at (1,1) and ends at (638,478))
        is_border = (y_use == 10'd0) || (y_use == 10'd479) ||
                    (x_use == 10'd0) || (x_use == 10'd639);
    end

    // Border detection for mask-based spatial filtering
    logic is_border_mask;
    always_comb begin
        // Single-pixel border based on mask buffer coordinates
        is_border_mask = (mask_out_y == 10'd0) || (mask_out_y == 10'd479) ||
                         (mask_out_x == 10'd0) || (mask_out_x == 10'd639);
    end

    // Border detection for erosion
    logic is_border_erosion;
    always_comb begin
        is_border_erosion = (erosion_out_y == 10'd0) || (erosion_out_y == 10'd479) ||
                            (erosion_out_x == 10'd0) || (erosion_out_x == 10'd639);
    end

    // Border detection for dilation
    logic is_border_dilation;
    always_comb begin
        is_border_dilation = (dilation_out_y == 10'd0) || (dilation_out_y == 10'd479) ||
                             (dilation_out_x == 10'd0) || (dilation_out_x == 10'd639);
    end

    // Border detection for color mask spatial filter
    logic is_border_color_mask;
    always_comb begin
        is_border_color_mask = (color_mask_out_y == 10'd0) || (color_mask_out_y == 10'd479) ||
                               (color_mask_out_x == 10'd0) || (color_mask_out_x == 10'd639);
    end
    
    // Address conversion: (x, y) → linear address
    function automatic [18:0] xy_to_addr(input logic [9:0] x, input logic [9:0] y);
        xy_to_addr = y * 19'd640 + x;
    endfunction
    
    // Output assignment (all 3-bit)
    always_comb begin
        // Color threshold with spatial filter: use line buffer coordinates and handle borders
        if (color_threshold_enable && spatial_filter_enable && color_mask_neighborhood_valid && !is_border_color_mask) begin
            processed_pixel = spatial_color_mask_3bit;
            processed_addr  = xy_to_addr(color_mask_out_x, color_mask_out_y);
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b0;
        end else if (color_threshold_enable && spatial_filter_enable) begin
            // Border/invalid pixels during color spatial filter: output black
            processed_pixel = 3'b000;
            processed_addr  = cam_pix_addr;
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b1;
        end else if (color_threshold_enable) begin
            // Color threshold mode without spatial filter: bypasses neighborhood_valid/border gating (per-pixel classification)
            processed_pixel = color_mask_3bit;
            processed_addr  = cam_pix_addr; // one-to-one with incoming pixel
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b0;
        end else if (blob_filter_enable) begin
            // Blob filter mode: display the masked region near the centroid
            processed_pixel = blob_mask_3bit;
            processed_addr  = cam_pix_addr;
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b0;
        end else if (skin_threshold_enable && !spatial_filter_enable && !erosion_enable && !dilation_enable) begin
            processed_pixel = skin_mask_3bit;
            processed_addr  = cam_pix_addr; // one-to-one with incoming pixel
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b0;
        end else if (dilation_enable && dilation_neighborhood_valid && !is_border_dilation) begin
            // Dilation uses dilation buffer coordinates
            processed_pixel = dilated_mask_3bit;
            processed_addr  = xy_to_addr(dilation_out_x, dilation_out_y);
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b0;
        end else if (dilation_enable) begin
            // Border/invalid pixels during dilation: output black
            processed_pixel = 3'b000;
            processed_addr  = cam_pix_addr;
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b1;
        end else if (erosion_enable && erosion_neighborhood_valid && !is_border_erosion) begin
            // Erosion uses erosion buffer coordinates
            processed_pixel = eroded_mask_3bit;
            processed_addr  = xy_to_addr(erosion_out_x, erosion_out_y);
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b0;
        end else if (erosion_enable) begin
            // Border/invalid pixels during erosion: output black
            processed_pixel = 3'b000;
            processed_addr  = cam_pix_addr;
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b1;
        end else if (spatial_filter_enable && mask_neighborhood_valid && !is_border_mask) begin
            // Spatial-filtered mask uses mask buffer coordinates
            processed_pixel = spatial_mask_3bit;
            processed_addr  = xy_to_addr(mask_out_x, mask_out_y);
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b0;
        end else if (spatial_filter_enable) begin
            // Border/invalid pixels during spatial filter: output black (not grayscale)
            processed_pixel = 3'b000;
            processed_addr  = cam_pix_addr;
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b1;
        end else if (spatial_active && neighborhood_valid && !is_border) begin
            // Interior pixels: use spatial processing
            processed_pixel = spatial_pixel;
            processed_addr = xy_to_addr(spatial_out_x, spatial_out_y);
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b0;
        end else if (spatial_active && convolution_enable && kernel_select == 2'b01) begin
            // Sobel filter: border/invalid pixels should be black (edges don't exist at borders)
            // This handles both border pixels and pixels during line buffer fill
            processed_pixel = 3'b000;
            processed_addr = cam_pix_addr;
            processed_valid = cam_pix_write;
            is_border_pixel = 1'b1;
        end else begin
            // Border pixels or no spatial processing: passthrough grayscale
            processed_pixel = gray_3bit;
            // Maintain original camera address for border fill
            processed_addr = cam_pix_addr;
            processed_valid = cam_pix_write;
            is_border_pixel = is_border;
        end
    end

    //=============================================================================
    // GESTURE DETECTION (Perceptron-based Learning)
    //=============================================================================
    // Adaptive perceptron classifier that learns from user feedback
    // sw[13]=1: Learning mode - buttons teach gestures
    // btn[2]: fist, btn[1]: open, btn[0]: wave
    gesture_perceptron gesture_percep (
        .clk(clk),
        .rst_n(rst_n),
        .enable(gesture_enable),
        .frame_start(frame_start),
        .centroid_valid(centroid_valid),
        .centroid_x(centroid_x),
        .centroid_y(centroid_y),
        .bbox_min_x(centroid_bbox_min_x),
        .bbox_min_y(centroid_bbox_min_y),
        .bbox_max_x(centroid_bbox_max_x),
        .bbox_max_y(centroid_bbox_max_y),
        .blob_area(blob_area),
        .learning_mode(learning_mode),
        .learning_buttons(learning_buttons),
        .fist(gesture_fist),
        .open_hand(gesture_open),
        .wave(gesture_wave),
        .debug_activation_fist(),   // Optional: wire to LEDs for debugging
        .debug_activation_open(),
        .debug_activation_wave()
    );

endmodule
