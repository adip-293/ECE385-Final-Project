/*
 * Morphological Filter Module
 * 
 * Description:
 *   Implements erosion and dilation operations for binary masks.
 *   Removes noise and restores blob size through morphological opening.
 * 
 * Purpose:
 *   Clean up binary masks before centroid detection.
 *   Remove small false positives while preserving main blob.
 * 
 * Notes:
 *   - Erosion: Pixel white only if ALL 8 neighbors white (shrinks blobs)
 *   - Dilation: Pixel white if ANY of 9 pixels white (expands blobs)
 *   - Opening = Erosion then Dilation (removes noise, restores size)
 */

module morphological_filter (
    input  logic       clk,
    input  logic       erosion_enable,
    input  logic       dilation_enable,
    input  logic [3:0] p00, p01, p02,
    input  logic [3:0] p10, p11, p12,
    input  logic [3:0] p20, p21, p22,
    output logic [3:0] pixel_out
);

    // Module-scope temps
    logic c00, c01, c02, c10, c11, c12, c20, c21, c22; // all 9 pixels as binary flags
    logic all_neighbors_skin;   // For erosion: all 8 neighbors must be skin
    logic any_neighbor_skin;    // For dilation: any of 9 pixels (including center) is skin
    logic eroded_pixel;
    logic dilated_pixel;

    // Combinational morphological operations
    always_comb begin
        if (!erosion_enable && !dilation_enable) begin
            pixel_out = 4'h0;
        end else begin
            // Convert all 9 pixels to binary flags (nonzero = skin)
            c00 = |p00; c01 = |p01; c02 = |p02;
            c10 = |p10; c11 = |p11; c12 = |p12;
            c20 = |p20; c21 = |p21; c22 = |p22;

            // EROSION: center stays white only if ALL neighbors are white
            all_neighbors_skin = c00 & c01 & c02 & c10 & c12 & c20 & c21 & c22;
            eroded_pixel = c11 & all_neighbors_skin;

            // DILATION: center becomes white if ANY pixel in 3x3 is white
            any_neighbor_skin = c00 | c01 | c02 | c10 | c11 | c12 | c20 | c21 | c22;
            dilated_pixel = any_neighbor_skin;

            // Select operation based on enable signals
            if (erosion_enable) begin
                pixel_out = eroded_pixel ? 4'hF : 4'h0;
            end else if (dilation_enable) begin
                pixel_out = dilated_pixel ? 4'hF : 4'h0;
            end else begin
                pixel_out = 4'h0;
            end
        end
    end

endmodule
