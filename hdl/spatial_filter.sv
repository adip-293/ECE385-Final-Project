/*
 * Spatial Filter Module
 * 
 * Description:
 *   Noise reduction filter for binary skin masks using 3x3 neighborhood analysis.
 *   Clears isolated pixels with insufficient neighboring support.
 * 
 * Purpose:
 *   Remove small noise specs from skin detection mask.
 *   Improve blob quality before centroid detection.
 * 
 * Notes:
 *   - Rule: Clear pixel if >=3 of 8 neighbors are non-skin
 *   - Removes isolated white pixels and small clusters
 */

module spatial_filter (
    input  logic       clk,
    input  logic       enable,
    input  logic [3:0] p00, p01, p02,
    input  logic [3:0] p10, p11, p12,
    input  logic [3:0] p20, p21, p22,
    output logic [3:0] pixel_out
);

    // Module-scope temps to avoid local declarations in procedural blocks
    logic c00, c01, c02, c10, c12, c20, c21, c22; // neighbor skin flags (exclude center)
    logic center_skin;
    logic [3:0] neighbors_skin_count;
    logic [3:0] neighbors_non_skin;
    logic keep_center;

    // Combinational evaluation of neighbors
    always_comb begin
        if (!enable) begin
            pixel_out = 4'h0;
        end else begin
            // Treat any nonzero nibble as skin
            c00 = |p00; c01 = |p01; c02 = |p02;
            c10 = |p10;                c12 = |p12;
            c20 = |p20; c21 = |p21; c22 = |p22;
            center_skin = |p11;

            neighbors_skin_count = c00 + c01 + c02 + c10 + c12 + c20 + c21 + c22;
            neighbors_non_skin   = 4'd8 - neighbors_skin_count;

            // If at least 3 of 8 neighbors are non-skin, clear the pixel
            keep_center = center_skin && (neighbors_non_skin < 4'd3);

            pixel_out = keep_center ? 4'hF : 4'h0;
        end
    end

endmodule
