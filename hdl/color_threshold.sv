/*
 * Color Threshold Module
 * 
 * Description:
 *   Performs color-based segmentation using Manhattan distance (L1 norm).
 *   Compares input RGB pixels against a user-defined reference color.
 * 
 * Purpose:
 *   Track colored objects (e.g., red ball, blue marker).
 *   Create binary mask based on color similarity.
 * 
 * Notes:
 *   - Manhattan distance = sum of absolute differences across RGB channels
 *   - More forgiving than Hamming distance for 4-bit color resolution
 *   - Reference color set via color_select[11:0] = {R[3:0], G[3:0], B[3:0]}
 *   - Threshold controls maximum allowed Manhattan distance (0-45 for 4-bit RGB)
 */

module color_threshold (
    input  logic        clk,
    input  logic        enable,
    input  logic [3:0]  r_in,         // 4-bit red channel
    input  logic [3:0]  g_in,         // 4-bit green channel
    input  logic [3:0]  b_in,         // 4-bit blue channel
    input  logic [11:0] ref_color,    // Reference color {R[3:0], G[3:0], B[3:0]}
    input  logic [5:0]  threshold,    // Manhattan distance threshold (0-45, max sum for 4-bit)
    
    output logic [3:0]  mask_pixel    // Binary output: 4'hF if match, 4'h0 otherwise
);

    // ===========================================================
    // Reference Color Extraction
    // ===========================================================
    logic [3:0] ref_r, ref_g, ref_b;
    
    assign ref_r = ref_color[11:8];
    assign ref_g = ref_color[7:4];
    assign ref_b = ref_color[3:0];
    
    // ===========================================================
    // Manhattan Distance Calculation
    // ===========================================================
    // Manhattan distance = |R_in - R_ref| + |G_in - G_ref| + |B_in - B_ref|
    // This is more forgiving than Hamming distance because it considers
    // the actual magnitude of color differences, not just bit flips
    
    logic [3:0] r_diff, g_diff, b_diff;  // Absolute differences per channel (0-15 each)
    logic [5:0] manhattan_dist;          // Total Manhattan distance (max 15+15+15 = 45 for 4-bit RGB)
    
    // Calculate absolute differences for each channel
    // For 4-bit values, difference can be 0-15
    always_comb begin
        if (r_in >= ref_r)
            r_diff = r_in - ref_r;
        else
            r_diff = ref_r - r_in;
            
        if (g_in >= ref_g)
            g_diff = g_in - ref_g;
        else
            g_diff = ref_g - g_in;
            
        if (b_in >= ref_b)
            b_diff = b_in - ref_b;
        else
            b_diff = ref_b - b_in;
    end
    
    // Sum of absolute differences (Manhattan distance)
    assign manhattan_dist = {2'b0, r_diff} + {2'b0, g_diff} + {2'b0, b_diff};
    
    // ===========================================================
    // Threshold Comparison
    // ===========================================================
    // If Manhattan distance <= threshold, pixel matches
    // Higher threshold = more forgiving (matches more colors)
    logic is_match;
    assign is_match = (manhattan_dist <= threshold);
    
    // ===========================================================
    // Output Generation
    // ===========================================================
    always_ff @(posedge clk) begin
        if (!enable)
            mask_pixel <= 4'h0;
        else
            mask_pixel <= is_match ? 4'hF : 4'h0;
    end

endmodule
