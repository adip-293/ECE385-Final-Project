/*
 * Color Threshold Module
 * 
 * Description:
 *   Performs color-based segmentation using Hamming distance metric.
 *   Compares input RGB pixels against reference color palette.
 * 
 * Purpose:
 *   Track colored objects (e.g., red ball, blue marker).
 *   Create binary mask based on color similarity.
 * 
 * Notes:
 *   - Hamming distance = count of differing bits across RGB channels
 *   - 16-color palette selected via color_select[7:4]
 *   - Threshold controls maximum allowed bit differences
 */

module color_threshold (
    input  logic        clk,
    input  logic        enable,
    input  logic [3:0]  r_in,         // 4-bit red channel
    input  logic [3:0]  g_in,         // 4-bit green channel
    input  logic [3:0]  b_in,         // 4-bit blue channel
    input  logic [7:0]  color_select, // 8-bit potentiometer value to select reference color
    input  logic [3:0]  threshold,    // Hamming distance threshold (0-15)
    
    output logic [3:0]  mask_pixel    // Binary output: 4'hF if match, 4'h0 otherwise
);

    // ===========================================================
    // Reference Color Palette
    // ===========================================================
    // Define a set of reference colors for thresholding
    // Each color is stored as {R[3:0], G[3:0], B[3:0]}
    // Using color_select[7:4] to select from 16 colors
    // color_select[3:0] can be used for fine-tuning or additional colors
    
    logic [11:0] ref_color;  // Selected reference color {R, G, B}
    logic [3:0] ref_r, ref_g, ref_b;
    
    // Extract reference color components
    assign ref_r = ref_color[11:8];
    assign ref_g = ref_color[7:4];
    assign ref_b = ref_color[3:0];
    
    // Color palette: 16 predefined colors
    // Colors are selected based on color_select[7:4] (4 bits = 16 colors)
    always_comb begin
        case (color_select[7:4])
            4'h0: ref_color = 12'hF00;  // Red
            4'h1: ref_color = 12'h0F0;  // Green
            4'h2: ref_color = 12'h00F;  // Blue
            4'h3: ref_color = 12'hFF0;  // Yellow
            4'h4: ref_color = 12'hF0F;  // Magenta
            4'h5: ref_color = 12'h0FF;  // Cyan
            4'h6: ref_color = 12'hFFF;  // White
            4'h7: ref_color = 12'h000;  // Black
            4'h8: ref_color = 12'hF80;  // Orange
            4'h9: ref_color = 12'h80F;  // Purple
            4'hA: ref_color = 12'h8F0;  // Lime
            4'hB: ref_color = 12'hF88;  // Pink
            4'hC: ref_color = 12'h088;  // Teal
            4'hD: ref_color = 12'h880;  // Olive
            4'hE: ref_color = 12'h888;  // Gray
            4'hF: ref_color = 12'hC84;  // Brown
            default: ref_color = 12'h888; // Default to gray
        endcase
    end
    
    // ===========================================================
    // Hamming Distance Calculation
    // ===========================================================
    // Hamming distance = number of bits that differ
    // For each channel (R, G, B), calculate XOR to find differing bits
    // Then count the number of set bits in the XOR result
    
    logic [3:0] r_xor, g_xor, b_xor;  // XOR results for each channel
    logic [2:0] r_hamming, g_hamming, b_hamming;  // Hamming distance per channel
    logic [4:0] total_hamming;  // Total Hamming distance (max 12 bits = 4+4+4)
    
    // Calculate XOR for each channel
    assign r_xor = r_in ^ ref_r;
    assign g_xor = g_in ^ ref_g;
    assign b_xor = b_in ^ ref_b;
    
    // Count set bits in each XOR result (popcount)
    // This is the Hamming distance for each channel
    function automatic [2:0] popcount4(input logic [3:0] x);
        popcount4 = x[0] + x[1] + x[2] + x[3];
    endfunction
    
    assign r_hamming = popcount4(r_xor);
    assign g_hamming = popcount4(g_xor);
    assign b_hamming = popcount4(b_xor);
    
    // Total Hamming distance across all channels
    assign total_hamming = r_hamming + g_hamming + b_hamming;
    
    // ===========================================================
    // Threshold Comparison
    // ===========================================================
    // If total Hamming distance <= threshold, pixel matches
    logic is_match;
    assign is_match = (total_hamming <= {1'b0, threshold});
    
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

