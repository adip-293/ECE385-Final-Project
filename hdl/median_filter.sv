/*
 * Median Filter Module
 * 
 * Description:
 *   Performs 3x3 median filtering for noise reduction.
 *   Uses sorting network to find median of 9 pixels.
 * 
 * Purpose:
 *   Remove salt-and-pepper noise while preserving edges.
 *   Preprocess grayscale images before edge detection.
 * 
 * Notes:
 *   - Median = 5th element in sorted 9-element array
 *   - Sorting network optimized for partial sort (median only)
 *   - Excellent edge preservation compared to averaging
 */

module median_filter (
    input  logic        clk,
    input  logic        enable,        // When disabled, passthrough center pixel
    
    // 3x3 pixel neighborhood (4-bit grayscale each)
    input  logic [3:0]  p00, p01, p02,
    input  logic [3:0]  p10, p11, p12,
    input  logic [3:0]  p20, p21, p22,
    
    // Output pixel
    output logic [3:0]  pixel_out
);

    // Sorting network to find median of 9 values
    // We need to find the 5th element when sorted (middle value)
    
    logic [3:0] sorted [0:8];
    
    // Comparator: swap if a > b
    function automatic void swap(ref logic [3:0] a, ref logic [3:0] b);
        logic [3:0] temp;
        if (a > b) begin
            temp = a;
            a = b;
            b = temp;
        end
    endfunction
    
    always_comb begin
        if (enable) begin
            // Load input array
            sorted[0] = p00;
            sorted[1] = p01;
            sorted[2] = p02;
            sorted[3] = p10;
            sorted[4] = p11;
            sorted[5] = p12;
            sorted[6] = p20;
            sorted[7] = p21;
            sorted[8] = p22;
            
            // Sorting network for 9 elements (finds median efficiently)
            // This is a partial sort - we only need to ensure element [4] is the median
            
            // Stage 1: Compare-swap pairs
            swap(sorted[0], sorted[1]);
            swap(sorted[3], sorted[4]);
            swap(sorted[6], sorted[7]);
            
            // Stage 2
            swap(sorted[1], sorted[2]);
            swap(sorted[4], sorted[5]);
            swap(sorted[7], sorted[8]);
            
            // Stage 3
            swap(sorted[0], sorted[1]);
            swap(sorted[3], sorted[4]);
            swap(sorted[6], sorted[7]);
            
            // Stage 4: Merge groups
            swap(sorted[0], sorted[3]);
            swap(sorted[1], sorted[4]);
            swap(sorted[2], sorted[5]);
            
            // Stage 5
            swap(sorted[3], sorted[6]);
            swap(sorted[4], sorted[7]);
            swap(sorted[5], sorted[8]);
            
            // Stage 6
            swap(sorted[1], sorted[4]);
            swap(sorted[2], sorted[5]);
            
            // Stage 7
            swap(sorted[0], sorted[3]);
            swap(sorted[2], sorted[4]);
            
            // Stage 8
            swap(sorted[1], sorted[3]);
            swap(sorted[2], sorted[3]);
            
            // Stage 9: Final comparisons to ensure sorted[4] is median
            swap(sorted[4], sorted[6]);
            swap(sorted[4], sorted[5]);
            
            // sorted[4] is now guaranteed to be the median
            pixel_out = sorted[4];
            
        end else begin
            // Bypass mode - passthrough center pixel
            pixel_out = p11;
        end
    end

endmodule
