/*
 * Threshold Module
 * 
 * Description:
 *   Applies binary thresholding to grayscale images.
 *   Converts grayscale to binary (black/white) based on threshold value.
 * 
 * Purpose:
 *   Segment image regions based on intensity.
 *   Create binary masks for object detection.
 * 
 * Notes:
 *   - Pixels >= threshold become white (0xFFF)
 *   - Pixels < threshold become black (0x000)
 */

module threshold (
    input  logic        enable,           // Enable thresholding
    input  logic [3:0]  threshold_value,  // Threshold (0-15, matching 4-bit channel depth)
    input  logic [11:0] rgb_in,           // 12-bit RGB input
    output logic [11:0] rgb_out           // 12-bit thresholded output
);

    // Extract grayscale value (assuming R=G=B for grayscale input)
    // Use green channel as it typically has best SNR
    logic [3:0] gray_value;
    assign gray_value = rgb_in[7:4];  // Green channel [7:4]
    
    always_comb begin
        if (enable) begin
            // Binary thresholding
            if (gray_value >= threshold_value) begin
                // Above threshold: white
                rgb_out = 12'hFFF;
            end else begin
                // Below threshold: black
                rgb_out = 12'h000;
            end
        end else begin
            // Passthrough when disabled
            rgb_out = rgb_in;
        end
    end

endmodule
