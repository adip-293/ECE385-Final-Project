/*
 * Convolution Kernel Module
 * 
 * Description:
 *   Applies 3x3 convolution kernels for image filtering.
 *   Supports Gaussian blur, Sobel edge detection, sharpen, and emboss.
 * 
 * Purpose:
 *   Perform spatial filtering using different convolution kernels.
 *   Enable edge detection, noise reduction, and image enhancement.
 * 
 * Notes:
 *   - Kernel formulas:
 *     Gaussian: [1 4 1; 4 16 4; 1 4 1] / 36
 *     Sobel: Gx=[-1 0 1;-2 0 2;-1 0 1], Gy=[-1 -2 -1;0 0 0;1 2 1], magnitude=|Gx|+|Gy|
 *     Sharpen: [0 -1 0; -1 5 -1; 0 -1 0]
 *     Emboss: [-2 -1 0; -1 1 1; 0 1 2]
 */

module convolution_kernel (
    input  logic        clk,
    input  logic        enable,        // When disabled, passthrough center pixel
    input  logic [1:0]  kernel_select, // 0=Gaussian, 1=Sobel, 2=Sharpen, 3=Emboss
    
    // 3x3 pixel neighborhood (4-bit grayscale each)
    input  logic [3:0]  p00, p01, p02,
    input  logic [3:0]  p10, p11, p12,
    input  logic [3:0]  p20, p21, p22,
    
    // Output pixel
    output logic [3:0]  pixel_out
);

    // Sobel edge detection
    // Gx (horizontal edges):        Gy (vertical edges):
    // [-1  0  1]                     [-1 -2 -1]
    // [-2  0  2]                     [ 0  0  0]
    // [-1  0  1]                     [ 1  2  1]
    
    logic signed [7:0] gx, gy;
    logic signed [8:0] gradient;
    logic [3:0] edge_magnitude;
    
    // Gaussian blur calculation
    logic [9:0] gaussian_sum;  // Increased bit width for stronger weights
    logic [9:0] rounded32;     // Rounded divide-by-32 approximation
    logic [3:0] blurred_pixel;
    
    // Sharpen calculation
    logic signed [7:0] sharpen_result;
    logic [3:0] sharpened_pixel;
    
    // Emboss calculation
    logic signed [7:0] emboss_result;
    logic [3:0] embossed_pixel;
    
    always_comb begin
        if (enable) begin
            case (kernel_select)
                2'b00: begin
                    // GAUSSIAN BLUR - stronger weighted average
                    // Kernel weights: [1 4 1; 4 16 4; 1 4 1] / 36
                    gaussian_sum = p00 + (p01 << 2) + p02 +
                                  (p10 << 2) + (p11 << 4) + (p12 << 2) +
                                   p20 + (p21 << 2) + p22;
                    
                    // Rounded divide-by-32 approximation to 36: add 16 before shifting
                    // This reduces truncation blackening of low-intensity pixels
                    rounded32 = (gaussian_sum + 10'd16) >> 5;

                    // Ensure a nonzero output when the sum is nonzero (min brightness clamp)
                    if ((gaussian_sum != 10'd0) && (rounded32 == 10'd0)) begin
                        blurred_pixel = 4'd1;
                    end else if (rounded32 > 10'd15) begin
                        blurred_pixel = 4'd15;
                    end else begin
                        blurred_pixel = rounded32[3:0];
                    end
                    pixel_out = blurred_pixel;
                end
                
                2'b01: begin
                    // SOBEL EDGE DETECTION
                    // Calculate Gx (horizontal gradient)
                    gx = -$signed({1'b0, p00}) + $signed({1'b0, p02})
                         -2*$signed({1'b0, p10}) + 2*$signed({1'b0, p12})
                         -$signed({1'b0, p20}) + $signed({1'b0, p22});
                    
                    // Calculate Gy (vertical gradient)
                    gy = -$signed({1'b0, p00}) - 2*$signed({1'b0, p01}) - $signed({1'b0, p02})
                         +$signed({1'b0, p20}) + 2*$signed({1'b0, p21}) + $signed({1'b0, p22});
                    
                    // Approximate magnitude: |Gx| + |Gy|
                    gradient = (gx[7] ? -gx : gx) + (gy[7] ? -gy : gy);
                    
                    // Clamp to 4-bit range (0-15)
                    if (gradient[8:2] > 7'd15)
                        edge_magnitude = 4'hF;
                    else
                        edge_magnitude = gradient[5:2];
                    
                    pixel_out = edge_magnitude;
                end
                
                2'b10: begin
                    // SHARPEN - edge enhancement
                    // Kernel: [0 -1 0; -1 5 -1; 0 -1 0]
                    sharpen_result = 5*$signed({1'b0, p11}) 
                                    -$signed({1'b0, p01}) 
                                    -$signed({1'b0, p10}) 
                                    -$signed({1'b0, p12}) 
                                    -$signed({1'b0, p21});
                    
                    // Clamp to 4-bit range
                    if (sharpen_result[7]) // Negative
                        sharpened_pixel = 4'h0;
                    else if (sharpen_result > 8'd15)
                        sharpened_pixel = 4'hF;
                    else
                        sharpened_pixel = sharpen_result[3:0];
                    
                    pixel_out = sharpened_pixel;
                end
                
                2'b11: begin
                    // EMBOSS - 3D relief effect
                    // Kernel: [-2 -1 0; -1 1 1; 0 1 2]
                    emboss_result = -2*$signed({1'b0, p00}) 
                                    -$signed({1'b0, p01})
                                    -$signed({1'b0, p10})
                                    +$signed({1'b0, p11})
                                    +$signed({1'b0, p12})
                                    +$signed({1'b0, p21})
                                    +2*$signed({1'b0, p22});
                    
                    // Add 8 to center around mid-gray, then clamp
                    emboss_result = emboss_result + 8'd8;
                    
                    if (emboss_result[7]) // Negative
                        embossed_pixel = 4'h0;
                    else if (emboss_result > 8'd15)
                        embossed_pixel = 4'hF;
                    else
                        embossed_pixel = emboss_result[3:0];
                    
                    pixel_out = embossed_pixel;
                end
                
                default: begin
                    // Default case - passthrough center pixel
                    pixel_out = p11;
                end
            endcase
            
        end else begin
            // Bypass mode - passthrough center pixel
            pixel_out = p11;
        end
    end

endmodule
