/*
 * Grayscale Converter Module
 * 
 * Description:
 *   Converts 12-bit RGB444 format to grayscale using luminance formula.
 *   Replicates grayscale value across RGB channels for display.
 * 
 * Purpose:
 *   Transform color image to grayscale for processing pipeline.
 *   Preserves perceived brightness using weighted channel contributions.
 * 
 * Notes:
 *   - Formula: Y = 0.299*R + 0.587*G + 0.114*B
 *   - Fixed-point approximation: Y = (77*R + 150*G + 29*B) / 256
 *   - Coefficients chosen for 8-bit precision without multiplication overflow
 */

module grayscale_converter (
    input  logic        clk,           // Clock for pipeline registers
    input  logic        enable,        // Enable grayscale conversion
    input  logic [11:0] rgb_in,        // 12-bit RGB input {R[3:0], G[3:0], B[3:0]}
    output logic [11:0] data_out       // 12-bit output (grayscale or passthrough)
);

    // ------------------------------------------------------------
    // Extract RGB Components
    // ------------------------------------------------------------
    logic [3:0] r, g, b;
    assign r = rgb_in[11:8];
    assign g = rgb_in[7:4];
    assign b = rgb_in[3:0];

    // ------------------------------------------------------------
    // Expand to 8-bit for Calculation Precision
    // ------------------------------------------------------------
    logic [7:0] r_expand, g_expand, b_expand;
    assign r_expand = {r, r};  // Replicate 4-bit to 8-bit
    assign g_expand = {g, g};
    assign b_expand = {b, b};

    // ------------------------------------------------------------
    // Luminance Calculation - Fixed Point Coefficients
    // Y = (77*R + 150*G + 29*B) / 256
    // ------------------------------------------------------------
    logic [15:0] r_weighted, g_weighted, b_weighted;
    logic [15:0] luminance_sum;
    logic [7:0]  gray_value;
    logic [3:0]  gray_4bit;

    always_comb begin
        r_weighted = r_expand * 8'd77;
        g_weighted = g_expand * 8'd150;
        b_weighted = b_expand * 8'd29;
        
        luminance_sum = r_weighted + g_weighted + b_weighted;
        
        // Divide by 256 (right shift by 8)
        gray_value = luminance_sum[15:8];
        
        // Convert back to 4-bit
        gray_4bit = gray_value[7:4];
    end

    // ------------------------------------------------------------
    // Output Mux
    // ------------------------------------------------------------
    always_comb begin
        if (enable) begin
            // Grayscale: replicate the gray value across all channels
            data_out = {gray_4bit, gray_4bit, gray_4bit};
        end else begin
            // Passthrough: output original RGB
            data_out = rgb_in;
        end
    end

endmodule
