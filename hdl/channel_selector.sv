/*
 * Channel Selector Module
 * 
 * Description:
 *   Selectively displays RGB color channels based on control signals.
 *   Enables independent viewing of R, G, and B contributions to image.
 * 
 * Purpose:
 *   Debug and demonstrate color channel contributions.
 *   Create false-color visualizations for analysis.
 * 
 * Notes:
 *   - channel_select encoding: [2]=R, [1]=G, [0]=B
 *   - Examples: 3'b111=color, 3'b100=red only, 3'b010=green only, 3'b001=blue only
 */

module channel_selector (
    input  logic        enable,           // Enable channel selection mode
    input  logic [2:0]  channel_select,   // Which channels to display
    input  logic [2:0]  red_in,           // Input red channel
    input  logic [2:0]  green_in,         // Input green channel
    input  logic [2:0]  blue_in,          // Input blue channel
    output logic [2:0]  red_out,          // Output red channel
    output logic [2:0]  green_out,        // Output green channel
    output logic [2:0]  blue_out          // Output blue channel
);

    always_comb begin
        if (enable) begin
            // Channel selection mode: mask channels based on select signal
            red_out   = channel_select[2] ? red_in   : 3'b000;
            green_out = channel_select[1] ? green_in : 3'b000;
            blue_out  = channel_select[0] ? blue_in  : 3'b000;
        end else begin
            // Passthrough mode: output all channels normally
            red_out   = red_in;
            green_out = green_in;
            blue_out  = blue_in;
        end
    end

endmodule
