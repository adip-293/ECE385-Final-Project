`timescale 1ns / 1ps

/*
 * Pot Controller Module
 * 
 * Description:
 *   Potentiometer controller using XADC for analog input.
 *   Reads analog voltage from auxiliary channel 3.
 * 
 * Purpose:
 *   Provides analog input for tunable parameters (thresholds, color selection, etc.).
 *   Interfaces with XADC IP core for ADC conversion.
 * 
 * Notes:
 *   - Uses XADC channel 3 (auxiliary analog input)
 *   - Samples on drdy_out pulse and latches 16-bit ADC value
 *   - VP/VN are differential analog inputs
 */

module pot_controller(
    // System
    input logic clk,
    input logic reset,
    
    // Analog inputs
    input logic VP,
    input logic VN,
    
    // Digital output
    output logic [15:0] pot_out
);
    
    logic [6:0] daddr_in;
    logic [15:0] do_out;
    logic drdy_out;
    
    assign daddr_in = 7'd3;
    
    always_ff @ (posedge clk) begin
        if (drdy_out) begin
            pot_out <= do_out[15:0];
        end
    end
    
    xadc_wiz_0 pot (
        .daddr_in(daddr_in),
        .den_in(1'd1),
        .di_in(16'd0), //en signal
        
        .do_out(do_out),
        .drdy_out(drdy_out),
        .dwe_in(1'd0), //wen signal
        
        .vn_in (VN),
        .vp_in (VP),
        
        .dclk_in(clk),
        .reset_in(reset)
    );
    

endmodule
