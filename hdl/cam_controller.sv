/*
 * Camera Controller Module
 * 
 * Description:
 *   Top-level coordinator for OV7670 camera system.
 *   Manages initialization via SCCB and pixel capture to memory.
 * 
 * Purpose:
 *   Instantiate and connect camera initializer and interface modules.
 *   Provide unified control interface for camera system.
 * 
 * Notes:
 *   - Debouncing handled at processor level
 *   - Camera operates at 24MHz pixel clock (xclk)
 *   - SCCB operates at 400kHz
 */

module cam_controller #(
    parameter int unsigned CAM_CONFIG_CLK = 100_000_000
)(
    // System
    input  logic clk,
    input  logic rst_n_clk,
    input  logic rst_n_pclk,
    
    // Control signals
    input  logic cam_start,
    output logic cam_done,
    
    // Camera physical interface
    input  logic       pclk,
    input  logic [7:0] pix_byte,
    input  logic       vsync,
    input  logic       href,
    output logic       cam_reset,
    output logic       cam_pwdn,
    
    // SCCB interface
    output logic sioc,
    inout  wire  siod,
    
    // Memory interface
    output logic [11:0] pix_data,
    output logic [18:0] pix_addr,
    output logic        pix_write
);
    
    // ------------------------------------------------------------
    // Camera Control Signals
    // ------------------------------------------------------------
    assign cam_reset = 1'b1;  // 0: reset registers, 1: normal mode
    assign cam_pwdn  = 1'b0;  // 0: normal mode, 1: power down mode
    
    // ------------------------------------------------------------
    // Camera Initializer
    // ------------------------------------------------------------
    cam_initializer #(
        .CLK_F(CAM_CONFIG_CLK),
        .SCCB_F(400_000)
    ) camera_init (
        // System
        .clk(clk),
        .rst_n(rst_n_clk),
        
        // Control signals
        .cam_init_start(cam_start),
        .cam_init_done(cam_done),
        
        // SCCB Physical
        .sioc(sioc),
        .siod(siod),
        
        // Debug/testbench signals
        .data_sent_done(),
        .sccb_dout()
    );
    
    // ------------------------------------------------------------
    // Camera Pixel Interface
    // ------------------------------------------------------------
    cam_interface pixel_capture (
        // Camera pixel clock domain
        .pclk(pclk),
        .vsync(vsync),
        .href(href),
        .pixel_data(pix_byte),
        
        // Initialization status
        .cam_init_done(cam_done),
        
        // Memory interface
        .mem_addr(pix_addr),
        .mem_data(pix_data),
        .mem_write(pix_write)
    );

endmodule
