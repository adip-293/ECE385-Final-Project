/*
 * Skin Threshold Module
 * 
 * Description:
 *   Detects skin pixels using RGB-based color space thresholds.
 *   Implements Python-equivalent logic ported to FPGA hardware.
 * 
 * Purpose:
 *   Create binary skin mask for hand tracking and gesture detection.
 *   Filter non-skin regions from image for blob analysis.
 * 
 * Notes:
 *   - Converts RGB444 to RGB333 internally for calculation
 *   - Python algorithm: Y=(R+2G+B)>>2, Cr=R-G, Cb=B-G
 *   - Default thresholds: Y∈[2,6], Cr∈[1,4], Cb∈[-3,0], R>G, R-B>=2, G>=B
 *   - Tuning: y_min_tune and y_max_tune offset Y range (signed 4-bit)
 */

module skin_threshold (
    input  logic        clk,
    input  logic        enable,
    input  logic [3:0]  r_in,         // 4-bit RGB444
    input  logic [3:0]  g_in,
    input  logic [3:0]  b_in,

    // Tuning parameters (signed offsets for Y range)
    input  logic [3:0]  y_min_tune,   // Y min offset
    input  logic [3:0]  y_max_tune,   // Y max offset

    output logic [3:0]  mask_pixel    // 4'hF=skin, 4'h0=non-skin
);

    // ===========================================================
    // 1) Convert from RGB444 (0-15) ? RGB333 (0-7)
    // ===========================================================
    logic [2:0] r3, g3, b3;
    assign r3 = r_in[3:1];
    assign g3 = g_in[3:1];
    assign b3 = b_in[3:1];

    // ===========================================================
    // 2) Python-style computations
    // ===========================================================

    // -------- Luma Y = (R + 2G + B) >> 2 --------
    logic [5:0] y_accum;   // max is 7 + 2*7 + 7 = 28 ? fits in 6 bits
    logic [2:0] y_luma;
    assign y_accum = {3'b000, r3} + {2'b00, g3, 1'b0} + {3'b000, b3};
    assign y_luma  = y_accum[5:2];  // divide by 4

    // -------- Simple chroma differences (signed 4-bit) --------
    logic signed [3:0] cr;  // r3 - g3
    logic signed [3:0] cb;  // b3 - g3
    assign cr = $signed({1'b0,r3}) - $signed({1'b0,g3});
    assign cb = $signed({1'b0,b3}) - $signed({1'b0,g3});

    // ===========================================================
    // 3) Default Python thresholds (RGB333 domain)
    // ===========================================================

    // Base thresholds from Python code
    localparam int PY_YMIN      = 2;
    localparam int PY_YMAX      = 6;
    localparam int PY_CR_MIN    = 1;
    localparam int PY_CR_MAX    = 4;
    localparam int PY_CB_MIN    = -3;
    localparam int PY_CB_MAX    = 0;
    localparam int PY_RB_MIN    = 2;   // r3 - b3 >= 2

    // ===========================================================
    // 4) Interpret your 8-bit tuning bus
    // lower = ymin offset, upper = ymax offset (signed nibble)
    // ===========================================================

    logic signed [3:0] ymin_off, ymax_off;
    assign ymin_off = $signed(y_min_tune);
    assign ymax_off = $signed(y_max_tune);

    logic signed [3:0] y_min_dyn, y_max_dyn;
    assign y_min_dyn = PY_YMIN + ymin_off;
    assign y_max_dyn = PY_YMAX + ymax_off;

    // Clamp Y-range to 0-7 (3-bit domain)
    function automatic [2:0] clamp3(input int x);
        if (x < 0)      clamp3 = 3'd0;
        else if (x > 7) clamp3 = 3'd7;
        else            clamp3 = x[2:0];
    endfunction

    logic [2:0] y_min_final, y_max_final;
    assign y_min_final = clamp3(y_min_dyn);
    assign y_max_final = clamp3(y_max_dyn);

    // ===========================================================
    // 5) Apply Python logic conditions
    // ===========================================================
    logic cond_y;
    logic cond_cr, cond_cb;
    logic cond_r_gt_g, cond_r_b, cond_g_ge_b;

    assign cond_y     = (y_luma >= y_min_final) && (y_luma <= y_max_final);
    assign cond_cr    = (cr >= PY_CR_MIN) && (cr <= PY_CR_MAX);
    assign cond_cb    = (cb >= PY_CB_MIN) && (cb <= PY_CB_MAX);
    assign cond_r_gt_g = (r3 > g3);
    assign cond_r_b    = ($signed({1'b0,r3}) - $signed({1'b0,b3})) >= PY_RB_MIN;
    assign cond_g_ge_b = (g3 >= b3);

    logic is_skin_comb;
    assign is_skin_comb =
        cond_y       &&
        cond_cr      &&
        cond_cb      &&
        cond_r_gt_g  &&
        cond_r_b     &&
        cond_g_ge_b;

    // ===========================================================
    // 6) Register output mask
    // ===========================================================
    always_ff @(posedge clk) begin
        if (!enable)
            mask_pixel <= 4'h0;
        else
            mask_pixel <= is_skin_comb ? 4'hF : 4'h0;
    end

endmodule
