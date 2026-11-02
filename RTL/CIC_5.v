`timescale 1ns / 1ps

module CIC_5 #(
    parameter N = 5,              
    parameter R = 64,             
    parameter OUT_BITS = 32       
)(
    input  wire                  clk,       
    input  wire                  rst,       // Active-High
    input  wire                  ce,        // 3.072MHz Enable מה-TOP
    input  wire                  in_valid,  
    input  wire                  signal,    // כבר מסונכרן מה-TOP
    output reg  [OUT_BITS-1:0]   PCM_Out,   
    output reg                   pcm_valid  
);
     
    // --- 3. סנכרון ודגימה ---
    reg signal_sampled;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            signal_sampled <= 1'b0;
        end else if (ce) begin 
            signal_sampled <= signal;
        end
    end

    wire signed [1:0] pdm_data = signal_sampled ? 2'sd1 : -2'sd1;

    localparam INT_BITS = 2 + (N * 6); // עבור N=5 ו-R=64 יוצא 32 ביט פנימיים

    reg signed [INT_BITS-1:0] i1, i2, i3, i4, i5;
    reg [5:0] rate_cnt;
    reg sample_en;
    reg sample_en_d1;   // sample_en מוזז בסייקל אחד - מיושר עם c5 המעודכן

    reg signed [INT_BITS-1:0] i5_d1;
    reg signed [INT_BITS-1:0] c1, c1_d1;
    reg signed [INT_BITS-1:0] c2, c2_d1;
    reg signed [INT_BITS-1:0] c3, c3_d1;
    reg signed [INT_BITS-1:0] c4, c4_d1;
    reg signed [INT_BITS-1:0] c5;

    // --- אינטגרטורים (5 שלבים) ---
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            i1 <= 'd0;
            i2 <= 'd0;
            i3 <= 'd0;
            i4 <= 'd0;
            i5 <= 'd0;
        end else if (ce) begin 
            i1 <= i1 + pdm_data;
            i2 <= i2 + i1;
            i3 <= i3 + i2;
            i4 <= i4 + i3;
            i5 <= i5 + i4;
        end
    end

    // --- דצימציה ---
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            rate_cnt  <= 6'd0;
            sample_en <= 1'b0;
        end else if (ce) begin
            if (rate_cnt == R - 1) begin
                rate_cnt  <= 6'd0;
                sample_en <= 1'b1; 
            end else begin
                rate_cnt  <= rate_cnt + 1'b1;
                sample_en <= 1'b0;
            end
        end else begin
            sample_en <= 1'b0; 
        end
    end

    // --- דיליי של סייקל אחד ל-sample_en, ליישור מול c5 ---
    always @(posedge clk or posedge rst) begin
        if (rst)
            sample_en_d1 <= 1'b0;
        else
            sample_en_d1 <= sample_en;
    end

    // --- Comb (5 שלבים) ---
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            i5_d1     <= 'd0;
            c1        <= 'd0; c1_d1 <= 'd0;
            c2        <= 'd0; c2_d1 <= 'd0;
            c3        <= 'd0; c3_d1 <= 'd0;
            c4        <= 'd0; c4_d1 <= 'd0;
            c5        <= 'd0;
        end else if (sample_en) begin
            i5_d1 <= i5;
            c1    <= i5 - i5_d1;
            c1_d1 <= c1;
            c2    <= c1 - c1_d1;
            c2_d1 <= c2;
            c3    <= c2 - c2_d1;
            c3_d1 <= c3;
            c4    <= c3 - c3_d1;
            c4_d1 <= c4;
            c5    <= c4 - c4_d1;
        end
    end

//    // --- 4. Rounding (מעבר מ-32 ביט פנימי ל-16 ביט מוצא עם עיגול) ---
//    wire signed [INT_BITS-1:0] round_const = 1 << (INT_BITS - OUT_BITS - 1);
//    wire signed [INT_BITS-1:0] c5_rounded = c5 + round_const;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            PCM_Out   <= 'd0;
            pcm_valid <= 1'b0;
        end else if (sample_en_d1) begin 
            PCM_Out   <= c5; //_rounded[INT_BITS-9 : INT_BITS-OUT_BITS-8];
            pcm_valid <= 1'b1;
        end else begin
            pcm_valid <= 1'b0;
        end
    end

endmodule