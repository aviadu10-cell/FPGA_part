`timescale 1ns / 1ps

module TOP (
    input Clk,
    input reset_n,

   
    inout [35:0] Gpio0,
    inout [35:0] Gpio1,

    output [1:0] led,

    // --- Ethernet RGMII Interface ---
    input        rgmii_rx_clk_i,
    input  [3:0] rgmii_rxd,
    input        rgmii_rxdv,
    output       rgmii_tx_clk,
    output [3:0] rgmii_txd,
    output       rgmii_txen,

    output eth_rstn,
    output eth_mdc,
    output eth_mdio
);

    wire start_trigger = 1'b1;
    wire res = 1'b0;
    reg  interupt = 1'b0;
    wire [511:0] wave;
    wire valid_CIC;
    wire clk_CIC;
    
    always @(posedge Clk) begin
    if (res) begin
        interupt <= 1'b0; 
    end else if (start_trigger) begin
        interupt <= 1'b1; 
    end
    end
    
    wire start = interupt;
    
CIC_and_Ethernet_16_mic CIC_and_Ethernet_16_mic_inst(
    .Clk             (Clk           ),   
    .reset_n         (reset_n       ),
    .start           (start         ),
    .Gpio0           (Gpio0         ),
    .Gpio1           (Gpio1         ),
    .led             (led           ),
    .rgmii_rx_clk_i  (rgmii_rx_clk_i),
    .rgmii_rxd       (rgmii_rxd     ),
    .rgmii_rxdv      (rgmii_rxdv    ),
    .rgmii_tx_clk    (rgmii_tx_clk  ),
    .rgmii_txd       (rgmii_txd     ),
    .rgmii_txen      (rgmii_txen    ),
    .eth_rstn        (eth_rstn      ),
    .eth_mdc         (eth_mdc       ),
    .eth_mdio        (eth_mdio      ),
    .wave            (wave          ),
    .valid_CIC       (valid_CIC     ),
    .clk_CIC         (clk_CIC       )  
    );
   
   wire                  out_valid;
   wire  signed [35:0]   out_sum;  
   wire signed [31:0]   out_avg;
   
//beamforming #(
//    .AW(6),
//    .DELAYS({
//        6'd15, 6'd13, 6'd14, 6'd10,         // ch15..ch12
//        6'd6,  6'd11, 6'd8,  6'd4,          // ch11..ch8
//        6'd12, 6'd1,  6'd5,  6'd9,          // ch7..ch4
//        6'd2,  6'd7,  6'd3,  6'd0           // ch3..ch0
//    })
//)
//    beamforming_inst(
//      .clk        (clk_CIC    ),
//      .valid      (valid_CIC  ),
//      .rstn       (reset_n    ),         
//      .signal     (wave[511:0]),
//      .out_valid  (out_valid  ),
//      .out_sum    (out_sum    ),
//      .out_avg    (out_avg    )
//      );
    
//FFT FFT_inst(
//      .clk   (clk_CIC     ) ,          
//      .valid (valid_CIC   ) ,          
//      .rstn  (reset_n     ) ,         
//      .signal(wave[31:0]  ) 
//      );
    
endmodule