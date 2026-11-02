`timescale 1ns / 1ps
/////////////////////////////////////////////////////////////////////////////////
// Company       : �人о·��Ƽ����޹�˾
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn
// 
// Create Date   : 2021/12/20 00:00:00
// Module Name   : eth_udp_loopback_rgmii
// Description   : ��̫��rgmii���ز��Զ���
// 
// Dependencies  : 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
/////////////////////////////////////////////////////////////////////////////////

module eth_udp_loopback_rgmii(
  input         reset_n,
  //eth_rx
  input         rgmii_rx_clk_i,
  input  [3:0]  rgmii_rxd,
  input         rgmii_rxdv,
  //eth_tx
  output        rgmii_tx_clk,
  output  [3:0] rgmii_txd,
  output        rgmii_txen,
  
  output        eth_rstn,  
  output        eth_mdc,
  output        eth_mdio,    
  //led
  output [1:0]  led
);
parameter LOCAL_MAC  = 48'h00_0a_35_01_fe_c0;
parameter LOCAL_IP   = 32'hc0_a8_0a_0a;
parameter LOCAL_PORT = 16'd5000;
//eth_rx
wire        reset_p;
wire        clk125m;
wire [47:0] exter_mac;
wire [31:0] exter_ip;
wire [15:0] exter_port;
wire [15:0] rx_data_length;
wire        data_overflow;
wire [7:0]  rx_payload_dat;
wire        rx_payload_valid;
wire        rx_pkt_done;
wire        rx_pkt_err;
reg  [3:0]  pkt_right_cnt;
reg  [3:0]  pkt_err_cnt;
//eth_tx
wire        tx_done;
wire        tx_en_pulse;
wire [47:0] tx_dst_mac;
wire [31:0] tx_dst_ip;
wire [15:0] tx_dst_port;
wire [15:0] tx_data_length;
wire [7:0]  tx_payload_dat;
wire        tx_payload_req;

//eth_msg_buf interface
 wire        eth_msg_wr_en;
 wire [111:0]eth_msg_din;
 reg         eth_msg_rd_en;
 wire [111:0]eth_msg_dout;
 wire [4 : 0]eth_msg_dat_cnt;
 reg         eth_msg_rd_en_dly1;
 reg         eth_tx_state;  //1:tx  0:idle
// gmii interface
 wire gmii_rx_clk;
 wire [7:0] gmii_rxd;
 wire gmii_rxdv;
 
 wire gmii_tx_clk;
 wire [7:0] gmii_txd;
 wire gmii_txen;
 
 wire rgmii_rx_clk;

 assign eth_rstn = 1;
 assign eth_mdc = 1;
 assign eth_mdio = 1;
   
   clk_wiz_0 clk_wiz_0   //�����໷������λ��ͬʱ��ǿʱ���ȳ�����
   (
    // Clock out ports
    .clk_out1(rgmii_rx_clk),     // output clk_out1
    .clk_in1(rgmii_rx_clk_i),      // input clk_in1
    .locked (pll_locked)
    );   

  assign led = {pkt_err_cnt,pkt_right_cnt};
  assign reset_p = ~reset_n;
  
  always@(posedge clk125m or posedge reset_p)
  begin
    if(reset_p)
      pkt_right_cnt <= 4'd0;
    else if(~rx_pkt_err && rx_pkt_done)
      pkt_right_cnt <= pkt_right_cnt + 1'b1;
    else
      pkt_right_cnt <= pkt_right_cnt;
  end

  always@(posedge clk125m or posedge reset_p)
  begin
    if(reset_p)
      pkt_err_cnt <= 4'd0;
    else if(rx_pkt_err && rx_pkt_done)
      pkt_err_cnt <= pkt_err_cnt + 1'b1;
    else
      pkt_err_cnt <= pkt_err_cnt;  
  end
 
  rgmii_to_gmii rgmii_to_gmii(
  .reset(reset_p),

  .rgmii_rx_clk(rgmii_rx_clk),
  .rgmii_rxd(rgmii_rxd),
  .rgmii_rxdv(rgmii_rxdv),

  .gmii_rx_clk(gmii_rx_clk),
  .gmii_rxdv(gmii_rxdv),
  .gmii_rxd(gmii_rxd),
  .gmii_rxer( )
);
  
  //��̫������
  eth_udp_rx_gmii eth_udp_rx_gmii(
    .reset_p         (reset_p               ),

    .local_mac       (LOCAL_MAC             ),
    .local_ip        (LOCAL_IP              ),
    .local_port      (LOCAL_PORT            ),

    .clk125m_o       (clk125m               ),
    .exter_mac       (exter_mac             ),
    .exter_ip        (exter_ip              ),
    .exter_port      (exter_port            ),
    .rx_data_length  (rx_data_length        ),
    .data_overflow_i (data_overflow         ),
    .payload_valid_o (rx_payload_valid      ),
    .payload_dat_o   (rx_payload_dat        ),

    .one_pkt_done    (rx_pkt_done           ),
    .pkt_error       (rx_pkt_err            ),
    .debug_crc_check (                      ),

    .gmii_rx_clk     (gmii_rx_clk           ),
    .gmii_rxdv       (gmii_rxdv             ),
    .gmii_rxd        (gmii_rxd              )
  );


    //����̫���������ݻ���
  eth_data_buf eth_data_buf (
    .clk   (clk125m          ), // input wire clk
    .din   (rx_payload_dat   ), // input wire [7 : 0] din
    .wr_en (rx_payload_valid ), // input wire wr_en
    .rd_en (tx_payload_req   ), // input wire rd_en
    .dout  (tx_payload_dat   ), // output wire [7 : 0] dout
    .full  (data_overflow    ), // output wire full
    .empty (                 )  // output wire empty
  );
  //ͬʱ�Ա�����MAC��IP����Ϣ���ݻ���
  assign eth_msg_wr_en = rx_pkt_done;
  assign eth_msg_din   = {exter_mac,exter_ip,exter_port,rx_data_length};

  eth_msg_buf eth_msg_buf (
    .clk        (clk125m         ), // input wire clk
    .din        (eth_msg_din     ), // input wire [111 : 0] din
    .wr_en      (eth_msg_wr_en   ), // input wire wr_en
    .rd_en      (eth_msg_rd_en   ), // input wire rd_en
    .dout       (eth_msg_dout    ), // output wire [111 : 0] dout
    .full       (                ), // output wire full
    .empty      (                ), // output wire empty
    .data_count (eth_msg_dat_cnt )  // output wire [4 : 0] data_count
  );

  always@(posedge clk125m or posedge reset_p)
  begin
    if(reset_p)
      eth_tx_state <= 1'b0;
    else if(tx_done)
      eth_tx_state <= 1'b0;
    else if(eth_msg_dat_cnt >0)
      eth_tx_state <= 1'b1;
    else
      eth_tx_state <= eth_tx_state;
  end

  always@(posedge clk125m or posedge reset_p)
  begin
    if(reset_p)
      eth_msg_rd_en <= 1'b0;
    else if((eth_tx_state == 1'b0)&&(eth_msg_dat_cnt >0))
      eth_msg_rd_en <= 1'b1;
    else
      eth_msg_rd_en <= 1'b0;
  end

  always@(posedge clk125m)
    eth_msg_rd_en_dly1 <= eth_msg_rd_en;

  assign tx_en_pulse    = eth_msg_rd_en_dly1;
  assign tx_dst_mac     = eth_msg_dout[111:64];
  assign tx_dst_ip      = eth_msg_dout[63:32];
  assign tx_dst_port    = eth_msg_dout[31:16];
  assign tx_data_length = eth_msg_dout[15:0];
  
  eth_udp_tx_gmii eth_udp_tx_gmii
  (
    .clk125m       (clk125m               ),
    .reset_p       (reset_p               ),

    .tx_en_pulse   (tx_en_pulse           ),
    .tx_done       (tx_done               ),

    .dst_mac       (tx_dst_mac            ),
    .src_mac       (LOCAL_MAC             ), 
    .dst_ip        (tx_dst_ip             ),
    .src_ip        (LOCAL_IP              ),
    .dst_port      (tx_dst_port           ),
    .src_port      (LOCAL_PORT            ),
    

    .data_length   (tx_data_length        ),
    
    .payload_req_o (tx_payload_req        ),
    .payload_dat_i (tx_payload_dat        ),

    .gmii_tx_clk   (gmii_tx_clk                ),
    .gmii_txen     (gmii_txen             ),
    .gmii_txd      (gmii_txd              )
  );
  
 gmii_to_rgmii gmii_to_rgmii(
  .reset_n(reset_n),

  .gmii_tx_clk(gmii_tx_clk),
  .gmii_txd(gmii_txd),
  .gmii_txen(gmii_txen),
  .gmii_txer(1'b0),

  .rgmii_tx_clk(rgmii_tx_clk),
  .rgmii_txd(rgmii_txd),
  .rgmii_txen(rgmii_txen)
);
endmodule