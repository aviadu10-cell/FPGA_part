`timescale 1ns / 1ps

module CIC_and_Ethernet_16_mic_16bit (
    input Clk,
    input reset_n,
    input start,          // level: 1 = CIC + Ethernet run, 0 = held in reset

   
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
    output eth_mdio,
    output [255:0] wave,
    output valid_CIC,
    output clk_CIC
);

  // ------------------------------------------------------------------------
  // Local Parameters & Ethernet Config
  // ------------------------------------------------------------------------
  localparam LOCAL_MAC       = 48'h00_0a_35_01_fe_c0;
  localparam LOCAL_IP        = 32'hc0_a8_0a_0a;       // 192.168.10.10
  localparam LOCAL_PORT      = 16'd5000;

  localparam DEST_MAC        = 48'hff_ff_ff_ff_ff_ff; // Broadcast
  localparam DEST_IP         = 32'hc0_a8_0a_ff;       // 192.168.10.255
  localparam DEST_PORT       = 16'd5000;
  
  localparam PACKET_BYTE_LEN = 16'd3400;

  // ------------------------------------------------------------------------
  // Clocking & Safe Reset Synchronization
  // ------------------------------------------------------------------------
  wire clk_12m288;
  wire pll_locked;

  assign eth_rstn = reset_n;
  assign eth_mdc  = 1'b1;
  assign eth_mdio = 1'b1;

  clk_wiz_2 clk_12m (
      .clk_out1(clk_12m288),
      .reset(~reset_n),
      .locked(pll_locked),
      .clk_in1(Clk)
  );
  
  assign clk_CIC = clk_12m288;

  (* ASYNC_REG = "TRUE" *) reg [2:0] rst_sync_p = 3'b111;
  always @(posedge clk_12m288 or negedge reset_n) begin
    if (!reset_n) begin
      rst_sync_p <= 3'b111;
    end else begin
      rst_sync_p <= {rst_sync_p[1:0], ~pll_locked};
    end
  end
  wire sys_rst_p = rst_sync_p[2];

  // ------------------------------------------------------------------------
  // Start gating: synchronize 'start' into clk_12m288 and build run_rst.
  // run_rst = 1 -> CIC / counter / serializer / FIFO / Ethernet TX held in reset.
  // ------------------------------------------------------------------------
  (* ASYNC_REG = "TRUE" *) reg [1:0] start_sync_r;
  always @(posedge clk_12m288 or posedge sys_rst_p) begin
    if (sys_rst_p) start_sync_r <= 2'b00;
    else           start_sync_r <= {start_sync_r[0], start};
  end
  wire start_sync = start_sync_r[1];

  reg run_rst;
  always @(posedge clk_12m288 or posedge sys_rst_p) begin
    if (sys_rst_p) run_rst <= 1'b1;
    else           run_rst <= ~start_sync;   // clean synchronous release
  end

  // Same reset, brought into the clk125m domain (async assert, sync release)
  wire clk125m;
  (* ASYNC_REG = "TRUE" *) reg [1:0] run_rst_125_r;
  always @(posedge clk125m or posedge run_rst) begin
    if (run_rst) run_rst_125_r <= 2'b11;
    else         run_rst_125_r <= {run_rst_125_r[0], 1'b0};
  end
  wire run_rst_125 = run_rst_125_r[1];

// ------------------------------------------------------------------------
  // Generator 3.072MHz Clock & Enable Pulse
  // ------------------------------------------------------------------------
  wire clk_3m072;
  reg [1:0] div; 

  always @(posedge clk_12m288 or posedge sys_rst_p) begin
    if (sys_rst_p) div <= 2'b0;
    else div <= div + 1'b1;
  end

  assign clk_3m072 = div[1];
  wire ce_3m072 = (div == 2'b11);

  // ------------------------------------------------------------------------
  // Clock Forwarding using ODDR for Microphone Clocks (4 Pins)
  // ------------------------------------------------------------------------
  wire [3:0] mic_clk_out;
  
  
  assign Gpio0[5] = mic_clk_out[0];
  assign Gpio0[31] = mic_clk_out[1];
  assign Gpio1[5] = mic_clk_out[2];
  assign Gpio1[31] = mic_clk_out[3];

  genvar k;
  generate
    for (k = 0; k < 4; k = k + 1) begin : MIC_CLK_ODDR
      ODDR #(
          .DDR_CLK_EDGE("OPPOSITE_EDGE"), 
          .INIT(1'b0),                    
          .SRTYPE("SYNC")                 
      ) ODDR_inst (
          .Q(mic_clk_out[k]),
          .C(clk_12m288),    
          .CE(1'b1),         
          .D1(clk_3m072),    
          .D2(clk_3m072),    
          .R(sys_rst_p),     
          .S(1'b0)
      );
    end
  endgenerate

  // ------------------------------------------------------------------------
  // 32-Channel PDM Input Aggregation & CDC Synchronizer
  // ------------------------------------------------------------------------
 
  wire [15:0] pdm_raw = {
    Gpio0[29:27],
    Gpio0[32], 
    Gpio0[2:0],
    Gpio0[7],   
    Gpio1[29:27],
    Gpio1[32], 
    Gpio1[2:0],
    Gpio1[7]   
  };

  (* ASYNC_REG = "TRUE" *) reg [15:0] pdm_sync_stage1;
  (* ASYNC_REG = "TRUE" *) reg [15:0] pdm_sync_stage2;
  reg [15:0] pdm_sync;

  always @(posedge clk_12m288 or posedge sys_rst_p) begin
    if (sys_rst_p) begin
      pdm_sync_stage1 <= 16'd0;
      pdm_sync_stage2 <= 16'd0;
      pdm_sync        <= 16'd0;
    end else begin
      pdm_sync_stage1 <= pdm_raw;
      pdm_sync_stage2 <= pdm_sync_stage1;
      if (ce_3m072) pdm_sync <= pdm_sync_stage2;
    end
  end

 // ------------------------------------------------------------------------
  // 32 CIC & FIR Filters (Generate Block)
  // ------------------------------------------------------------------------
  wire [271:0]  cic_data_out;   // 16 ch x 16 bit + 16-bit Counter   
  wire [15:0]   cic_out_valid;
  wire [15:0]   s_axis_data_tready;

  wire [271:0]  fir_data_out;   // 16 ch x 16 bit + 16-bit Counter   
  wire [15:0]   fir_out_valid;

  // ------------------------------------------------------------------------
  // Sample / Frame Counter (Synced to clk_12m288)
  // ------------------------------------------------------------------------
  reg [15:0] Counter;
  
  // Held at 1 while start=0, so every run begins counting from 1
  always @(posedge clk_12m288 or posedge run_rst)
    if (run_rst) 
      Counter <= 16'd1;
    else if (cic_out_valid[0])
      Counter <= Counter + 1'b1;

  assign fir_data_out[271:256] = Counter;
  assign cic_data_out[271:256] = Counter;

  genvar j;
  generate
    for (j = 0; j < 16; j = j + 1) begin : DSP_CHANNELS
      
      CIC_filter #(
          .N(3),
          .R(64),
          .OUT_BITS(16)
      ) CIC_filter_inst (
          .clk      (clk_12m288),
          .rst      (run_rst),
          .ce       (ce_3m072),
          .in_valid (1'b1),
          .signal   (pdm_sync[j]),
          .PCM_Out  (cic_data_out[j*16+:16]),
          .pcm_valid(cic_out_valid[j])
      );
//      wire signed [7:0] pdm_axis_tdata = pdm_sync[j] ? 8'sd1 : 8'sd0;
//      wire [15:0] cic_pcm_out;
//      wire        cic_pcm_valid;

      // Xilinx Vivado CIC Compiler IP Instance
//      cic_compiler_0 cic_filter_inst (
//          .aclk               (clk_12m288),
//          .s_axis_data_tvalid (ce_3m072),
//          .s_axis_data_tready (),                
//          .s_axis_data_tdata  (pdm_axis_tdata),
//          .m_axis_data_tvalid (cic_pcm_valid),
//          .m_axis_data_tdata  (cic_pcm_out)
//      );

//      assign cic_data_out[j*16+:16] = cic_pcm_out;
//      assign cic_out_valid[j]       = cic_pcm_valid;

      // FIR Compiler Filter Instance
      fir_compiler_1 fir_compiler_1_inst (
          .aclk               (clk_12m288),
          .s_axis_data_tvalid (cic_out_valid[j]),
          .s_axis_data_tready (s_axis_data_tready[j]),
          .s_axis_data_tdata  (cic_data_out[j*16+:16]),
          .m_axis_data_tvalid (fir_out_valid[j]),
          .m_axis_data_tdata  (fir_data_out[j*16+:16])
      );
     end
   endgenerate
   
  assign wave = fir_data_out[255:0];
  assign valid_CIC = fir_out_valid[0];
  // ------------------------------------------------------------------------
  // 32-Channel PCM to 8-bit Serialization FSM
  // ------------------------------------------------------------------------
  reg [15:0] pcm_channels[0:16];
  reg [ 5:0] byte_cnt;
  reg        writing_to_fifo;
  reg [ 7:0] fifo_din;
  reg        fifo_wr_en;
  integer    i;
  
  always @(posedge clk_12m288 or posedge run_rst) begin
    if (run_rst) begin
      byte_cnt        <= 6'd0;
      writing_to_fifo <= 1'b0;
      fifo_wr_en      <= 1'b0;
      fifo_din        <= 8'd0;
      for (i = 0; i < 16; i = i + 1) begin
        pcm_channels[i] <= 16'd0;
      end
    end else begin
      if (cic_out_valid[0] && !writing_to_fifo) begin
        for (i = 0; i < 16; i = i + 1) begin
          pcm_channels[i] <= cic_data_out[(i*16) +: 16];
        end
        pcm_channels[16] <= cic_data_out[271:256];
        
        writing_to_fifo <= 1'b1;
        byte_cnt        <= 6'd0;
        fifo_wr_en      <= 1'b0;
      end else if (writing_to_fifo) begin
        if (byte_cnt < 6'd34) begin
          fifo_wr_en <= 1'b1;
          if (byte_cnt[0] == 1'b0) begin
            fifo_din <= pcm_channels[byte_cnt[5:1]][15:8];
          end else begin
            fifo_din <= pcm_channels[byte_cnt[5:1]][7:0];
          end
          byte_cnt <= byte_cnt + 1'b1;
        end else begin
          fifo_wr_en      <= 1'b0;
          writing_to_fifo <= 1'b0;
        end
      end else begin
        fifo_wr_en <= 1'b0;
      end
    end
  end


  // ------------------------------------------------------------------------
  // Ethernet Clocks & RX GMII Conversion
  // ------------------------------------------------------------------------
  wire gmii_rx_clk;
  wire [7:0] gmii_rxd;
  wire gmii_rxdv;
  wire rgmii_rx_clk;

  clk_wiz_0 clk_wiz_eth (
      .clk_out1(rgmii_rx_clk),
      .clk_in1 (rgmii_rx_clk_i)
  );

  rgmii_to_gmii rgmii_to_gmii_inst (
      .reset       (sys_rst_p),
      .rgmii_rx_clk(rgmii_rx_clk),
      .rgmii_rxd   (rgmii_rxd),
      .rgmii_rxdv  (rgmii_rxdv),
      .gmii_rx_clk (gmii_rx_clk),
      .gmii_rxdv   (gmii_rxdv),
      .gmii_rxd    (gmii_rxd),
      .gmii_rxer   ()
  );

  eth_udp_rx_gmii eth_udp_rx_gmii_inst (
      .reset_p    (sys_rst_p),
      .local_mac  (LOCAL_MAC),
      .local_ip   (LOCAL_IP),
      .local_port (LOCAL_PORT),
      .clk125m_o  (clk125m),
      .gmii_rx_clk(gmii_rx_clk),
      .gmii_rxdv  (gmii_rxdv),
      .gmii_rxd   (gmii_rxd)
  );

  // ------------------------------------------------------------------------
  // Async FIFO (Crossing clk_12m288 -> clk125m)
  // ------------------------------------------------------------------------
  wire [7:0] fifo_dout;
  wire tx_payload_req;
  wire [16:0] fifo_rd_data_count;

  fifo_generator_0 pcm_fifo (
      .rst(run_rst),
      .wr_clk(clk_12m288),
      .wr_en(fifo_wr_en),
      .din(fifo_din),

      .rd_clk(clk125m),
      .rd_en(tx_payload_req),
      .dout(fifo_dout),
      .rd_data_count(fifo_rd_data_count),
      .empty(),
      .full()
  );

  // ------------------------------------------------------------------------
  // UDP Transmit State Machine (Ethernet Clock Domain: clk125m)
  // ------------------------------------------------------------------------
  reg       tx_en_pulse;
  wire      tx_done;
  reg [1:0] eth_tx_fsm;

  always @(posedge clk125m or posedge run_rst_125) begin
    if (run_rst_125) begin
      tx_en_pulse <= 1'b0;
      eth_tx_fsm  <= 2'd0;
    end else begin
      case (eth_tx_fsm)
        2'd0: begin
          if (fifo_rd_data_count >= PACKET_BYTE_LEN) begin
            tx_en_pulse <= 1'b1;
            eth_tx_fsm  <= 2'd1;
          end
        end
        2'd1: begin
          tx_en_pulse <= 1'b0;
          eth_tx_fsm  <= 2'd2;
        end
        2'd2: begin
          if (tx_done) begin
            eth_tx_fsm <= 2'd0;
          end
        end
        default: eth_tx_fsm <= 2'd0;
      endcase
    end
  end

  // ------------------------------------------------------------------------
  // Ethernet UDP TX Modules & RGMII Output
  // ------------------------------------------------------------------------
  wire       gmii_tx_clk;
  wire [7:0] gmii_txd;
  wire       gmii_txen;

  eth_udp_tx_gmii eth_udp_tx_gmii_inst (
      .clk125m      (clk125m),
      .reset_p      (run_rst_125),
      .tx_en_pulse  (tx_en_pulse),
      .tx_done      (tx_done),
      .dst_mac      (DEST_MAC),
      .src_mac      (LOCAL_MAC),
      .dst_ip       (DEST_IP),
      .src_ip       (LOCAL_IP),
      .dst_port     (DEST_PORT),
      .src_port     (LOCAL_PORT),
      .data_length  (PACKET_BYTE_LEN),
      .payload_req_o(tx_payload_req),
      .payload_dat_i(fifo_dout),
      .gmii_tx_clk  (gmii_tx_clk),
      .gmii_txen    (gmii_txen),
      .gmii_txd     (gmii_txd)
  );

  gmii_to_rgmii gmii_to_rgmii_inst (
      .reset_n     (reset_n),
      .gmii_tx_clk (gmii_tx_clk),
      .gmii_txd    (gmii_txd),
      .gmii_txen   (gmii_txen),
      .gmii_txer   (1'b0),
      .rgmii_tx_clk(rgmii_tx_clk),
      .rgmii_txd   (rgmii_txd),
      .rgmii_txen  (rgmii_txen)
  );

  // Diagnostics LEDs
  assign led[0] = (fifo_rd_data_count >= PACKET_BYTE_LEN);
  assign led[1] = tx_done;

    reg [31:0] counter_ILA;
    always @(posedge clk_12m288 or negedge reset_n)
    if (!reset_n) counter_ILA <= 32'h00000000;
    else counter_ILA <= counter_ILA + 1;

//    ila_0 ila_0_inst (
//        .clk(clk_12m288),
//        .probe0(cic_data_out[15:0]),   // 16 bit
//        .probe1(counter_ILA),
//        .probe2(fir_out_valid[0]),
//        .probe3(sys_rst_p),
//        .probe4(pdm_sync[0]),
//        .probe5(fir_data_out[15:0]), // ch0, 16 bit
//        .probe6(fir_data_out[31:16]), // ch1, 16 bit
//        .probe7(fir_data_out[47:32]), // ch2, 16 bit
//        .probe8(fir_data_out[63:48]), // ch3, 16 bit
//        .probe9(fir_data_out[79:64]), // ch4, 16 bit
//        .probe10(fir_data_out[95:80]), // ch5, 16 bit
//        .probe11(fir_data_out[111:96]), // ch6, 16 bit
//        .probe12(fir_data_out[127:112]), // ch7, 16 bit
//        .probe13(fir_data_out[143:128]), // ch8, 16 bit
//        .probe14(fir_data_out[159:144]), // ch9, 16 bit
//        .probe15(fir_data_out[175:160]), // ch10, 16 bit
//        .probe16(fir_data_out[191:176]), // ch11, 16 bit
//        .probe17(fir_data_out[207:192]), // ch12, 16 bit
//        .probe18(fir_data_out[223:208]), // ch13, 16 bit
//        .probe19(fir_data_out[239:224]), // ch14, 16 bit
//        .probe20(fir_data_out[255:240])  // ch15, 16 bit
//    );

endmodule
