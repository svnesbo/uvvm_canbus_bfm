//-----------------------------------------------------------------------------
// Title      : CAN SystemVerilog BFM test bench
// Project    :
//-----------------------------------------------------------------------------
// File       : can_bfm_sv_tb.sv
// Author     : Simon Voigt Nesbø  <svn@hvl.no>
// Company    : Western Norway University of Applied Sciences
// Created    : 2018-10-15
// Last update: 2018-10-15
// Platform   : Simulation
// Standard   : System Verilog
//-----------------------------------------------------------------------------
// Description: Test bench for SystemVerilog version of CAN bus BFM
//              It tests the BFM by transmitting random packages to/from
//              an instance of the CAN controller from OpenCores,
//              and also by transmitting/receiving packages between two
//              instances of the BFM itself.
//-----------------------------------------------------------------------------
// Copyright (c)   2018
//-----------------------------------------------------------------------------
// Revisions  :
// Date        Version  Author        Description
// 2018-10-15  1.0      simon	      Created
//-----------------------------------------------------------------------------
`include "can_bfm.sv"
`include "can_regs.sv"

timeunit 1ns/100ps;

interface wishbone_if(input logic wb_clk);
   byte addr;
   byte data_master;
   byte data_slave;
   bit stb;
   bit cyc;
   bit we;
   bit ack;

   clocking cb @(posedge wb_clk);
      output           addr;
      output           data_master;
      input            data_slave;
      output           stb;
      output           cyc;
      output           we;
      input            ack;
   endclocking // cb

   modport bfm_if(clocking cb);
   modport fpga(input  addr,
                input  data_master,
                output data_slave,
                input  stb,
                input  cyc,
                input  we,
                output ack);
endinterface // wishbone_if

typedef struct {
   int         arb_lost_count;
   int         ack_received_count;
   int         crc_error_count;
   int         timeout_count;
   int         arb_id_mismatch_count;
   int         remote_bit_mismatch_count;
   int         ext_id_bit_mismatch_count;
   int         data_len_mismatch_count;
   int         data_byte_mismatch_count;
   int         success_count;
} result_t;

// Creates two instances of the CAN BFM class, one for each CAN interface
// Runs a test loop where random packages are written on one interface,
// and received on the other.
program can_bfm_tb_prog(canbus_if can_bfm_if1,
                        canbus_if can_bfm_if2,
                        //canbus_if can_ctrl_if,
                        wishbone_if.bfm_if wb_if,
                        output bit arb_lost,
                        output bit ack_received,
                        output bit timeout,
                        output bit crc_error,
                        output bit reset,
                        input bit can_ctrl_irq_n);

   parameter realtime              CAN_CTRL_IRQ_TIMEOUT = 1ms;

   can_bfm inst_can_bfm1;
   can_bfm inst_can_bfm2;
   result_t bfm_bfm_results = {0,0,0,0,0,0,0,0,0,0};
   result_t bfm_ctrl_results = {0,0,0,0,0,0,0,0,0,0};
   result_t ctrl_bfm_results = {0,0,0,0,0,0,0,0,0,0};

   int                       num_tests = 10;


   initial begin
      inst_can_bfm1 = new(can_bfm_if1, 1000000, 25, 1000000, 1, 1, 5, 3);
      inst_can_bfm2 = new(can_bfm_if2, 1000000, 25, 1000000, 1, 1, 5, 3);

      // Have to setup CAN controller even before it is used in TB,
      // otherwise can_tx is in unknown state in the CAN controller,
      // which obviously messes up the CAN signals for the BFM
      can_ctrl_setup();

      // Test loop: Send with BFM, receive with BFM
      for(int test_count = 0; test_count < num_tests; test_count++) begin
         test_can_bfm_bfm(bfm_bfm_results);
      end

      // Reinitialize CAN controller before use
      can_ctrl_setup();

      // Test loop: Send with BFM, receive with CAN controller
      for(int test_count = 0; test_count < num_tests; test_count++) begin
         test_can_bfm_ctrl(bfm_ctrl_results);
      end

      // Test loop: Send with CAN controller, receive with BFM
      for(int test_count = 0; test_count < num_tests; test_count++) begin
         test_can_ctrl_bfm(ctrl_bfm_results);
      end

      // Clear IRQ after sending. Necessary before trying to use receive task with the controller
      can_ctrl_wait_and_clr_irq();

      print_results("Transmit with BFM, receive with BFM", bfm_bfm_results, num_tests);
      print_results("Transmit with BFM, receive with CAN controller", bfm_ctrl_results, num_tests);
      print_results("Transmit with CAN controller, receive with BFM", ctrl_bfm_results, num_tests);

   end // initial begin

   task wb_write(input byte addr,
                 input byte data);
      wb_if.cb.stb         <= 1'b1;
      wb_if.cb.cyc         <= 1'b1;
      wb_if.cb.we          <= 1'b1;
      wb_if.cb.data_master <= data;
      wb_if.cb.addr        <= addr;

      wait(wb_if.cb.ack == 1'b1);
      @(wb_if.cb);

      wb_if.cb.stb  <= 1'b0;
      wb_if.cb.cyc  <= 1'b0;
      wb_if.cb.we   <= 1'b0;
   endtask; // wb_write

   task wb_read(input byte addr,
                output byte data);
      wb_if.cb.stb  <= 1'b1;
      wb_if.cb.cyc  <= 1'b1;
      wb_if.cb.we   <= 1'b0;
      wb_if.cb.addr <= addr;

      wait(wb_if.cb.ack == 1'b1);
      @(wb_if.cb);

      data = wb_if.cb.data_slave;
      wb_if.cb.stb  <= 1'b0;
      wb_if.cb.cyc  <= 1'b0;
      wb_if.cb.we   <= 1'b0;
   endtask; // wb_write

   // Performs wb_read() and compares read back data to data_exp
   task wb_check(input byte addr,
                 input byte data_exp);
      byte                  data;

      wb_read(addr, data);

      if(data != data_exp)
        $error("@%0dns: wb_mask_check: read back data (after masking): %0d, expected data: %0d",
               $time, data, data_exp);

   endtask; // wb_mask_check

   // Performs wb_read(), masks the read back data with mask, and compares to data_exp
   task wb_mask_check(input byte addr,
                      input byte mask,
                      input byte data_exp);
      byte                       data;

      wb_read(addr, data);

      data = data & mask;

      if(data != data_exp)
        $error("@%0dns: wb_mask_check: read back data (after masking): %0d, expected data: %0d",
               $time, data, data_exp);

   endtask; // wb_mask_check


   task can_ctrl_send_basic_mode(input can_package_t packet);
      byte tx_id1;
      byte tx_id2;
      byte can_irq_reg;

      wb_read(`C_CAN_IR, can_irq_reg); // Read out IR register to clear interrupts

      tx_id1 = packet.arb_id[10:3];
      tx_id2 = {packet.arb_id[2:0], packet.remote_frame, packet.data_length[3:0]};

      wb_write(`C_CAN_BM_TXB_ID1, tx_id1); // Set TXID1
      wb_write(`C_CAN_BM_TXB_ID2, tx_id2); // Set TXID2

      // Write payload bytes to TX buffer
      if(!packet.remote_frame && packet.data_length > 0) begin
         for(int byte_num = 0; byte_num < packet.data_length; byte_num++) begin
            wb_write(`C_CAN_BM_TXB_DATA1+byte_num, packet.data[byte_num]);
         end
      end

      wb_write(`C_CAN_CMR, 8'h01); // Request transmission on CAN0
    endtask // can_ctrl_send_basic_mode


    // Todo:
    // Can can controller receive both basic and extended frames when it is
    // in basic mode? Can one procedure do both, or are two procedures needed?
   task can_ctrl_recv_basic_mode (output can_package_t packet);
      byte rx_id1;
      byte rx_id2;
      bit remote_frame;
      byte can_irq_reg;

      timeout <= 1'b0;

      if(can_ctrl_irq_n == 1'b0) begin
         $error("@%0dns: can_ctrl_recv_basic_mode(): called with IRQ=0 already.", $time);
      end


      $display("@%0dns: can_ctrl_recv_basic_mode(): Waiting for IRQ.", $time);

      // Wait for irq signal or timeout
      fork : irq_wait
         begin
            #CAN_CTRL_IRQ_TIMEOUT;
            $display("@%0dns: can_ctrl_recv_basic_mode(): Timeout.", $time);
            disable irq_wait;
         end

         begin
            @(negedge can_ctrl_irq_n);
            $display("@%0dns: can_ctrl_recv_basic_mode(): Got IRQ.", $time);
            disable irq_wait;
         end
      join

      if(can_ctrl_irq_n == 1'b1) begin
         timeout <= 1'b1;
         $display("@%0dns: can_ctrl_recv_basic_mode(): Timeout while waiting for CAN controller to assert interrupt.", $time);
         return;
      end else begin
         $display("@%0dns: can_ctrl_recv_basic_mode(): Got IRQ.", $time);
      end

      wb_mask_check(`C_CAN_IR, 8'b00000001, 8'b00000001); // Check that receive interrupt was set

      wb_read(`C_CAN_BM_RXB_ID1, rx_id1); // Read out RXID1
      wb_read(`C_CAN_BM_RXB_ID2, rx_id2); // Read out RXID2

      packet.arb_id[10:3] = rx_id1;
      packet.arb_id[2:0]  = rx_id2[7:5];
      packet.remote_frame = rx_id2[4];
      packet.data_length  = rx_id2[3:0];

      // Read in data from buffer
      if(packet.remote_frame == 1'b0 && packet.data_length > 0) begin
         for(int byte_num = 0; byte_num < packet.data_length; byte_num++) begin
            wb_read(`C_CAN_BM_RXB_DATA1+byte_num, packet.data[byte_num]);
         end
      end

      wb_write(`C_CAN_CMR, 8'b00000100); // Release receive buffer
    endtask // can_ctrl_recv_basic_mode


   // Sets timeout = 1 if timeout was reached while waiting for IRQ
   task can_ctrl_wait_and_clr_irq();

      byte can_irq_reg;

      timeout <= 1'b0;

      $display("@%0dns: can_ctrl_wait_and_clr_irq(): Wait for IRQ.", $time);

      if(can_ctrl_irq_n == 1'b1) begin
         // Wait for irq signal (if not active already), or timeout
         fork : irq_wait
            begin
               #CAN_CTRL_IRQ_TIMEOUT;
               $display("@%0dns: can_ctrl_wait_and_clr_irq(): Timeout.", $time);
               disable irq_wait;
            end

            begin
               @(negedge can_ctrl_irq_n);
               $display("@%0dns: can_ctrl_wait_and_clr_irq(): Got IRQ.", $time);
               disable irq_wait;
            end
         join
      end

      if(can_ctrl_irq_n == 1'b0) begin
         $display("@%0dns: can_ctrl_wait_and_clr_irq(): Got IRQ.", $time);
         timeout <= 1'b0;
         wb_read(`C_CAN_IR, can_irq_reg); // Read out IR register to clear interrupts
      end else begin
         $display("@%0dns: can_ctrl_wait_and_clr_irq(): Timeout.", $time);
         timeout <= 1'b1;
         @(wb_if.cb);
      end
   endtask // can_ctrl_wait_and_clr_irq

   // Setup an instance of the OpenCores CAN controller
   // Configures bitrate registers etc.
   task can_ctrl_setup();
      reset <= 1'b1;
      #200;
      reset <= 1'b0;
      #200;

      wb_write(`C_CAN_BM_ACR, 8'hAA); // Acceptance code
      wb_write(`C_CAN_BM_AMR, 8'hFF); // Acceptance mask
      wb_write(`C_CAN_BTR0, 8'h01);   // 4x baud prescale and minimum synch jump width time
      wb_write(`C_CAN_BTR1, 8'h25);   // 7 baud clocks before and 3 after sampling point, tSEG1=6 and tSEG2=3
      wb_write(`C_CAN_BM_CR, 8'b00111110); // CAN0 interrupts enabled, operation mode

      #100000;
   endtask // can_ctrl_setup


   // Test of CAN BFM, with a BFM instance talking to OpenCores CAN controller
   // Runs one write and read test on can bus
   // 1. Generate random CAN package
   // 2. Transmit CAN package with CAN controller on can_ctrl_if,
   //    while receiving from BFM on can_bfm_if1
   // 3. Compare transmitted vs. received packages and check for errors
   task automatic test_can_ctrl_bfm(ref result_t res);
      // Generate random package to transmit
      // Allow remote frame requests, don't allow extended IDs (doesn't work yet..)
      can_package_t pkt_xmit = generate_random_packet(1'b1, 1'b0);
      can_package_t pkt_recv;

      $display("@%0dns: test_can_ctrl_bfm(): Starting transaction.", $time);

      fork
         can_ctrl_send_basic_mode(pkt_xmit);

         inst_can_bfm2.can_read(pkt_recv,
                                timeout,
                                crc_error);
      join

      $display("@%0dns: test_can_ctrl_bfm(): Transaction done.", $time);

      # 100 $display("@%0dns: test_can_ctrl_bfm(): Checking results.", $time);
      check_results(pkt_xmit, pkt_recv, res);
   endtask // test_can_ctrl_bfm


   // Test of CAN BFM, with a BFM instance talking to OpenCores CAN controller
   // Runs one write and read test on can bus
   // 1. Generate random CAN package
   // 2. Transmit CAN package with BFM on can_bfm_if1,
   //    while receiving from CAN controller on can_ctrl_if
   // 3. Compare transmitted vs. received packages and check for errors
   task automatic test_can_bfm_ctrl(ref result_t res);
      // Generate random package to transmit
      // Allow remote frame requests, don't allow extended IDs (doesn't work yet..)
      can_package_t pkt_xmit = generate_random_packet(1'b1, 1'b0);
      can_package_t pkt_recv;

      $display("@%0dns: test_can_bfm_ctrl(): Starting transaction.", $time);

      fork
         inst_can_bfm1.can_write(pkt_xmit,
                                 arb_lost,
                                 ack_received);

         can_ctrl_recv_basic_mode(pkt_recv);
      join

      $display("@%0dns: test_can_bfm_ctrl(): Transaction done.", $time);

      # 100 $display("@%0dns: test_can_bfm_ctrl(): Checking results.", $time);
      check_results(pkt_xmit, pkt_recv, res);
   endtask // test_can_bfm_ctrl


   // Test of CAN BFM, with two BFM instances talking together
   // Runs one write and read test on can bus
   // 1. Generate random CAN package
   // 2. Transmit CAN package on can_bfm_if1, while receiving on can_bfm_if2
   // 3. Compare transmitted vs. received packages and check for errors
   task automatic test_can_bfm_bfm(ref result_t res);
      // Generate random package to transmit
      // Allow remote frame requests and extended IDs
      can_package_t pkt_xmit = generate_random_packet(1'b1, 1'b1);
      can_package_t pkt_recv;

      fork
         inst_can_bfm1.can_write(pkt_xmit,
                                 arb_lost,
                                 ack_received);

         inst_can_bfm2.can_read(pkt_recv,
                                timeout,
                                crc_error);
      join

      # 100 $display("@%0dns: can_write and can_read finished. Checking results..", $time);
      check_results(pkt_xmit, pkt_recv, res);
   endtask // test_can_bfm_bfm


   function can_package_t generate_random_packet(bit rtr_frame_enable,
                                                 bit extended_frame_enable);
      can_package_t pkt;

      if(extended_frame_enable == 1'b1)
        pkt.extended_id = $urandom_range(1,0);
      else
        pkt.extended_id = 1'b0;

      pkt.arb_id = 0;

      if(pkt.extended_id)
        pkt.arb_id = $urandom_range(0,536870911);
      else
        pkt.arb_id[10:0] = $urandom_range(0,2047);

      pkt.data_length = $urandom_range(0,8);

      if(rtr_frame_enable == 1'b1)
        pkt.remote_frame = $urandom_range(1,0);
      else
        pkt.remote_frame = 1'b0;

      pkt.data = {0, 0, 0, 0, 0, 0, 0, 0};

      if(!pkt.remote_frame) begin
         for(int byte_count = 0; byte_count < pkt.data_length; byte_count++) begin
            pkt.data[byte_count] = $urandom_range(0,255);
         end
      end

      return pkt;
   endfunction


   function automatic check_results(ref can_package_t pkt_xmit,
                                    ref can_package_t pkt_recv,
                                    ref result_t res);
      automatic bit error_detected = 0;

      if(arb_lost == 1) begin
         $display("@%0dns: Error transmitting: arbitration lost.", $time);
         res.arb_lost_count++;
         error_detected = 1;
      end


      if(ack_received == 0) begin
         $display("@%0dns: Error transmitting: no ACK received.", $time);
         error_detected = 1;
      end else begin
         res.ack_received_count++;
      end

      if(crc_error == 1) begin
         $display("@%0dns: Error receiving: CRC error.", $time);
         res.crc_error_count++;
         error_detected = 1;
      end

      if(timeout == 1) begin
         $display("@%0dns: Error receiving: timeout.", $time);
         res.timeout_count++;
         error_detected = 1;
      end

      if(pkt_xmit.arb_id != pkt_recv.arb_id) begin
         $display("@%0dns: Error: received arbitration ID does not match transmitted ID.", $time);
         res.arb_id_mismatch_count++;
         error_detected = 1;
      end

      if(pkt_xmit.remote_frame != pkt_recv.remote_frame) begin
         $display("@%0dns: Error: RTR bit mismatch.", $time);
         res.remote_bit_mismatch_count++;
         error_detected = 1;
      end

      if(pkt_xmit.extended_id != pkt_recv.extended_id) begin
         $display("@%0dns: Error: Extended ID bit mismatch.", $time);
         res.ext_id_bit_mismatch_count++;
         error_detected = 1;
      end


      if(pkt_xmit.data_length != pkt_recv.data_length) begin
         $display("@%0dns: Error: DLC (data length) mismatch.", $time);
         res.data_len_mismatch_count++;
         error_detected = 1;
      end


      if(!pkt_xmit.remote_frame) begin
         for(int byte_count = 0; byte_count < pkt_xmit.data_length; byte_count++) begin
            if(pkt_xmit.data[byte_count] != pkt_recv.data[byte_count]) begin
               $display("@%0dns: Error: Payload data byte %0d mismatch.", $time, byte_count);
               res.data_byte_mismatch_count++;
               error_detected = 1;
            end
         end
      end

      if(!error_detected)
        res.success_count++;
   endfunction

   function automatic print_results(string message, ref result_t res, int test_count);
      $display("");
      $display("");
      $display(message);
      $display("----------------------------------------------------------------");
      $display("Number of successful tests: %0d of %0d tests", res.success_count, test_count);
      $display("Arbitration lost count: %0d", res.arb_lost_count);
      $display("ACK received count: %0d", res.ack_received_count);
      $display("CRC error count: %0d", res.crc_error_count);
      $display("Timeout count: %0d", res.timeout_count);
      $display("Arbitration ID mismatch count: %0d", res.arb_id_mismatch_count);
      $display("Remote (RTR) bit mismatch count: %0d", res.remote_bit_mismatch_count);
      $display("Extended ID bit mismatch count: %0d", res.ext_id_bit_mismatch_count);
      $display("Data length (DLC) mismatch count: %0d", res.data_len_mismatch_count);
      $display("Payload data byte mismatch count: %0d", res.data_byte_mismatch_count);
   endfunction // print_and_clear_results
endprogram


// Main testbench module for CAN BFM
module can_bfm_sv_tb();
   logic clk = 0;
   logic can_ctrl_rx;
   logic can_ctrl_tx = 1'b1;
   logic arb_lost;
   logic ack_received;
   logic timeout;
   logic crc_error;
   logic can_ctrl_bus_on_off;
   logic can_ctrl_irq_n;
   logic reset;

   always  #12.5  clk = ~clk;

   wishbone_if wb_if(.wb_clk(clk));

   canbus_if can_if1(.can_clk(clk));
   canbus_if can_if2(.can_clk(clk));

   can_connection #(3) can_conn(.can_rx({can_if1.can_rx,
                                         can_if2.can_rx,
                                         can_ctrl_rx}),
                                .can_tx({can_if1.can_tx,
                                         can_if2.can_tx,
                                         can_ctrl_tx}));

   can_top can_ctrl(.clk_i(clk),
                    .rx_i(can_ctrl_rx),
                    .tx_o(can_ctrl_tx),
                    .bus_off_on(can_ctrl_bus_on_off),
                    .irq_on(can_ctrl_irq_n),
                    .clkout_o(),
                    .wb_clk_i(clk),
                    .wb_rst_i(reset),
                    .wb_dat_i(wb_if.data_master),
                    .wb_dat_o(wb_if.data_slave),
                    .wb_cyc_i(wb_if.cyc),
                    .wb_stb_i(wb_if.stb),
                    .wb_we_i(wb_if.we),
                    .wb_adr_i(wb_if.addr),
                    .wb_ack_o(wb_if.ack));

   can_bfm_tb_prog can_bfm_tb_prog_inst(.can_bfm_if1(can_if1),
                                        .can_bfm_if2(can_if2),
                                        .wb_if(wb_if),
                                        .arb_lost(arb_lost),
                                        .ack_received(ack_received),
                                        .timeout(timeout),
                                        .crc_error(crc_error),
                                        .reset(reset),
                                        .can_ctrl_irq_n(can_ctrl_irq_n));
endmodule
