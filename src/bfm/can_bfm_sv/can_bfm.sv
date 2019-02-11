//-----------------------------------------------------------------------------
// Title      : CAN bus SystemVerilog BFM
// Project    : 
//-----------------------------------------------------------------------------
// File       : can_bfm.sv
// Author     : Simon Voigt Nesbø  <svn@hvl.no>
// Company    : Western Norway University of Applied Sciences
// Created    : 2018-10-15
// Last update: 2018-10-15
// Platform   : Simulation
// Standard   : System Verilog
//-----------------------------------------------------------------------------
// Description: Bus Functional Model (BFM) for CAN bus written in SystemVerilog
//-----------------------------------------------------------------------------
// Copyright (c)   2018
//-----------------------------------------------------------------------------
// Revisions  :
// Date        Version  Author        Description
// 2018-10-15  1.0      simon	      Created
//-----------------------------------------------------------------------------
//`include "canbus_if.sv"

`ifndef CAN_BFM_INCLUDED_V
`define CAN_BFM_INCLUDED_V

// Bit positions/indexes of fields in standard CAN frame
`define      C_STD_SOF_INDEX       0  // Start Of Frame
`define      C_STD_ARB_ID_INDEX    1  // Arbitration ID
`define      C_STD_RTR_INDEX      12  // Remote Transmission Request
`define      C_STD_IDE_INDEX      13  // Identifier Extension Bit
`define      C_STD_R0_INDEX       14  // Reserved Bit 0 (dominant zero)
`define      C_STD_DLC_INDEX      15  // Data Length Code
`define      C_STD_DATA_INDEX     19  // Data

// Bit positions/indexes of fields in extended CAN frame
`define      C_EXT_SOF_INDEX       0  // Start Of Frame
`define      C_EXT_ARB_ID_A_INDEX  1  // Arbitration ID A
`define      C_EXT_SRR_INDEX      12  // Substitute Remote Request
`define      C_EXT_IDE_INDEX      13  // Identifier Extension Bit
`define      C_EXT_ARB_ID_B_INDEX 14  // Arbitration ID A
`define      C_EXT_RTR_INDEX      32  // Remote Transmission Request
`define      C_EXT_R0_INDEX       33  // Reserved Bit 0 (dominant zero)
`define      C_EXT_R1_INDEX       34  // Reserved Bit 1 (dominant zero)
`define      C_EXT_DLC_INDEX      35  // Data Length Code
`define      C_EXT_DATA_INDEX     39  // Data

// Bit positions of CRC, ACK, and EOF bits.
//
`define      C_CRC_INDEX           0  // Cyclic Redundancy Check
`define      C_CRC_DELIM_INDEX    15  // CRC delimiter (recessive 1)
`define      C_ACK_SLOT_INDEX     16  // ACK slot (transmitter sends
                                      // recessive 1, receiver can
                                      // assert dominant 0)

`define      C_ACK_DELIM_INDEX    17  // ACK delimiter (recessive 1)
`define      C_EOF_INDEX          18  // End Of Frame (recessive 1)

// Field sizes
`define      C_STD_ARB_ID_SIZE    11  // Size of arbitration ID for standard frame
`define      C_EXT_ARB_ID_A_SIZE  11  // Size of first part of arbitration ID for extended frame
`define      C_EXT_ARB_ID_B_SIZE  18  // Size of second part of arbitration ID for extended frame
`define      C_DLC_SIZE            4  // Size of DLC field
`define      C_EOF_SIZE            7  // 7 End Of Frame bits (recessive 1)
`define      C_IFS_SIZE            3  // 3 Interframe Spacing bits (recessive 1)
`define      C_EOF_IFS_SIZE       10
`define      C_CRC_SIZE           15

timeunit 1ns/100ps;


typedef struct {
   bit [28:0]  arb_id;
   bit         extended_id;
   bit         remote_frame;
   int         data_length;
   byte        data[7:0];
} can_package_t;


class can_bfm;
   virtual     canbus_if sigs;

   int            m_bit_rate;  // bitrate per second
   realtime       m_clock_period;
   realtime       m_timeout;

   int            m_sync_quanta;
   int            m_prop_quanta;
   int            m_phase1_quanta;
   int            m_phase2_quanta;

   realtime       m_bit_period;
   realtime       m_bit_quanta;
   int            m_bit_quanta_cycles;
   int            m_sync_cycles;
   int            m_prop_cycles;
   int            m_phase1_cycles;
   int            m_phase2_cycles;
   int            m_sample_point_cycles;
   int            m_timeout_cycles;

   task can_init();
      sigs.can_tx <= 1'b1;
      #10000;
   endtask

   task can_write(input can_package_t can_package,
                  output bit arb_lost,
                  output bit ack_received);

      bit [0:200]            bit_buffer;
      int                    byte_start_index = 0;
      int                    byte_end_index = 0;
      int                    frame_end_index = 0;
      int                    crc_start_index = 0;
      bit [14:0]             crc;
      bit                    previous_bit_value = 0;
      int                    bit_stuffing_counter = 0;

      arb_lost = 0;
      ack_received = 0;
      sigs.can_tx <= 1;

      ////////////////////////////////////////////////////////////////////////////
      // Initialize bit buffer for CAN frame
      ////////////////////////////////////////////////////////////////////////////

      if(can_package.extended_id == 0) begin
         /////////////////////////////////////////////////////////////////////////
         // Standard frame
         /////////////////////////////////////////////////////////////////////////
         bit_buffer[`C_STD_SOF_INDEX] = 0;
         bit_buffer[`C_STD_ARB_ID_INDEX +: `C_STD_ARB_ID_SIZE] = can_package.arb_id[10:0];
         bit_buffer[`C_STD_RTR_INDEX] = can_package.remote_frame;
         bit_buffer[`C_STD_IDE_INDEX] = can_package.extended_id;
         bit_buffer[`C_STD_R0_INDEX] = 0;
         bit_buffer[`C_STD_DLC_INDEX +: `C_DLC_SIZE] = can_package.data_length;

         if(can_package.remote_frame == 0) begin
            for(int byte_counter = 0; byte_counter < can_package.data_length; byte_counter++) begin
               byte_start_index = `C_STD_DATA_INDEX+byte_counter*8;
               //byte_end_index = byte_start_index+7;
               bit_buffer[byte_start_index +: 8] = can_package.data[byte_counter];
            end
            crc_start_index = `C_STD_DATA_INDEX+(can_package.data_length*8);
         end else begin
            crc_start_index = `C_STD_DATA_INDEX;
         end

         crc = calc_can_crc15(bit_buffer, crc_start_index);
         bit_buffer[crc_start_index +: `C_CRC_SIZE] = crc;

         bit_buffer[crc_start_index+`C_CRC_DELIM_INDEX] = 1;
         bit_buffer[crc_start_index+`C_ACK_SLOT_INDEX] = 1;
         bit_buffer[crc_start_index+`C_ACK_DELIM_INDEX] = 1;

         frame_end_index = crc_start_index+`C_EOF_INDEX+`C_EOF_SIZE+`C_IFS_SIZE-1;

         bit_buffer[crc_start_index+`C_EOF_INDEX +: `C_EOF_IFS_SIZE] = '1;
      end else begin
         /////////////////////////////////////////////////////////////////////////
         // Extended frame
         /////////////////////////////////////////////////////////////////////////
         bit_buffer[`C_EXT_SOF_INDEX] = 0;
         //bit_buffer[`C_EXT_ARB_ID_A_INDEX:`C_EXT_ARB_ID_A_INDEX+10] = can_package.arb_id[10:0];
         bit_buffer[`C_EXT_ARB_ID_A_INDEX +: `C_EXT_ARB_ID_A_SIZE] = can_package.arb_id[10:0];

         // RTR bit has a different position in extended frame.
         // The SRR bit is placed in the same position as RTR a in standard frame.
         // SRR should always be 1 (recessive) for extended frame, effectively
         // giving an extended frame lower priority than a standard frame
         bit_buffer[`C_EXT_SRR_INDEX] = 1;
         bit_buffer[`C_EXT_IDE_INDEX] = can_package.extended_id;
         //bit_buffer[`C_EXT_ARB_ID_B_INDEX:`C_EXT_ARB_ID_B_INDEX+17] = can_package.arb_id[28:11];
         bit_buffer[`C_EXT_ARB_ID_B_INDEX +: `C_EXT_ARB_ID_B_SIZE] = can_package.arb_id[28:11];
         bit_buffer[`C_EXT_RTR_INDEX] = can_package.remote_frame;
         bit_buffer[`C_EXT_R0_INDEX] = 0;
         bit_buffer[`C_EXT_R1_INDEX] = 0;
         //bit_buffer[`C_EXT_DLC_INDEX:`C_EXT_DLC_INDEX+3] = can_package.data_length;
         bit_buffer[`C_EXT_DLC_INDEX +: `C_DLC_SIZE] = can_package.data_length;


         if(can_package.remote_frame == 0) begin
            for(int byte_counter = 0; byte_counter < can_package.data_length; byte_counter++) begin
               byte_start_index = `C_EXT_DATA_INDEX+byte_counter*8;
               //byte_end_index = byte_start_index+7;
               bit_buffer[byte_start_index +: 8] = can_package.data[byte_counter];
            end
            crc_start_index = `C_EXT_DATA_INDEX+(can_package.data_length*8);
         end else begin
            crc_start_index = `C_EXT_DATA_INDEX;
         end

         crc = calc_can_crc15(bit_buffer, crc_start_index);
         //bit_buffer[crc_start_index:crc_start_index+14] = crc;
         bit_buffer[crc_start_index +: `C_CRC_SIZE] = crc;

         bit_buffer[crc_start_index+`C_CRC_DELIM_INDEX] = 1;
         bit_buffer[crc_start_index+`C_ACK_SLOT_INDEX] = 1;
         bit_buffer[crc_start_index+`C_ACK_DELIM_INDEX] = 1;

         frame_end_index = crc_start_index+`C_EOF_INDEX+`C_EOF_SIZE+`C_IFS_SIZE-1;

         bit_buffer[crc_start_index+`C_EOF_INDEX +: `C_EOF_IFS_SIZE] = '1;
      end


      ////////////////////////////////////////////////////////////////////////////
      // Transmit bit buffer (CAN frame), and do bit stuffing
      ////////////////////////////////////////////////////////////////////////////
      @(sigs.cb);


      for(int bit_counter = 0; bit_counter <= frame_end_index; bit_counter++) begin

         sigs.can_tx <= bit_buffer[bit_counter];

         // Wait for sampling point before reading back CAN RX value
         for(int cycle_count = 0; cycle_count < m_sample_point_cycles; cycle_count++) begin
            @(sigs.cb);
         end

         // Check for ACK and arbitration lost
         if(bit_counter == crc_start_index+`C_ACK_SLOT_INDEX) begin
            if(sigs.can_rx == 0) begin
               $display("can_write: ACK received.");
               ack_received = 1;
            end else begin
               $display("can_write: ACK not received.");
            end
         end else if(sigs.can_rx != bit_buffer[bit_counter]) begin
            arb_lost = 1;
            $display("can_write: Arbitration lost, bit %0d", bit_counter);
            $display("can_write: tx value: %0d, rx value: %0d", sigs.can_tx, sigs.can_rx);
            return;
         end

         // Calculate number of consecutive bits of same value,
         // used for bit stuffing below.
         if(previous_bit_value == bit_buffer[bit_counter]) begin
            bit_stuffing_counter = bit_stuffing_counter+1;
         end else begin
            previous_bit_value   = bit_buffer[bit_counter];
            bit_stuffing_counter = 1;
         end

         for(int cycle_count = 0; cycle_count < m_phase2_cycles; cycle_count++) begin
            @(sigs.cb);
         end

         // Do bit stuffing if we sent 5 consecutive bits of same value
         // Bit stuffing should end after CRC code (before delimiter)
         // See page 45 here:
         // https://www.nxp.com/docs/en/reference-manual/BCANPSV2.pdf
         if(bit_stuffing_counter == 5 && bit_counter < crc_start_index+`C_CRC_DELIM_INDEX) begin
            //bit_stuffing_dbg = 1;
            sigs.can_tx <= !bit_buffer[bit_counter];
            previous_bit_value = !bit_buffer[bit_counter];
            bit_stuffing_counter = 1;

            for(int cycle_count = 0; cycle_count < m_sample_point_cycles; cycle_count++) begin
               @(sigs.cb);
            end

            // Check if arbitration was lost
            // Should probably not be possible for a stuff bit
            if(sigs.can_rx != previous_bit_value) begin
               arb_lost = 1;
               return;
            end

            for(int cycle_count = 0; cycle_count < m_phase2_cycles; cycle_count++) begin
               @(sigs.cb);
            end

         end
      end // bit_counter

      sigs.can_tx <= 1;
      @(sigs.cb);
   endtask


   task can_read(output can_package_t can_package,
                 output timeout,
                 output crc_error);

      int               bit_stuffing_counter = 0;
      int               bit_counter = 0;
      int               timeout_cycle_count = 0;
      int               data_length_bits;
      int               can_frame_bit_size;

      int               crc_start_index = 0;
      int               data_bits_index = 0;

      bit [0:200]       bit_buffer;
      bit [14:0]        crc_calc;
      bit [14:0]        crc_received;
      bit               previous_bit_value = 0;
      bit               can_frame_rx_done = 0;


      sigs.can_tx <= 1;

      crc_error        = 0;
      timeout          = 0;

      ////////////////////////////////////////////////////////////////////////////
      // Wait for activity on CAN RX signal
      ////////////////////////////////////////////////////////////////////////////
      while(sigs.can_rx == 1) begin
         @(sigs.cb);

         // Break out of procedure if timeout is reached
         if(m_timeout_cycles != 0) begin
            timeout_cycle_count = timeout_cycle_count + 1;
            if(timeout_cycle_count == m_timeout_cycles) begin
               timeout = 1;
               return;
            end
         end
      end

      ////////////////////////////////////////////////////////////////////////////
      // Receive CAN msg
      ////////////////////////////////////////////////////////////////////////////
      while(can_frame_rx_done == 0) begin
         // Wait for sampling point before sampling CAN RX
         for(int cycle_count = 0; cycle_count < m_sample_point_cycles; cycle_count++) begin
            @(sigs.cb);
         end

         // TODO: Check for bit stuffing errors...

         //         if(bit_stuff_en == 1) begin
         if(1) begin
            // After 5 consecutive bits of same value, a stuffing bit is sent
            if(bit_stuffing_counter == 5) begin
               // Discard stuff bits
               //bit_stuffing_dbg     = 1;
               bit_stuffing_counter = 1;

               assert(previous_bit_value != sigs.can_rx)
                 else $error("Error, stuff bit with wrong polarity received.");

               previous_bit_value   = sigs.can_rx;
            end else begin
               // Data bits
               bit_buffer[bit_counter] = sigs.can_rx;
               bit_counter             = bit_counter + 1;

               if(previous_bit_value != sigs.can_rx || bit_counter == 0) begin
                  // Reset bit stuffing counter when bit differs from previous bit
                  bit_stuffing_counter = 1;
                  previous_bit_value   = sigs.can_rx;
               end else begin
                  // Increase bit stuffing counter for consecutive bits of same value
                  bit_stuffing_counter = bit_stuffing_counter + 1;
               end
            end
         end

         // Check if we've received data length,
         // so we know how many bits we're receiving
         if(bit_counter > `C_STD_IDE_INDEX) begin
            if(bit_buffer[`C_STD_IDE_INDEX] == 0 && bit_counter == `C_STD_DLC_INDEX+`C_DLC_SIZE) begin
               can_package.data_length = bit_buffer[`C_STD_DLC_INDEX +: `C_DLC_SIZE];

               // Payload data is not sent for remote frames
               if(bit_buffer[`C_STD_RTR_INDEX] == 1) begin
                  data_length_bits = 0;
               end else begin
                  data_length_bits = 8 * bit_buffer[`C_STD_DLC_INDEX +: `C_DLC_SIZE];
               end

               can_frame_bit_size = data_length_bits + `C_STD_DATA_INDEX + `C_CRC_DELIM_INDEX + 1;

            end else if(bit_buffer[`C_EXT_IDE_INDEX] == 1 && bit_counter == `C_EXT_DLC_INDEX+`C_DLC_SIZE) begin
               can_package.data_length = bit_buffer[`C_EXT_DLC_INDEX +: `C_DLC_SIZE];

               // Payload data is not sent for remote frames
               if(bit_buffer[`C_EXT_RTR_INDEX] == 1) begin
                  data_length_bits = 0;
               end else begin
                  data_length_bits = 8 * bit_buffer[`C_EXT_DLC_INDEX +: `C_DLC_SIZE];
               end

               can_frame_bit_size = data_length_bits + `C_EXT_DATA_INDEX + `C_CRC_DELIM_INDEX + 1;
            end
         end // if (bit_counter > `C_STD_IDE_INDEX)

         // Wait for the remaining time (phase 2) of this bit
         for(int cycle_count = 0; cycle_count < m_phase2_cycles; cycle_count++) begin
            @(sigs.cb);
         end

         // End loop when last bit was received
         // can_frame_bit_size is set to zero until the DLC field has been received
         if(can_frame_bit_size != 0 && can_frame_bit_size == bit_counter) begin
            can_frame_rx_done = 1;
         end
      end // while (can_frame_rx_done = 0)


      ////////////////////////////////////////////////////////////////////////////
      // Verify CRC and CRC delimiter value
      ////////////////////////////////////////////////////////////////////////////
      crc_start_index = can_frame_bit_size - (`C_CRC_DELIM_INDEX+1);
      crc_calc = calc_can_crc15(bit_buffer, crc_start_index);
      crc_received = bit_buffer[crc_start_index +: `C_CRC_SIZE];

      if(crc_calc != crc_received) begin
         crc_error = 1;
         @(sigs.cb);
         crc_error = 0;
         @(sigs.cb);
         return;
      end

      assert(bit_buffer[crc_start_index + `C_CRC_DELIM_INDEX] == 1)
        else $error("Error: CRC delimiter was 0, expected 1");

      ////////////////////////////////////////////////////////////////////////////
      // Send ACK signal
      ////////////////////////////////////////////////////////////////////////////
      sigs.can_tx <= 0;
      #(m_bit_period);
      sigs.can_tx <= 1;

      ////////////////////////////////////////////////////////////////////////////
      // Wait for ACK delimiter, End Of Frame (EOF), and Interframe Spacing (IFS)
      ////////////////////////////////////////////////////////////////////////////
      for(int ack_delim_eof_ifs_bit_num = 0;
          ack_delim_eof_ifs_bit_num <= (1+`C_EOF_SIZE+`C_IFS_SIZE);
          ack_delim_eof_ifs_bit_num++) begin

         // Wait for sampling point before sampling CAN RX
         for(int cycle_count = 0; cycle_count < m_sample_point_cycles; cycle_count++) begin
            @(sigs.cb);
         end

         if(ack_delim_eof_ifs_bit_num == 0) begin
            assert(sigs.can_rx == 1) else $error("Error: ACK delimiter was 0, expected 1");
         end else if(ack_delim_eof_ifs_bit_num <= (1+`C_EOF_SIZE)) begin
            assert(sigs.can_rx == 1) else $error("Error: EOF bit was 0, expected 1");
         end else begin
            assert(sigs.can_rx == 1) else $error("Error: IFS bit was 0, expected 1");
         end

         // Wait for the remaining time (phase 2) of this bit
         //for cycle_count in 0 to m_phase2_cycles-1 loop
         for(int cycle_count = 0; cycle_count < m_phase2_cycles; cycle_count++) begin
            @(sigs.cb);
         end
      end

      // /////////////////////////////////////////////////////////////////////////
      // Parse CAN message
      // /////////////////////////////////////////////////////////////////////////
      if(bit_buffer[`C_STD_IDE_INDEX] == 0) begin
         // Standard frame
         $display("can_read: Standard frame received");
         can_package.arb_id[10:0]  = bit_buffer[`C_STD_ARB_ID_INDEX +: `C_STD_ARB_ID_SIZE];
         can_package.arb_id[28:11] = '0;
         can_package.remote_frame  = bit_buffer[`C_STD_RTR_INDEX];
         can_package.extended_id   = 0;
         data_bits_index           = `C_STD_DATA_INDEX;
      end else begin
         // Extended frame
         $display("can_read: Extended frame received");
         can_package.arb_id[10:0]  = bit_buffer[`C_EXT_ARB_ID_A_INDEX +: `C_EXT_ARB_ID_A_SIZE];
         can_package.arb_id[28:11] = bit_buffer[`C_EXT_ARB_ID_B_INDEX +: `C_EXT_ARB_ID_B_SIZE];
         can_package.remote_frame  = bit_buffer[`C_EXT_RTR_INDEX];
         can_package.extended_id   = 1;
         data_bits_index           = `C_EXT_DATA_INDEX;
      end // else: !if(bit_buffer[`C_STD_IDE_INDEX] == 0)

      // Copy data from bit buffer
      // No copying takes place for remote frame requests,
      // because data_length_bits is set to zero in that case
      for(int bit_num = 0; bit_num < 64; bit_num++) begin
         // Bits are sent MSB first
         if(bit_num < data_length_bits) begin
            can_package.data[bit_num/8][7 - (bit_num % 8)] = bit_buffer[data_bits_index+bit_num];
         end else begin
            can_package.data[bit_num/8][7 - (bit_num % 8)] = 0;
         end
      end

      @(sigs.cb);
   endtask


   function bit[14:0] calc_can_crc15(bit[0:200] data, int data_len);
      bit[14:0] crc_shift_reg = 0;
      bit       crc_next;
      const bit [15:0] crc_poly =  16'h4599;

      for(int bit_count = 0; bit_count < data_len; bit_count++) begin
         crc_next = data[bit_count] ^ crc_shift_reg[14];

         crc_shift_reg[14:1] = crc_shift_reg[13:0];
         crc_shift_reg[0] = 0;

         if(crc_next == 1) begin
            crc_shift_reg[14:0] = crc_shift_reg[14:0] ^ crc_poly[14:0];
         end
      end

      return crc_shift_reg;
   endfunction


   // Constructor
   function new (virtual  canbus_if _sigs,
                 int      bit_rate,
                 realtime clock_period,
                 realtime timeout,
                 int      sync_quanta,
                 int      prop_quanta,
                 int      phase1_quanta,
                 int      phase2_quanta);
      this.sigs = _sigs;
      this.m_bit_rate = bit_rate;
      this.m_clock_period = clock_period;
      this.m_timeout = timeout;
      this.m_sync_quanta = sync_quanta;
      this.m_prop_quanta = prop_quanta;
      this.m_phase1_quanta = phase1_quanta;
      this.m_phase2_quanta = phase2_quanta;

      m_bit_period = 1s / m_bit_rate;
      m_bit_quanta = m_bit_period / 10;
      m_bit_quanta_cycles = m_bit_quanta / m_clock_period;
      m_sync_cycles = m_sync_quanta * m_bit_quanta_cycles;
      m_prop_cycles = m_prop_quanta * m_bit_quanta_cycles;
      m_phase1_cycles = m_phase1_quanta * m_bit_quanta_cycles;
      m_phase2_cycles = m_phase2_quanta * m_bit_quanta_cycles;
      m_sample_point_cycles = m_sync_cycles+m_prop_cycles+m_phase1_cycles;
      m_timeout_cycles = m_timeout / m_clock_period;

      assert(sync_quanta+prop_quanta+phase1_quanta+phase2_quanta == 10)
        else $error("Error, illegal bit timing setup. Quantas do not add up to 10.");
   endfunction // new
endclass


`endif // CAN_BFM_INCLUDED_V
