//-----------------------------------------------------------------------------
// Title      : CAN bus interface
// Project    : 
//-----------------------------------------------------------------------------
// File       : canbus_if.sv
// Author     : Simon Voigt Nesbø  <svn@hvl.no>
// Company    : Western Norway University of Applied Sciences
// Created    : 2018-10-15
// Last update: 2018-10-15
// Platform   : Simulation
// Standard   : System Verilog
//-----------------------------------------------------------------------------
// Description: CAN bus inteface.
//-----------------------------------------------------------------------------
// Copyright (c)   2018
//-----------------------------------------------------------------------------
// Revisions  :
// Date        Version  Author        Description
// 2018-10-15  1.0      simon	      Created
//-----------------------------------------------------------------------------
`ifndef CANBUS_IF_INCLUDED_V
 `define CANBUS_IF_INCLUDED_V

interface canbus_if(input logic can_clk);
   logic               can_tx;
   logic               can_rx;

   clocking cb @(posedge can_clk);
   endclocking // cb

   // Modport for drivers of CAN signals (FPGA, BFM)
   modport driver(input can_rx,
                  output can_tx);

   // Modport used with can_connection module, to connect
   // several interfaces to drive the common bus line to simulate pull on CAN line
   modport bus_conn(output can_rx,
                    input can_tx);

endinterface // canbus_if

// Connects can bus signals to simulate the pull up on the CAN bus line
module can_connection #(int NUM_IFS=2) (output logic[NUM_IFS-1:0] can_rx, input logic[NUM_IFS-1:0] can_tx);
   generate
      genvar i;
      for(i = 0; i < NUM_IFS; i++)
        begin
           //always_comb
             //begin
           assign can_rx[i] = &can_tx;
             //end
        end
   endgenerate
endmodule

/* -----\/----- EXCLUDED -----\/-----
package can;

 `include "can_bfm.sv"
 `include "can_comm.sv"

endpackage // can
 -----/\----- EXCLUDED -----/\----- */

`endif // CANBUS_IF_INCLUDED_V
