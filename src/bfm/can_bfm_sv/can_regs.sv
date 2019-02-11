`ifndef CAN_REGS_INCLUDED_V
 `define CAN_REGS_INCLUDED_V

`define C_CAN_CMR  1   // Command register
`define C_CAN_SR   2   // Status register
`define C_CAN_IR   3   // Interrupt Register
`define C_CAN_BTR0 6   // Bus timing 0 register
`define C_CAN_BTR1 7   // Bus timing 1 register
`define C_CAN_CDR  31  // Clock divider register


//////////////////////////////////////////////////////////////////////////////
// BASIC Mode register map
//////////////////////////////////////////////////////////////////////////////
`define C_CAN_BM_CR        0   // Control register
`define C_CAN_BM_ACR       4   // Acceptance code register
`define C_CAN_BM_AMR       5   // Acceptance mask register
`define C_CAN_BM_TXB_ID1   10  // TX buffer - ID byte 1 (10:3)
`define C_CAN_BM_TXB_ID2   11  // TX buffer - ID byte 2 (2:0)
`define C_CAN_BM_TXB_DATA1 12  // TX buffer - ID data 1
`define C_CAN_BM_TXB_DATA2 13  // TX buffer - ID data 2
`define C_CAN_BM_TXB_DATA3 14  // TX buffer - ID data 3
`define C_CAN_BM_TXB_DATA4 15  // TX buffer - ID data 4
`define C_CAN_BM_TXB_DATA5 16  // TX buffer - ID data 5
`define C_CAN_BM_TXB_DATA6 17  // TX buffer - ID data 6
`define C_CAN_BM_TXB_DATA7 18  // TX buffer - ID data 7
`define C_CAN_BM_TXB_DATA8 19  // TX buffer - ID data 8
`define C_CAN_BM_RXB_ID1   20  // RX buffer - ID byte 1 (10:3)
`define C_CAN_BM_RXB_ID2   21  // RX buffer - ID byte 2 (2:0)
`define C_CAN_BM_RXB_DATA1 22  // RX buffer - ID data 1
`define C_CAN_BM_RXB_DATA2 23  // RX buffer - ID data 2
`define C_CAN_BM_RXB_DATA3 24  // RX buffer - ID data 3
`define C_CAN_BM_RXB_DATA4 25  // RX buffer - ID data 4
`define C_CAN_BM_RXB_DATA5 26  // RX buffer - ID data 5
`define C_CAN_BM_RXB_DATA6 27  // RX buffer - ID data 6
`define C_CAN_BM_RXB_DATA7 28  // RX buffer - ID data 7
`define C_CAN_BM_RXB_DATA8 29  // RX buffer - ID data 8


//////////////////////////////////////////////////////////////////////////////
// EXTENDED Mode register map
//////////////////////////////////////////////////////////////////////////////
`define C_CAN_EM_MOD   0   // Mode register
`define C_CAN_EM_IER   4   // Interrupt enable register
`define C_CAN_EM_ALC   11  // Arbitration lost register
`define C_CAN_EM_ECC   12  // Error code capture register
`define C_CAN_EM_EWLR  13  // Error warning limit register
`define C_CAN_EM_RXERR 14  // RX error counter register
`define C_CAN_EM_TXERR 15  // TX error counter register

// TX frame information (in operation mode only)
// SFF in standard mode, EFF in extended mode
`define C_CAN_EM_SFF       16  // TX frame information register
`define C_CAN_EM_EFF       16  // TX frame information register


// TX/RX data buffer (in operation mode only)
// Write operation writes to TX buffer, read operation reads from RX buffer
// SFF registers are valid when using standard frame, EFF registers are
// valid when using extended frame
`define C_CAN_EM_SFF_TXB_ID1   17  // TX buffer - ID byte 1 (10:3)
`define C_CAN_EM_SFF_TXB_ID2   18  // TX buffer - ID byte 2 (2:0)
`define C_CAN_EM_SFF_TXB_DATA1 19  // TX buffer - ID data 1
`define C_CAN_EM_SFF_TXB_DATA2 20  // TX buffer - ID data 2
`define C_CAN_EM_SFF_TXB_DATA3 21  // TX buffer - ID data 3
`define C_CAN_EM_SFF_TXB_DATA4 22  // TX buffer - ID data 4
`define C_CAN_EM_SFF_TXB_DATA5 23  // TX buffer - ID data 5
`define C_CAN_EM_SFF_TXB_DATA6 24  // TX buffer - ID data 6
`define C_CAN_EM_SFF_TXB_DATA7 25  // TX buffer - ID data 7
`define C_CAN_EM_SFF_TXB_DATA8 26  // TX buffer - ID data 8

`define C_CAN_EM_SFF_RXB_ID1   17  // RX buffer - ID byte 1 (10:3)
`define C_CAN_EM_SFF_RXB_ID2   18  // RX buffer - ID byte 2 (2:0)
`define C_CAN_EM_SFF_RXB_DATA1 19  // RX buffer - ID data 1
`define C_CAN_EM_SFF_RXB_DATA2 20  // RX buffer - ID data 2
`define C_CAN_EM_SFF_RXB_DATA3 21  // RX buffer - ID data 3
`define C_CAN_EM_SFF_RXB_DATA4 22  // RX buffer - ID data 4
`define C_CAN_EM_SFF_RXB_DATA5 23  // RX buffer - ID data 5
`define C_CAN_EM_SFF_RXB_DATA6 24  // RX buffer - ID data 6
`define C_CAN_EM_SFF_RXB_DATA7 25  // RX buffer - ID data 7
`define C_CAN_EM_SFF_RXB_DATA8 26  // RX buffer - ID data 8

`define C_CAN_EM_EFF_TXB_ID1   17  // TX buffer - ID byte 1 (28:21)
`define C_CAN_EM_EFF_TXB_ID2   18  // TX buffer - ID byte 2 (20:13)
`define C_CAN_EM_EFF_TXB_ID3   19  // TX buffer - ID byte 1 (12:5)
`define C_CAN_EM_EFF_TXB_ID4   20  // TX buffer - ID byte 2 (4:0)
`define C_CAN_EM_EFF_TXB_DATA1 21  // TX buffer - ID data 1
`define C_CAN_EM_EFF_TXB_DATA2 22  // TX buffer - ID data 2
`define C_CAN_EM_EFF_TXB_DATA3 23  // TX buffer - ID data 3
`define C_CAN_EM_EFF_TXB_DATA4 24  // TX buffer - ID data 4
`define C_CAN_EM_EFF_TXB_DATA5 25  // TX buffer - ID data 5
`define C_CAN_EM_EFF_TXB_DATA6 26  // TX buffer - ID data 6
`define C_CAN_EM_EFF_TXB_DATA7 27  // TX buffer - ID data 7
`define C_CAN_EM_EFF_TXB_DATA8 28  // TX buffer - ID data 8

`define C_CAN_EM_EFF_RXB_ID1   17  // RX buffer - ID byte 1 (28:21)
`define C_CAN_EM_EFF_RXB_ID2   18  // RX buffer - ID byte 2 (20:13)
`define C_CAN_EM_EFF_RXB_ID3   19  // RX buffer - ID byte 1 (12:5)
`define C_CAN_EM_EFF_RXB_ID4   20  // RX buffer - ID byte 2 (4:0)
`define C_CAN_EM_EFF_RXB_DATA1 21  // RX buffer - ID data 1
`define C_CAN_EM_EFF_RXB_DATA2 22  // RX buffer - ID data 2
`define C_CAN_EM_EFF_RXB_DATA3 23  // RX buffer - ID data 3
`define C_CAN_EM_EFF_RXB_DATA4 24  // RX buffer - ID data 4
`define C_CAN_EM_EFF_RXB_DATA5 25  // RX buffer - ID data 5
`define C_CAN_EM_EFF_RXB_DATA6 26  // RX buffer - ID data 6
`define C_CAN_EM_EFF_RXB_DATA7 27  // RX buffer - ID data 7
`define C_CAN_EM_EFF_RXB_DATA8 28  // RX buffer - ID data 8

// Acceptance code/mask registers (available in reset mode only)
`define C_CAN_EM_ACR0      16  // Acceptance Code 0
`define C_CAN_EM_ACR1      17  // Acceptance Code 1
`define C_CAN_EM_ACR2      18  // Acceptance Code 2
`define C_CAN_EM_ACR3      19  // Acceptance Code 3
`define C_CAN_EM_AMR0      20  // Acceptance Mask 0
`define C_CAN_EM_AMR1      21  // Acceptance Mask 1
`define C_CAN_EM_AMR2      22  // Acceptance Mask 2
`define C_CAN_EM_AMR3      23  // Acceptance Mask 3


// Rx message count and buffer start address
// (available in operation and
`define C_CAN_EM_RMC       29  // RX Message Counter
`define C_CAN_EM_RBSA      30  // RX Buffer Start Address


//////////////////////////////////////////////////////////////////////////////
//Interrupt Register (IR) bit definitions
//////////////////////////////////////////////////////////////////////////////
`define C_CAN_IR_RI_IRQ_BIT  0  // Receive Interrupt
`define C_CAN_IR_TI_IRQ_BIT  1  // Transmit Interrupt
`define C_CAN_IR_EI_IRQ_BIT  2  // Error warning Interrupt
`define C_CAN_IR_DOI_IRQ_BIT 3  // Data Overrun Interrupt
`define C_CAN_IR_WUI_IRQ_BIT 4  // Wake-Up Interrupt
`define C_CAN_IR_EPI_IRQ_BIT 5  // Error Passive Interrupt
`define C_CAN_IR_ALI_IRQ_BIT 6  // Arbitration Lost Interrupt
`define C_CAN_IR_BEI_IRQ_BIT 7  // Bus Error Interrupt


`endif // CAN_REGS_INCLUDED_V
