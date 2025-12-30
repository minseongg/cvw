///////////////////////////////////////////
// uart.sv
//
// Written: David_Harris@hmc.edu 21 January 2021
// Modified:
//
// Purpose: Universal Asynchronous Receiver/ Transmitter with FIFOs
//          Emulates interface of Texas Instruments PC16550D
//          https://media.digikey.com/pdf/Data%20Sheets/Texas%20Instruments%20PDFs/PC16550D.pdf
//          Compatible with UART in Imperas Virtio model
//
//  Compatible with most of PC16550D with the following known exceptions:
//   Generates 2 rather than 1.5 stop bits when 5-bit word length is selected and LCR[2] = 1
//   Timeout not yet implemented
//
// Documentation: RISC-V System on Chip Design
//
// A component of the CORE-V-WALLY configurable RISC-V project.
// https://github.com/openhwgroup/cvw
//
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file
// except in compliance with the License, or, at your option, the Apache License version 2.0. You
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
// either express or implied. See the License for the specific language governing permissions
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////

/* verilator lint_off UNOPTFLAT */

module uartPC16550D #(parameter UART_PRESCALE) (
  // Processor Interface
  input  logic       PCLK, PRESETn,                  // UART clock and active low reset
  input  logic [2:0] A,                              // address input (8 registers)
  input  logic [7:0] Din,                            // 8-bit WriteData
  output logic [7:0] Dout,                           // 8-bit ReadData
  input  logic       MEMRb, MEMWb,                   // Active low memory read/write
  output logic       INTR, TXRDYb, RXRDYb,           // interrupt and ready lines
  // E1A Driver
  input  logic       SIN, DSRb, DCDb, CTSb, RIb,     // UART external serial and flow-control inputs
  output logic       SOUT, RTSb, DTRb, OUT1b, OUT2b  // UART external serial and flow-control outputs
);

  // register map
  localparam UART_DLL_RBR = 3'b000;
  localparam UART_DLM_IER = 3'b001;
  localparam UART_IIR = 3'b010;
  localparam UART_LCR = 3'b011;
  localparam UART_MCR = 3'b100;
  localparam UART_LSR = 3'b101;
  localparam UART_MSR = 3'b110;
  localparam UART_SCR = 3'b111;

  // transmit and receive states
  typedef enum logic [1:0] {UART_IDLE, UART_ACTIVE, UART_DONE, UART_BREAK} statetype;

  // Registers
  logic [7:0]  FCR, LCR, LSR, SCR, DLL, DLM;
  logic [3:0]  IER, MSR;
  logic [4:0]  MCR;

  // Control signals
  logic        DLAB; // Divisor Latch Access Bit (LCR bit 7)

  // Baud and rx/tx timing
  statetype txstate;

  // transmit data
  logic                         txhrfull, txsrfull;

  // interrupts
  logic                         THRE, THRE_IP, squashTHRE_IP, prevSquashTHRE_IP, setSquashTHRE_IP, resetSquashTHRE_IP;
  logic                         intrpending;
  logic [2:0]                   intrID;

  ///////////////////////////////////////////
  // Register interface (Table 1, note some are read only and some write only)
  ///////////////////////////////////////////

  always_ff @(posedge PCLK)
    if (~PRESETn) begin // Table 3 Reset Configuration
      IER <= 4'b0;
      FCR <= 8'b0;
      LCR <= 8'b11; // PC16550D datasheet resets to 0, but all modern systems will use 8-bit data.  Wally resets to 3 for 8-bit data.
      MCR <= 5'b0;
      LSR <= 8'b0110_0000;
      MSR <= 4'b0;
      DLL <= 8'd1; // this cannot be zero with DLM also zero.
      DLM <= 8'b0;
      SCR <= 8'b0; // not strictly necessary to reset
    end else begin
      if (~MEMWb) begin
        /* verilator lint_off CASEINCOMPLETE */
        case (A)
          UART_DLL_RBR: if (DLAB) DLL <= Din; // TX handled in TX register/FIFO section
          UART_DLM_IER: if (DLAB) DLM <= Din; else IER <= Din[3:0];
          UART_IIR: FCR <= {Din[7:6], 2'b0, Din[3], 2'b0, Din[0]}; // Write only FIFO Control Register; 4:5 reserved and 2:1 self-clearing
          UART_LCR: LCR <= Din;
          UART_MCR: MCR <= Din[4:0];
          UART_SCR: SCR <= Din;
        endcase
        /* verilator lint_on CASEINCOMPLETE */
      end

      // Line Status Register (8.6.3)
      // Ben 6/9/21 I don't like how this is a register. A lot of the individual bits have clocked components, so this just adds unnecessary delay.
      if (~MEMWb & (A == UART_LSR))
        LSR[6:1] <= Din[6:1]; // recommended only for test, see 8.6.3
      else begin
        LSR[0]   <= 1'b0; // Data ready
        LSR[5]   <= THRE; // THRE
        LSR[6]   <= ~txsrfull & THRE; //  TEMT
      end

      // Modem Status Register (8.6.8)
      if (~MEMWb & (A == UART_MSR))
        MSR <= Din[3:0];
      else if (~MEMRb & (A == UART_MSR))
        MSR <= 4'b0; // Reading MSR clears the flags in MSR bits 3:0
    end

  always_comb
    if (~MEMRb)
      case (A)
        UART_DLL_RBR: if (DLAB) Dout = DLL; else Dout = 8'b0;
        UART_DLM_IER: if (DLAB) Dout = DLM; else Dout = {4'b0, IER[3:0]};
        UART_IIR: Dout = {4'b0000, intrID[2:0], ~intrpending}; // Read only Interrupt Ident Register
        UART_LCR: Dout = LCR;
        UART_MCR: Dout = {3'b000, MCR};
        UART_LSR: Dout = LSR;
        UART_MSR: Dout = {4'b0001, MSR[3:0]};
        UART_SCR: Dout = SCR;
      endcase
    else Dout = 8'b0;

  ///////////////////////////////////////////
  // transmit timing and control
  ///////////////////////////////////////////

  always_ff @(posedge PCLK)
    if (~PRESETn) begin
      txstate <= UART_IDLE;
    end else if ((txstate == UART_IDLE) & txsrfull) begin // start transmitting
      txstate <= UART_ACTIVE;
    end else if (txstate == UART_ACTIVE) begin
      txstate <= UART_DONE;
    end else if (txstate == UART_DONE) begin
      txstate <= UART_IDLE;
    end

  ///////////////////////////////////////////
  // transmit holding register, shift register, FIFO
  ///////////////////////////////////////////

  // registers & FIFO
  always_ff @(posedge PCLK)
    if (~PRESETn) begin
      txhrfull <= 1'b0; txsrfull <= 1'b0;
    end else begin
      if (~MEMWb & A == 3'b000 & ~DLAB) begin // writing transmit holding register or fifo
        txhrfull <= 1'b1;
        $write("%c",Din); // for testbench
      end
      if (txstate == UART_IDLE) begin // move data into tx shift register if available
        if (txhrfull) begin
          txhrfull <= 1'b0;
          txsrfull <= 1'b1;
        end
      end else if (txstate == UART_DONE) txsrfull <= 1'b0; // done transmitting shift register
    end

  ///////////////////////////////////////////
  // interrupts
  ///////////////////////////////////////////

  assign THRE = ~txhrfull;
  assign THRE_IP = THRE & ~squashTHRE_IP; // THRE_IP squashed upon reading IIR

  // IIR: interrupt priority (Table 5)
  // set intrID based on highest priority pending interrupt source; otherwise, no interrupt is pending
  always_comb begin
    intrpending = 1'b1;
    if (THRE_IP & IER[1])                     intrID = 3'b001;
    else begin
      intrID = 3'b000;
      intrpending = 1'b0;
    end
  end
  always_ff @(posedge PCLK) INTR <= intrpending; // prevent glitches on interrupt pin

  // Side effect of reading IIR is lowering THRE_IP if most significant intr
  assign setSquashTHRE_IP = ~MEMRb & (A==3'b010) & (intrID==3'h1); // there's a 1-cycle delay on set squash so that THRE_IP doesn't change during the process of reading IIR (otherwise combinational loop)
  assign resetSquashTHRE_IP = ~THRE;
  assign squashTHRE_IP = prevSquashTHRE_IP & ~resetSquashTHRE_IP;
  flopr #(1) squashTHRE_IPreg(PCLK, ~PRESETn, squashTHRE_IP | setSquashTHRE_IP, prevSquashTHRE_IP);

  ///////////////////////////////////////////
  // modem control logic
  ///////////////////////////////////////////

  assign DLAB = LCR[7];

endmodule

/* verilator lint_on UNOPTFLAT */
