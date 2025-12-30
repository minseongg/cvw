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
  logic                         baudpulse, txbaudpulse; // high one system clk cycle each baud/16 period
  logic [16+UART_PRESCALE-1:0]  baudcount;
  logic [3:0]                   txoversampledcnt;  // count oversampled-by-16
  logic [3:0]                   txbitssent;
  statetype txstate;

  // shift registers and FIFOs
  logic [3:0]                   txbitsexpected;

  // transmit data
  logic [7:0]                   TXHR, nexttxdata;
  logic [11:0]                  txdata, txsr;
  logic                         txnextbit, txhrfull, txsrfull;
  logic                         txparity;

  // control signals
  logic                         evenparitysel;

  // interrupts
  logic                         THRE, THRE_IP, squashTHRE_IP, prevSquashTHRE_IP, setSquashTHRE_IP, resetSquashTHRE_IP;
  logic                         intrpending;
  logic [2:0]                   intrID;

  logic                         baudpulseComb;

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
          UART_DLL_RBR: if (DLAB) DLL <= Din; // else TXHR <= Din; // TX handled in TX register/FIFO section
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
  // Baud rate generator
  // consider switching to same fixed-frequency reference clock used for TIME register
  // prescale by factor of 2^UART_PRESCALE to allow for high-frequency reference clock
  // Unlike PC16550D, this unit is hardwired with same rx and tx baud clock
  // For example, with PCLK = 320 MHz, UART_PRESCALE = 5, DLM = 0, DLL = 65,
  // 320 MHz system clock is divided by 65 x 2^5.  The UART clock 16x oversamples
  // the data, so the baud rate is 320x10^6 / (65 x 2^5 x 16) = 9615 Hz, which is
  // close enough to 9600 baud to stay synchronized over the duration of one character.
  ///////////////////////////////////////////
  always_ff @(posedge PCLK)
    if (~PRESETn) begin
      baudcount <= 1;
      baudpulse <= 1'b0;
    end else if (~MEMWb & DLAB & (A == 3'b0 | A == 3'b1)) begin
      baudcount <= 1;
    end else begin
      // the baudpulse is too long by 2 clock cycles.
      // This is cause baudpulse is registered adding 1 cycle and
      // baudcount is reset when baudcount equals the threshold {DLM, DLL, UART_PRESCALE}
      // rather than 1 less than that value.  Alternatively the reset value could be 1 rather
      // than 0.
      baudpulse <= baudpulseComb;
      baudcount <= baudpulseComb ? 1 :  baudcount +1;
    end

  assign baudpulseComb = (baudcount == {DLM, DLL, {(UART_PRESCALE){1'b0}}});

  assign txbaudpulse = baudpulse;

  ///////////////////////////////////////////
  // transmit timing and control
  ///////////////////////////////////////////

  always_ff @(posedge PCLK)
    if (~PRESETn) begin
      txoversampledcnt <= '0;
      txstate          <= UART_IDLE;
      txbitssent       <= '0;
    end else if ((txstate == UART_IDLE) & txsrfull) begin // start transmitting
      txstate          <= UART_ACTIVE;
      txoversampledcnt <= 4'b1;
      txbitssent       <= '0;
    end else if (txbaudpulse & (txstate == UART_ACTIVE)) begin
      txoversampledcnt <= txoversampledcnt + 1'b1;
      if (txnextbit) begin // transmit at end of phase
        txbitssent <= txbitssent + 1'b1;
        if (txbitssent == txbitsexpected) txstate <= UART_DONE;
      end
    end else if (txstate == UART_DONE) begin
      txstate <= UART_IDLE;
    end

  assign txbitsexpected = 4'd1 + (4'd5 + {2'b00, LCR[1:0]}) + {3'b000, LCR[3]} + 4'd1 + {3'b000, LCR[2]} - 4'd1; // start bit + data bits + (parity bit) + stop bit(s) - 1
  assign txnextbit = txbaudpulse & (txoversampledcnt == 4'b0000);  // implies txstate = UART_ACTIVE

  ///////////////////////////////////////////
  // transmit holding register, shift register, FIFO
  ///////////////////////////////////////////

  always_comb begin // compute value for parity and tx holding register
    nexttxdata = TXHR; // pick from FIFO or holding register
    case (LCR[1:0]) // compute parity from appropriate number of bits
      2'b00: txparity = ^nexttxdata[4:0] ^ ~evenparitysel;
      2'b01: txparity = ^nexttxdata[5:0] ^ ~evenparitysel;
      2'b10: txparity = ^nexttxdata[6:0] ^ ~evenparitysel;
      2'b11: txparity = ^nexttxdata[7:0] ^ ~evenparitysel;
    endcase
    case({LCR[3], LCR[1:0]}) // parity, data bits
      // load up start bit (0), 5-8 data bits, 0-1 parity bits, 2 stop bits (only one sometimes used), padding
      3'b000: txdata = {1'b0, nexttxdata[0], nexttxdata[1], nexttxdata[2], nexttxdata[3], nexttxdata[4], 6'b111111};                                                       // 5 data, no parity
      3'b001: txdata = {1'b0, nexttxdata[0], nexttxdata[1], nexttxdata[2], nexttxdata[3], nexttxdata[4], nexttxdata[5], 5'b11111};                                         // 6 data, no parity
      3'b010: txdata = {1'b0, nexttxdata[0], nexttxdata[1], nexttxdata[2], nexttxdata[3], nexttxdata[4], nexttxdata[5], nexttxdata[6], 4'b1111};                           // 7 data, no parity
      3'b011: txdata = {1'b0, nexttxdata[0], nexttxdata[1], nexttxdata[2], nexttxdata[3], nexttxdata[4], nexttxdata[5], nexttxdata[6], nexttxdata[7], 3'b111};             // 8 data, no parity
      3'b100: txdata = {1'b0, nexttxdata[0], nexttxdata[1], nexttxdata[2], nexttxdata[3], nexttxdata[4], txparity, 5'b11111};                                              // 5 data, parity
      3'b101: txdata = {1'b0, nexttxdata[0], nexttxdata[1], nexttxdata[2], nexttxdata[3], nexttxdata[4], nexttxdata[5], txparity, 4'b1111};                                // 6 data, parity
      3'b110: txdata = {1'b0, nexttxdata[0], nexttxdata[1], nexttxdata[2], nexttxdata[3], nexttxdata[4], nexttxdata[5], nexttxdata[6], txparity, 3'b111};                  // 7 data, parity
      3'b111: txdata = {1'b0, nexttxdata[0], nexttxdata[1], nexttxdata[2], nexttxdata[3], nexttxdata[4], nexttxdata[5], nexttxdata[6], nexttxdata[7], txparity, 2'b11};    // 8 data, parity
    endcase
  end

  // registers & FIFO
  always_ff @(posedge PCLK)
    if (~PRESETn) begin
      txhrfull <= 1'b0; txsrfull <= 1'b0; TXHR <= '0; txsr <= 12'hfff;
    end else begin
      if (~MEMWb & A == 3'b000 & ~DLAB) begin // writing transmit holding register or fifo
        TXHR     <= Din;
        txhrfull <= 1'b1;
        $write("%c",Din); // for testbench
      end
      if (txstate == UART_IDLE) begin // move data into tx shift register if available
        if (txhrfull) begin
          txsr     <= txdata;
          txhrfull <= 1'b0;
          txsrfull <= 1'b1;
        end
      end else if (txstate == UART_DONE) txsrfull <= 1'b0; // done transmitting shift register
      else if (txstate == UART_ACTIVE & txnextbit) txsr <= {txsr[10:0], 1'b1}; // shift txhr
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
  assign evenparitysel  = LCR[4];

endmodule

/* verilator lint_on UNOPTFLAT */
