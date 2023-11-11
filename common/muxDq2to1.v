// SPDX-License-Identifier: GPL-3.0-only
/*
 * SDRAM inout mux for the SDRAM controller
 *
 * Copyright (C) 2023 Luís Mendes <luis.p.mendes@gmail.com>
 */

module muxDq2to1(input clk, input wrn, inout [31:0] dq, input [31:0] dqIn, output [31:0] dqOut );

  assign dq = wrn == 1 ? dqIn : 32'bzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz;

  assign dqOut = dq;
      
endmodule
