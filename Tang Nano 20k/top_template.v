module top(..., 
           O_sdram_clk, O_sdram_cke, O_sdram_cas_n, O_sdram_ras_n, O_sdram_cs_n, O_sdram_wen_n,
           O_sdram_ba, O_sdram_addr, IO_sdram_dq, O_sdram_dqm);

  /* For the GW2AR-LV18QN88C8/I7 the SDRAM clock must be the same as the FPGA SDRAM controller */   
  output O_sdram_clk;
  output O_sdram_cke;
  output O_sdram_cas_n;           // columns address select
  output O_sdram_ras_n;           // row address select
  output O_sdram_cs_n;            // chip select
  output O_sdram_wen_n;           // write enable
  output  [1:0] O_sdram_ba;       // four banks
  output [10:0] O_sdram_addr;     // 11 bit multiplexed address bus
  inout  [31:0] IO_sdram_dq;      // 32 bit bidirectional data bus
  output  [3:0] O_sdram_dqm;      // 32/4

wire clkSDRAM;

wire [20:0] startReadAddress;
wire [20:0] startWriteAddress;
wire [23:0] wordRead;
wire [23:0] wordToWrite;
wire incRdADdress;
wire incWrAddress;
wire startWrite;
wire startRead;
wire ctrlReady;

assign O_sdram_clk = clkSDRAM;

sdramController mySDRAMController (
    .sdramClk(clkSDRAM),
    .sdramClkEn(O_sdram_cke),    
    .sdramRASn(O_sdram_ras_n),    
    .sdramCASn(O_sdram_cas_n),    
    .sdramWEn(O_sdram_wen_n),    
    .sdramCSn(O_sdram_cs_n),     
    .sdramAddress(O_sdram_addr),     
    .sdramBank(O_sdram_ba),     
    .sdramDqIn(sdramDqIn),     
    .sdramDqOut(sdramDqOut),     
    .sdramDqWRn(sdramDqWRn),     
    .sdramDataMasks(O_sdram_dqm),     
    .ctrlReady(ctrlReady), //Controller is ready to operate and SDRAM has been initialized
    .ctrlRd(startRead), //Input: Signal for 1 clock cycle to start a Page Read, when ctrlReady is high
    .ctrlWr(startWrite), //Input: Signal for 1 clock cycle to start a Page Write, when ctrl Read is high
    .ctrlRdAddress(startReadAddress), //Input: Start Read address should point to the start of the page to be read
    .ctrlWrAddress(startWriteAddress), //Input: Start Write address should point to the start of the page to be read
    .ctrlRdIncAddress(incRdAddress), //Output: Controller indicates to increment the read page address, since it will output the next word in the page
    .ctrlRdDataOut(wordRead), //Output: Word read
    .ctrlWrIncAddress(incWrAddress), //Output: Controller indicates to increment the write page address and provide the next word in ctrlWrDataIn
    .ctrlWrDataIn(wordToWrite) //Input: Word to write
);

muxDq2to1 myDqMux (
    .clk(clkSDRAM),    
    .wrn(sdramDqWRn), 
    .dq(IO_sdram_dq),    
    .dqIn(sdramDqIn),    
    .dqOut(sdramDqOut)
);


endmodule
