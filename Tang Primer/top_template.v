//Template for a top interface of the SDRAM controller

wire [10:0] sdramAddress;
wire [1:0] sdramBank;
wire sdramCASn;
wire sdramCSn;
wire sdramClk;
wire sdramClkEn;
wire [3:0] sdramDataMasks;
wire [31:0] sdramDqIn;
wire [31:0] sdramDqOut;
wire sdramDqWRn;
wire sdramRASn;
wire sdramWEn;

wire [20:0] startReadAddress;
wire [20:0] startWriteAddress;
wire [23:0] wordRead;
wire [23:0] wordToWrite;
wire incRdADdress;
wire incWrAddress;
wire startWrite;
wire startRead;
wire ctrlReady;

sdramController mySDRAMController (
    .sdramClk(clkSDRAM),
    .sdramClkEn(sdramClkEn),    
    .sdramRASn(sdramRASn),    
    .sdramCASn(sdramCASn),    
    .sdramWEn(sdramWEn),    
    .sdramCSn(sdramCSn),     
    .sdramAddress(sdramAddress),     
    .sdramBank(sdramBank),     
    .sdramDqIn(sdramDqIn),     
    .sdramDqOut(sdramDqOut),     
    .sdramDqWRn(sdramDqWRn),     
    .sdramDataMasks(sdramDataMasks),     
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
    .dq(sdramDq),    
    .dqIn(sdramDqIn),    
    .dqOut(sdramDqOut)
);
 
 /* For the EG4S20BG256 the SDRAM clock must be the same as the FPGA SDRAM controller */   
sdram mySdram (  
    .clk(clkSDRAM), 
    .ras_n(sdramRASn),    
    .cas_n(sdramCASn),    
    .we_n(sdramWEn),    
    .addr(sdramAddress),    
    .ba(sdramBank),    
    .dq(sdramDq),    
    .cs_n(sdramCSn),    
    .dm0(sdramDataMasks[0:0]),    
    .dm1(sdramDataMasks[1:1]),    
    .dm2(sdramDataMasks[2:2]),    
    .dm3(sdramDataMasks[3:3]),    
    .cke(sdramClkEn)    
);    


