## SPDX-License-Identifier: GPL-3.0-only
##
## GW2AR-LV18QN88C8/I7 SDRAM controller
##
## Copyright (C) 2023 Luís Mendes <luis.p.mendes@gmail.com>
##
import sys
import numpy as np
from typing import List
from enum import IntEnum

from amaranth import Elaboratable, Module, Signal, Array, Mux, ResetSignal, ClockSignal
from amaranth import Memory, ClockDomain, Const, Cat, Repl, unsigned
from amaranth.lib.coding import Decoder 
from amaranth.asserts import Assert, Cover, Assume, Past, Fell
from amaranth.sim import Simulator, Delay
from amaranth.build import Platform
from amaranth.cli import main_parser, main_runner

## This SDRAM controller implements only the basic commands.
## It does not allow interruptions of Writes with other Reads or Writes, nor Read with other Writes or Read operations.
## In its current form it only allows burst operations, either Read or Write. It has no operation scheduler.
## The GW2AR-LV18QN88C8/I7 timing parameters are not known, since the datasheets seem not to include this information.

#Apparently commands can be interleaved by NOP or Device De-Selection (DeviceDeSelect command)
#Do not allow: Write interruptions (Write interrupted by a write, write interrupted by a read)
#Do not allow: Read interruptions
#
#Burst write operation (see AS4C16M16SA - page 11)
#Burst configuration (Mode register cycle - page 14) 
#
#Burst write with auto-precharge (see AS4C16M16SA - page 13)
#A2 - A1 - A0 -> Burst length (see AS4C16M16SA - page 14)
#
#Termination of a burst write operation (see AS4C16M16SA - page 16)
#Termination of a burst read operation (see AS4C16M16SA - page 16)
#
#https://www.bx.psu.edu/~nate/pexpect/FSM.html
#

class BurstLengthField(IntEnum):
   BurstLength_1 = 0
   BurstLength_2 = 1
   BurstLength_4 = 2
   BurstLength_8 = 3
   BurstLength_Reserved1 = 4
   BurstLength_Reserved2 = 5
   BurstLength_Reserved3 = 6
   BurstLength_FullPage = 7
   
class BurstTypeField(IntEnum):
   Burst_Sequential = 0
   Burst_Interleaved = 1
   
class CASLatencyField(IntEnum):
   CAS_Reserved0 = 0
   CAS_Reserved1 = 1
   CAS_2Clocks   = 2
   CAS_3Clocks   = 3
   CAS_Reserved4 = 4

class TestModeField(IntEnum):
   NormalMode     = 0
   VendorUseOnly1 = 1
   VendorUseOnly2 = 2
   VendorUseOnly3 = 3
   
class BurstWriteMode(IntEnum):
   BurstReadBurstWrite  = 0
   BurstReadSingleWrite = 1
   
#-----------------------------------

class SDRAMControllerStates(IntEnum):
   InitOp = 0
   ConfigurationOp = 1
   Idle = 2
   RefreshOp = 3
   WriteBurstOp = 4
   WriteOp = 5
   ReadOp = 6
   Error = 7

class BankControllerStates(IntEnum):
   NotReady = 0
   Idle = 1
   Active = 2
   ActiveBurst = 3
   PreCharge = 4
   Refreshing = 5

class RowStates(IntEnum):
   Idle = 0
   PreCharge = 1
   Active = 2
   Activate = 3
   Read = 4
   Write = 5

#See page 13
class Command(IntEnum):
   NoCommand = 0
   BankActivate = 1
   BankPreCharge = 2
   PreChargeAll = 3
   Write = 4
   WriteAndAutoPreCharge = 5
   Read = 6
   ReadAndAutoPreCharge = 7
   ModeRegisterSet = 8
   NoOperation = 9
   BurstStop = 10
   DeviceDeSelect = 11
   AutoRefresh = 12
   SelfRefreshEntry = 13
   SelfRefreshExit = 14
   ClockSuspendModeExit = 15
   PowerDownModeExit = 16
   DataWrite_OutputEnable = 17
   DataMask_OutputDisable = 18
   
   
'''
SimpleSDRAMController class
---------------------------
Implements the HDL for a simple SDRAM controller for an embedded SDRAM chip like the one found in GoWin fpgas.

- The controller supports only full page burst reads and writes
- When the address is higher than zero, the read/write starts at that address and ends at the end of the full page, thus actually
  will not read the full page.
- Currently it is not possible to end the read/write before the full page address has been reached.
- The controller supports decomposing the SDRAM 32-bit word data width into separate 16-bit or 8-bit words thus making it
  16-bit oriented or byte oriented.
- When using 16-bit or byte oriented capabilities the 1 or 2-bit address offset must be all zeros, because the controller
  logic currently does not handle offsets within the word
- For 24-bit words the 8 MSB bits are filtered out with DQM and not used. In this configuration it would be possible to do a further
  upgrade to have another virtual SDRAM with byte oriented interface. 
'''
class SimpleSDRAMController(Elaboratable):
   def __init__(self, systemClockFrequency = 120e6,
                      dataByteWidth = 3, 
                      sdramRowAddressWidth = 11, sdramBanks = 4, sdramDataWidth = 32,
                      sdramMaxWords = 2*1024*1024, simulate = False):                  
      self.systemClockFrequency = systemClockFrequency
      self.simulate = simulate

      self.CASLatency = CASLatencyField.CAS_3Clocks
      self.tPowerUp_ns = 200000.0 #Power Up time with stable inputs - ns
      self.tRC_ns      =     55.0 #Row cycle time (same bank) - ns
      self.tRCD_ns     =     42.0 #RAS to CAS delay (same bank) - ns #30
      self.tRP_ns      =     15.0 #Precharge to refresh/row activate delay (same bank) - ns
      self.tRRD_ns     =     10.0 #Row activate to row activate delay (different banks)
      self.tRAS_ns     =     40.0 #Row activate to precharge time, minimum (same bank)
      self.tWR_ns      =     15.0 #Write recovery time
      self.tMRD_ns     =     30.0 #Mode register set cycle time
      self.tREFI_ns    =   7800.0 #Average refresh interval time - An autorefresh must be performed each 7.8us on average
            
      self.sdramBanks = sdramBanks
      
      self.sdramDataWidth          = sdramDataWidth
      self.sdramBankWidth          = int(np.log2(sdramBanks))
      self.sdramRowAddressWidth    = sdramRowAddressWidth
      self.sdramColumnAddressWidth = int(np.log2(int(sdramMaxWords / sdramBanks / 2**self.sdramRowAddressWidth)))
      self.sdramMaxWords           = sdramMaxWords
      
      assert sdramBanks == 4, 'Although configurable there are places where it is hardcoded to 4 banks'
      assert self.CASLatency == CASLatencyField.CAS_2Clocks or self.CASLatency == CASLatencyField.CAS_3Clocks, 'Invalid CAS Latency value specified'
      assert dataByteWidth == 1 or dataByteWidth == 2 or dataByteWidth == 3 or dataByteWidth == 4, 'Invalid memory controller interface data width'
      
      self.dataByteWidth = dataByteWidth
      self.roundedDataByteWidth = dataByteWidth
      if self.roundedDataByteWidth == 3:
         self.roundedDataByteWidth = 4
 
      self.nrOfReadWriteSuspendCycles = int((self.sdramDataWidth/8)/self.roundedDataByteWidth) - 1
      if self.nrOfReadWriteSuspendCycles > 0:
         self.suspendCyclesShiftBits = int(np.log2(self.nrOfReadWriteSuspendCycles+1)) - 1
      else:
         self.suspendCyclesShiftBits = 0
         
      #Required cycles at system clock frequency
      self.PowerUPcycles = int(np.ceil(self.tPowerUp_ns * 1e-9 * systemClockFrequency)) #PowerUp time in clock cycles
      self.RCcycles      = int(np.ceil(self.tRC_ns      * 1e-9 * systemClockFrequency)) #Row cycle time in clock cycles
      self.RCDcycles     = int(np.ceil(self.tRCD_ns     * 1e-9 * systemClockFrequency)) #Ras to CAS delay
      self.RPcycles      = int(np.ceil(self.tRP_ns      * 1e-9 * systemClockFrequency)) #Precharge to refresh/row activate delay (banks≠)
      self.RRDcycles     = int(np.ceil(self.tRRD_ns     * 1e-9 * systemClockFrequency)) #Row activate to row activate delay (banks≠)
      self.RAScycles     = int(np.ceil(self.tRAS_ns     * 1e-9 * systemClockFrequency)) #Row Active time
      self.WRcycles      = int(np.ceil(self.tWR_ns      * 1e-9 * systemClockFrequency)) #Write recovery time
      self.MRDcycles     = int(np.ceil(self.tMRD_ns     * 1e-9 * systemClockFrequency)) #Mode Register Delay
      self.REFIcycles    = int(np.ceil(self.tREFI_ns    * 1e-9 * systemClockFrequency)) #Average Refresh interval timer
      
      assert self.RCDcycles >= 3, 'There is a code dependency in Burst Write Op that needs RCDcycles to be higher or equal to three (SRAM latency), but was: ' + str(self.RCDcycles)
      #
      #SDRAM memory signals
      self.sdramClk       = Signal()
      self.sdramClkEn     = Signal()
      self.sdramRASn      = Signal()
      self.sdramCASn      = Signal()
      self.sdramWEn       = Signal()
      self.sdramCSn       = Signal()
      self.sdramAddress   = Signal(unsigned(self.sdramRowAddressWidth))
      self.sdramBank      = Signal(self.sdramBankWidth)
      self.sdramDqOut     = Signal(unsigned(self.sdramDataWidth))
      self.sdramDqIn      = Signal(unsigned(self.sdramDataWidth))
      self.sdramDqWRn     = Signal()
      self.sdramDataMasks = Signal(unsigned(4), reset = 0xF);
      #
      #Memory controller interface
      self.ctrlReady        = Signal()                                                                             #Controller is ready for a Read/Write operation
      #Write related signals
      self.ctrlWrAddress    = Signal(range(int(sdramMaxWords * (sdramDataWidth / 8) / self.roundedDataByteWidth))) #Complete address for the Write operation 
      self.ctrlWr           = Signal()                                                                             #Perform a (full page) write
      self.ctrlWrDataIn     = Signal(self.dataByteWidth * 8)                                                       #Controller data in word to write to SDRAM
      self.ctrlWrIncAddress = Signal()                                                                             #Controller output strobe signal to indicate an address increment
      self.ctrlWrInProgress = Signal()                                                                             #Write is in progress
      #Read related signals
      self.ctrlRdAddress    = Signal(range(int(sdramMaxWords * (sdramDataWidth / 8) / self.roundedDataByteWidth))) #Complete address for the Read operation 
      self.ctrlRd           = Signal()                                                                             #Perform a (full page) read
      self.ctrlRdDataOut    = Signal(self.dataByteWidth * 8)                                                       #Controller data out with SDRAM read word
      self.ctrlRdIncAddress = Signal()                                                                             #Controller output strobe signal to indicate an address increment
      self.ctrlRdInProgress = Signal()                                                                             #Read is in progress
      #
      #Internal signals
      self.errorState = Signal()
      #
      self.currentControllerState  = Signal(SDRAMControllerStates)
      self.previousControllerState = Signal(SDRAMControllerStates)
      #
      self.targetBankAddress   = Signal(unsigned(self.sdramBankWidth))
      self.targetRowAddress    = Signal(unsigned(self.sdramRowAddressWidth))
      self.targetColumnAddress = Signal(unsigned(self.sdramColumnAddressWidth))
      self.targetMask          = Signal(unsigned(4))
      
      self.banksShouldRefresh = Signal()
      self.allBanksIdle = Signal(reset = 1) #This will be overriden aynchronously, so no issue on reset state      
      self.targetBankCanActivate = Signal()
      self.targetBankCanPreCharge = Signal()
      self.targetBankRefreshCounter = Signal(range(self.REFIcycles))
      self.targetBankState = Signal(BankControllerStates)
      self.bankControllers = Array([SimpleBankController(self.REFIcycles, self.RAScycles, self.RCcycles, self.RRDcycles, systemClockFrequency) for _ in range(sdramBanks)])
      #for i in range(sdramBanks):
      #   bankController = SimpleBankController(self.REFIcycles, self.RAScycles, self.RCcycles, self.RRDcycles, systemClockFrequency)
      #   self.bankControllers.append(bankController)
      
      self.ctrlAddress               = Signal(range(int(sdramMaxWords * (sdramDataWidth / 8) / self.roundedDataByteWidth))) #Complete address where to start the RD/WR operation
      self.state                     = Signal(SDRAMControllerStates)
      self.nextCommand               = Signal(Command)
      self.currentCommand            = Signal(Command)
      self.cmdRemainingCyclesCounter = Signal(2)
      self.cmdCompleted              = Signal()
      self.cmdIndex                  = Signal(range(11))
      self.refreshCmdIndex           = Signal(range(2)) #Internal command index for the RefreshOp, so that it doesn't interfere with ReadOp and WriteBurstOp
      self.refreshRequired           = Signal() #Set be ReadOp or WriteBurstOp in case the refresh timing is to short to perform the burst, cleared by refreshOp
      self.powerUpCounter            = Signal(range(self.PowerUPcycles))
      self.delayCounter              = Signal(range(-1, max(self.RCcycles, self.RCDcycles, self.RPcycles, 
                                                            self.RAScycles, self.WRcycles, self.MRDcycles) + 1))                                                        
      self.burstWritesMode           = Signal() #Whether SDRAM is configured for Full-Page burst write mode or single word only                                                 
      self.pageColumnIndex           = Signal(self.sdramColumnAddressWidth) #Helper signal/counter that does not influence the SDRAM column address actual signals
      self.pageWords                 = int(2**(self.sdramColumnAddressWidth))      
      
      if self.nrOfReadWriteSuspendCycles > 0:
         self.suspendedCyclesCounter = Signal(range(self.nrOfReadWriteSuspendCycles + 1))
         #Arrays can both be indexed by Signals as well as literals
         self.rdDataRegisters  = Array([Signal(self.dataByteWidth * 8, name='rdDataRegister' + str(i)) for i in range(self.nrOfReadWriteSuspendCycles)])
         self.wrDataRegisters  = Array([Signal(self.dataByteWidth * 8, name='wrDataRegister' + str(i)) for i in range(self.nrOfReadWriteSuspendCycles)])

      self.maskBitOffset = 0
      if self.roundedDataByteWidth == 4:
         self.maskBitOffset = 0
      elif self.roundedDataByteWidth == 2:
         self.maskBitOffset = 1
      elif self.roundedDataByteWidth == 1:
         self.maskBitOffset = 2

      #Simulation only signals
      if simulate:
         self.totalCyclesCounter = Signal(32)

      print('// SPDX-License-Identifier: GPL-3.0-only');
      print('/*');
      print(' * GW2AR-LV18QN88C8/I7 SDRAM controller');
      print(' *');
      print(' * Copyright (C) 2023 Luís Mendes <luis.p.mendes@gmail.com>');
      print(' */');
            
      print('/* PageWords= ' + str(self.pageWords) + ', Address width: ' + str(self.sdramColumnAddressWidth) + ' *)')
      print('/* Number of Read/Write suspend cycles/wait states    : ' + str(self.nrOfReadWriteSuspendCycles) + ' *)')
      print('/* Row cycle time (same bank)                   - RC  : ' + str(self.RCcycles) + ' */')
      print('/* RAS to CAS delay (same bank)                 - RCD : ' + str(self.RCDcycles) + ' */')
      print('/* PreCharge to Refresh/Row activate (same bank)- RP  : ' + str(self.RPcycles) + ' */')
      print('/* Row activate to row activate (diff. banks)   - RRD : ' + str(self.RRDcycles) + ' */')
      print('/* Row activate to pre-charge cycles (same bank)- RAS : ' + str(self.RAScycles) + ' */')
      print('/* Write recovery time                          - WR  : ' + str(self.WRcycles) + ' */')
      print('/* Mode register set cycle cycles               - MRD : ' + str(self.MRDcycles) + ' */')
      print('/* Average refresh interval cycles              - REFI: ' + str(self.REFIcycles) + ' */')
      print('/* Mask bit offset                                    : ' + str(self.maskBitOffset) + ' */')
      print('/* Column address width                               : ' + str(self.sdramColumnAddressWidth) + ' */')
      print('/* Bank address width                                 : ' + str(self.sdramBankWidth) + ' */')
      print('/* Row address width                                  : ' + str(self.sdramRowAddressWidth) + ' */')
      
      assert self.RCcycles >= 1,  'RCcycles should have at least 1 cycle delay'
      assert self.RCDcycles >= 1, 'RCDcycles should have at least 1 cycle delay'
      assert self.RPcycles >= 1, 'RPcycles should have at least 1 cycle delay'
      assert self.RRDcycles >= 1, 'RRDcycles should have at least 1 cycle delay'
      assert self.RAScycles >= 1, 'RAScycles should have at least 1 cycle delay'
      assert self.WRcycles >= 1, 'WRcycles should have at least 1 cycle delay'
      assert self.MRDcycles >= 1, 'MRDcycles should have at least 1 cycle delay'      

   '''
   # applySDRAMCommand
   # -----------------
   # Maps all valid comands into their respective signals and handles the asertion and de-assertion of clkEn-1 signal state
   #
   # TODO: - To make the function easier to user one could add the DeviceDeDeselect as the last comand cycle, since it only
   #       requires setting the CSn signal and would make it easier to implement the SDRAM controller state machine.
   #       - One could also embbed the setModeConfig() into this function, by passing an optional named tuple with the
   #       SDRAM mode config 
   '''            
   def applySDRAMCommand(self, m : Module, cmd : Command):
      #Note: Commands can only be issued in proper states..., check if state is Ok, otherwise set the error signal
      if cmd == Command.BankActivate:
            m.d.comb += self.nextCommand.eq(Command.BankActivate)
            m.d.clkSDRAM += self.currentCommand.eq(Command.BankActivate)
            
            with m.If((self.targetBankState != BankControllerStates.Idle) | ~self.targetBankCanActivate):
               m.d.clkSDRAM += self.errorState.eq(1)
            with m.Elif(self.sdramClkEn == 0):
               m.d.clkSDRAM += self.sdramClkEn.eq(1)
               m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
            with m.Elif(self.sdramClkEn == 1):
               m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
               m.d.comb += self.cmdCompleted.eq(1)
               for bankIndex in range(self.sdramBanks): 
                  with m.If(self.targetBankAddress == bankIndex):
                     m.d.comb += self.bankControllers[bankIndex].bankActivated.eq(1)
                     with m.If(self.burstWritesMode):
                        m.d.clkSDRAM += self.bankControllers[bankIndex].bankState.eq(BankControllerStates.ActiveBurst)
                     with m.Else():
                        m.d.clkSDRAM += self.bankControllers[bankIndex].bankState.eq(BankControllerStates.Active)                     
                  with m.Else():
                     m.d.comb += self.bankControllers[bankIndex].otherBankActivated.eq(1)
               #
               m.d.clkSDRAM += self.sdramBank.eq(self.targetBankAddress)
               m.d.clkSDRAM += self.sdramAddress.eq(self.targetRowAddress)
               m.d.clkSDRAM += self.sdramCSn.eq(0)
               m.d.clkSDRAM += self.sdramRASn.eq(0)
               m.d.clkSDRAM += self.sdramCASn.eq(1)
               m.d.clkSDRAM += self.sdramWEn.eq(1)            
      elif cmd == Command.BankPreCharge:
            m.d.comb += self.nextCommand.eq(Command.BankPreCharge)
            m.d.clkSDRAM += self.currentCommand.eq(Command.BankPreCharge)
            with m.If(self.sdramClkEn == 0):
               m.d.clkSDRAM += self.sdramClkEn.eq(1)
               m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
            with m.If(self.sdramClkEn == 1):
               m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
               m.d.comb += self.cmdCompleted.eq(1)       
               for bankIndex in range(self.sdramBanks): 
                  with m.If(self.targetBankAddress == bankIndex):
                     m.d.clkSDRAM += self.bankControllers[bankIndex].bankState.eq(BankControllerStates.Idle)
               #
               m.d.clkSDRAM += self.sdramAddress[10].eq(0)
               m.d.clkSDRAM += self.sdramCSn.eq(0)
               m.d.clkSDRAM += self.sdramRASn.eq(0)
               m.d.clkSDRAM += self.sdramCASn.eq(1)
               m.d.clkSDRAM += self.sdramWEn.eq(0)
      elif cmd == Command.PreChargeAll:
            m.d.comb += self.nextCommand.eq(Command.PreChargeAll)      
            m.d.clkSDRAM += self.currentCommand.eq(Command.PreChargeAll)
                        
            with m.If(self.sdramClkEn == 0):
               m.d.clkSDRAM += self.sdramClkEn.eq(1)
               m.d.comb     += self.cmdRemainingCyclesCounter.eq(1)
            with m.If(self.sdramClkEn == 1):
               m.d.comb     += self.cmdRemainingCyclesCounter.eq(0)
               m.d.clkSDRAM += self.delayCounter.eq(self.RPcycles - 1)
               #---
               m.d.clkSDRAM += self.sdramAddress[10].eq(1)
               m.d.clkSDRAM += self.sdramCSn.eq(0)
               m.d.clkSDRAM += self.sdramRASn.eq(0)
               m.d.clkSDRAM += self.sdramCASn.eq(1)
               m.d.clkSDRAM += self.sdramWEn.eq(0)               
               #Move all banks to the Idle state
               for bankIndex in range(self.sdramBanks):
                  m.d.clkSDRAM += self.bankControllers[bankIndex].bankState.eq(BankControllerStates.Idle)
            with m.If(self.delayCounter > 0):
               m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
            with m.Elif(self.delayCounter == 0):
               m.d.comb += self.cmdCompleted.eq(1)
               m.d.clkSDRAM += self.delayCounter.eq(-1)
      elif cmd == Command.Write:
            m.d.comb += self.nextCommand.eq(Command.Write)
            m.d.clkSDRAM += self.currentCommand.eq(Command.Write)

            with m.If((self.targetBankState != BankControllerStates.Active) &
                      (self.targetBankState != BankControllerStates.ActiveBurst)):
                m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.comb += self.cmdCompleted.eq(1)                  
                  m.d.clkSDRAM += self.sdramDataMasks.eq(self.targetMask)
                  m.d.clkSDRAM += self.sdramBank.eq(self.targetBankAddress)
                  #NOTE: This will also force A10 to low logic level, as required by the command
                  m.d.clkSDRAM += self.sdramAddress.eq(Cat(self.targetColumnAddress, Repl(0, len(self.targetRowAddress) - len(self.targetColumnAddress))))
                  m.d.clkSDRAM += self.sdramCSn.eq(0)
                  m.d.clkSDRAM += self.sdramRASn.eq(1)                  
                  m.d.clkSDRAM += self.sdramCASn.eq(0)
                  m.d.clkSDRAM += self.sdramWEn.eq(0)
      elif cmd == Command.WriteAndAutoPreCharge:
            m.d.comb += self.nextCommand.eq(Command.WriteAndAutoPreCharge)
            m.d.clkSDRAM += self.currentCommand.eq(Command.WriteAndAutoPreCharge)
            with m.If((self.targetBankState != BankControllerStates.Active) &
                      (self.targetBankState != BankControllerStates.ActiveBurst)):
                m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.comb += self.cmdCompleted.eq(1)
                  m.d.clkSDRAM += self.sdramDataMasks.eq(self.targetMask)
                  m.d.clkSDRAM += self.sdramBank.eq(self.targetBankAddress)
                  m.d.clkSDRAM += self.sdramAddress.eq(Cat(self.targetColumnAddress, Repl(0, len(self.targetRowAddress) - len(self.targetColumnAddress))))
                  #NOTE: This will override A10 write intention to high logic level, as required by the command
                  m.d.clkSDRAM += self.sdramAddress[10].eq(1)
                  m.d.clkSDRAM += self.sdramCSn.eq(0)
                  m.d.clkSDRAM += self.sdramRASn.eq(1)                  
                  m.d.clkSDRAM += self.sdramCASn.eq(0)
                  m.d.clkSDRAM += self.sdramWEn.eq(0)
      elif cmd == Command.Read:
            m.d.comb += self.nextCommand.eq(Command.Read)
            m.d.clkSDRAM += self.currentCommand.eq(Command.Read)            

            with m.If((self.targetBankState != BankControllerStates.Active) &
                      (self.targetBankState != BankControllerStates.ActiveBurst)):
                m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.comb += self.cmdCompleted.eq(1)
                  m.d.clkSDRAM += self.sdramBank.eq(self.targetBankAddress)
                  #NOTE: This will also force A10 to low logic level, as required by the command
                  m.d.clkSDRAM += self.sdramAddress.eq(Cat(self.targetColumnAddress, Repl(0, len(self.targetRowAddress) - len(self.targetColumnAddress))))
                  m.d.clkSDRAM += self.sdramCSn.eq(0)
                  m.d.clkSDRAM += self.sdramRASn.eq(1)
                  m.d.clkSDRAM += self.sdramCASn.eq(0)
                  m.d.clkSDRAM += self.sdramWEn.eq(1)
      elif cmd == Command.ReadAndAutoPreCharge:
            m.d.comb += self.nextCommand.eq(Command.ReadAndAutoPreCharge)
            m.d.clkSDRAM += self.currentCommand.eq(Command.ReadAndAutoPreCharge)            

            with m.If((self.targetBankState != BankControllerStates.Active) &
                      (self.targetBankState != BankControllerStates.ActiveBurst)):
                m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.clkSDRAM += self.sdramBank.eq(self.targetBankAddress)
                  m.d.clkSDRAM += self.sdramAddress.eq(Cat(self.targetColumnAddress, Repl(0, len(self.targetRowAddress) - len(self.targetColumnAddress))))
                  #NOTE: This will override A10 write intention to high logic level, as required by the command
                  m.d.clkSDRAM += self.sdramAddress[10].eq(1)
                  m.d.clkSDRAM += self.sdramCSn.eq(0)
                  m.d.clkSDRAM += self.sdramRASn.eq(1)                  
                  m.d.clkSDRAM += self.sdramCASn.eq(0)
                  m.d.clkSDRAM += self.sdramWEn.eq(1)
      elif cmd == Command.ModeRegisterSet:
            #This command is applicable to the whole memory 
            #(all banks, so the bank selection is used for configuration too)
            m.d.comb += self.nextCommand.eq(Command.ModeRegisterSet)            
            m.d.clkSDRAM += self.currentCommand.eq(Command.ModeRegisterSet)
            #All banks must be Idle
            with m.If(~self.allBanksIdle):
               m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.clkSDRAM += self.delayCounter.eq(self.MRDcycles - 1)
                  #Mode must also be set to the address lines with setModeConfig(...)
                  m.d.clkSDRAM += self.sdramCSn.eq(0)             
                  m.d.clkSDRAM += self.sdramRASn.eq(0)
                  m.d.clkSDRAM += self.sdramCASn.eq(0)
                  m.d.clkSDRAM += self.sdramWEn.eq(0)
               with m.If(self.delayCounter > 0):
                  m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
               with m.Elif(self.delayCounter == 0):
                  m.d.comb += self.cmdCompleted.eq(1)
                  m.d.clkSDRAM += self.delayCounter.eq(-1)
      elif cmd == Command.NoOperation:         
            m.d.comb += self.nextCommand.eq(Command.NoOperation)
            m.d.clkSDRAM += self.currentCommand.eq(Command.NoOperation)
            with m.If(self.sdramClkEn == 0):
               m.d.clkSDRAM += self.sdramClkEn.eq(1)
               m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
            with m.If(self.sdramClkEn == 1):
               m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
               m.d.comb += self.cmdCompleted.eq(1)
               m.d.clkSDRAM += self.sdramCSn.eq(0)
               m.d.clkSDRAM += self.sdramRASn.eq(1)               
               m.d.clkSDRAM += self.sdramCASn.eq(1)
               m.d.clkSDRAM += self.sdramWEn.eq(1)
      elif cmd == Command.BurstStop:
            m.d.comb += self.nextCommand.eq(Command.BurstStop)
            m.d.clkSDRAM += self.currentCommand.eq(Command.BurstStop)            
            with m.If(self.targetBankState != BankControllerStates.ActiveBurst):
               m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.clkSDRAM += self.sdramCSn.eq(0)
                  m.d.clkSDRAM += self.sdramRASn.eq(1)                  
                  m.d.clkSDRAM += self.sdramCASn.eq(1)
                  m.d.clkSDRAM += self.sdramWEn.eq(0)
      elif cmd == Command.DeviceDeSelect:
            m.d.comb += self.nextCommand.eq(Command.DeviceDeSelect)
            m.d.clkSDRAM += self.currentCommand.eq(Command.DeviceDeSelect)            
            with m.If(self.sdramClkEn == 0):
               m.d.clkSDRAM += self.sdramClkEn.eq(1)
               m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
            with m.If(self.sdramClkEn == 1):
               m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
               m.d.comb += self.cmdCompleted.eq(1)
               m.d.clkSDRAM += self.sdramCSn.eq(1)
      elif cmd == Command.AutoRefresh:      
            m.d.comb += self.nextCommand.eq(Command.AutoRefresh)
            m.d.clkSDRAM += self.currentCommand.eq(Command.AutoRefresh)            
            with m.If(~self.allBanksIdle):
               m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.comb += self.cmdCompleted.eq(1)
                  #
                  for bankIndex in range(self.sdramBanks):
                     m.d.clkSDRAM += self.bankControllers[bankIndex].bankState.eq(BankControllerStates.Refreshing)               
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.clkSDRAM += self.sdramCSn.eq(0)
                  m.d.clkSDRAM += self.sdramRASn.eq(0)
                  m.d.clkSDRAM += self.sdramCASn.eq(0)
                  m.d.clkSDRAM += self.sdramWEn.eq(1)
      elif cmd == Command.SelfRefreshEntry:
            m.d.comb += self.nextCommand.eq(Command.SelfRefreshEntry)
            m.d.clkSDRAM += self.currentCommand.eq(Command.SelfRefreshEntry)            
            with m.If(~self.allBanksIdle):
               m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.clkSDRAM += self.sdramClkEn.eq(0)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.clkSDRAM += self.sdramCSn.eq(0)
                  m.d.clkSDRAM += self.sdramCASn.eq(0)
                  m.d.clkSDRAM += self.sdramRASn.eq(0)
                  m.d.clkSDRAM += self.sdramWEn.eq(1)         
      elif cmd == Command.SelfRefreshExit:
            m.d.comb += self.nextCommand.eq(Command.SelfRefreshExit)
            m.d.clkSDRAM += self.currentCommand.eq(Command.SelfRefreshExit)            
            #Check for for the all banks Idle state *and* if the self refresh entry has been executed
            with m.If(~self.allBanksIdle): #| !self.inSelfRefresh
               m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 1):
                  m.d.clkSDRAM += self.sdramClkEn.eq(0)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.clkSDRAM += self.sdramCSn.eq(0)
                  m.d.clkSDRAM += self.sdramRASn.eq(1)
                  m.d.clkSDRAM += self.sdramCASn.eq(1)
                  m.d.clkSDRAM += self.sdramWEn.eq(1)
      elif cmd == Command.DataWrite_OutputEnable:
            m.d.comb += self.nextCommand.eq(Command.DataWrite_OutputEnable)
            m.d.clkSDRAM += self.currentCommand.eq(Command.DataWrite_OutputEnable)            
            with m.If((self.targetBankState != BankControllerStates.Active) &
                      (self.targetBankState != BankControllerStates.ActiveBurst)):
               m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.clkSDRAM += self.sdramDataMasks.eq(Repl(0, 4))
      elif cmd == Command.DataMask_OutputDisable:
            m.d.comb += self.nextCommand.eq(Command.DataMask_OutputDisable)
            m.d.clkSDRAM += self.currentCommand.eq(Command.DataMask_OutputDisable)            
            with m.If((self.targetBankState != BankControllerStates.Active) &
                      (self.targetBankState != BankControllerStates.ActiveBurst)):
               m.d.clkSDRAM += self.errorState.eq(1)
            with m.Else():      
               with m.If(self.sdramClkEn == 0):
                  m.d.clkSDRAM += self.sdramClkEn.eq(1)
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
               with m.If(self.sdramClkEn == 1):
                  m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
                  m.d.comb += self.cmdCompleted.eq(1)
                  m.d.clkSDRAM += self.sdramDataMasks.eq(Repl(1, 4))
      else:
            #Send a device de-select command and make sure the last cycle had CKEn H
            m.d.comb += self.nextCommand.eq(Command.DeviceDeSelect)
            m.d.clkSDRAM += self.currentCommand.eq(Command.DeviceDeSelect)            
            with m.If(self.sdramClkEn == 0):
               m.d.clkSDRAM += self.sdramClkEn.eq(1)
               m.d.comb += self.cmdRemainingCyclesCounter.eq(1)
            with m.If(self.sdramClkEn == 1):
               m.d.comb += self.cmdRemainingCyclesCounter.eq(0)
               m.d.clkSDRAM += self.sdramCSn.eq(1)

   def setModeConfig(self, m : Module, burstLength : BurstLengthField, burstType : BurstTypeField, casLatency : CASLatencyField, testMode : TestModeField, writeMode : BurstWriteMode):
      m.d.clkSDRAM += self.sdramBank.eq(0)
      m.d.clkSDRAM += self.sdramAddress[10:].eq(0)
      m.d.clkSDRAM += self.sdramAddress[9].eq(writeMode)
      m.d.clkSDRAM += self.sdramAddress[7:9].eq(testMode)
      m.d.clkSDRAM += self.sdramAddress[4:7].eq(casLatency)
      m.d.clkSDRAM += self.sdramAddress[3].eq(burstType)
      m.d.clkSDRAM += self.sdramAddress[0:3].eq(burstLength)
      
   def elaborate(self, platform : Platform):
      m = Module()
      
      clkSDRAM = ClockSignal(domain="clkSDRAM")
      rstSDRAM = ResetSignal(domain="clkSDRAM")

      if self.simulate:
         m.d.clkSDRAM += self.totalCyclesCounter.eq(self.totalCyclesCounter + 1)
      
      m.d.comb += self.sdramClk.eq(clkSDRAM)
      
      for bankIndex in range(self.sdramBanks):
         exec('m.submodules.bankController' + str(bankIndex) + ' = self.bankControllers[bankIndex]') 
            
      for bankIndex in range(self.sdramBanks):
         with m.If(self.bankControllers[bankIndex].bankState != BankControllerStates.Idle):
            m.d.comb += self.allBanksIdle.eq(0)

      for bankIndex in range(self.sdramBanks):
         with m.If(self.bankControllers[bankIndex].bankShouldRefresh):
            m.d.comb += self.banksShouldRefresh.eq(1)
            
      #Define decoding logic for the Bank states given the currently selected targetBankAddress
      m.d.comb += self.targetBankState.eq(self.bankControllers[self.targetBankAddress].bankState)
      m.d.clkSDRAM += self.targetBankCanActivate.eq(self.bankControllers[self.targetBankAddress].bankCanActivate)
      m.d.clkSDRAM += self.targetBankCanPreCharge.eq(self.bankControllers[self.targetBankAddress].bankCanPreCharge)
      m.d.clkSDRAM += self.targetBankRefreshCounter.eq(self.bankControllers[self.targetBankAddress].bankREFIcyclesCounter)
      
      assert(2**(len(self.targetBankAddress) + len(self.targetRowAddress) + len(self.targetColumnAddress)) == self.sdramMaxWords) 
            
      #The overall external memory address has the bank bits appearing after the column bits, thus to allow interleaved bank accesses
      #That is.. while a bank is precharging, another bank can be read or written to, but it is never possible to refresh an individual bank.
      #So if a sequential access is made, then, we can achieve max. memory throughput.
      columnAddressBitOffset = self.maskBitOffset
      bankAddressBitOffset   = columnAddressBitOffset + self.sdramColumnAddressWidth
      rowAddressBitOffset    = bankAddressBitOffset + self.sdramBankWidth



      m.d.comb += self.targetColumnAddress.eq(self.ctrlAddress[columnAddressBitOffset : bankAddressBitOffset])
      m.d.comb += self.targetBankAddress.eq(self.ctrlAddress[bankAddressBitOffset : rowAddressBitOffset])
      m.d.comb += self.targetRowAddress.eq(self.ctrlAddress[rowAddressBitOffset : ])
      
      #Decode maskBits to DQM signal bits, according to the byte, 16-bit word, 24-bit word or 32-bit word boundaries
      #NOTE1: It is not possible to have the Writes to write partial words for different cycles at the same target address.
      #- Based on this to keep the behavior consistent it is recommended to use the Suspended read and write methods,
      #  in order to give time for the whole word to be assembled during a write and disassembled during a read.
      #How it works... The client hardware performs three consecutive writes to a 24-bit register during three clock cycles,
      # and at the fourth clock cycle the whole word 32-bit word is assembled and written in a single cycle to the SDRAM, followed by
      # another three cycles of suspended write, to accumulate the following 24-bits and then the write of the full 32-bit word. 
      #NOTE2: DQM must be controlled differently depending wether it is a Read or a Write...
      #- On Writes, setting the DQM will mask immediatelly the corresponding bytes, ignoring those bytes (masking).
      #- On Reads, setting the DQM will make the output driver of the corresponding bytes go into high impedance state, 
      #  but it has a two cycle latency, so it must be set two cycles earlier, to take effect at the expected clock cycle.
      #IMPORTANT: Special care must be taken into account when interrupting a Read into a Write or a Write with a PreCharge.
      #targetMask is a helper signal for the whole Write and Read process, so it is independent of the acutal SDRAM DQM (DataMask)
      #signals.      
      if self.dataByteWidth == 4 or self.dataByteWidth == 2 or self.dataByteWidth == 1:
         m.d.comb += self.targetMask.eq( 0x0 )
      elif self.dataByteWidth == 3:
         m.d.comb += self.targetMask.eq( 0x8 )
      
      repeatRefresh = Signal() #Helper signal to check if the AutoRefresh command was repeated, since the specs. require two AutoRefreshes on power up      
      
      with m.FSM(domain='clkSDRAM', reset='InitOp', name="sdramCtrlr"):
         with m.State('InitOp'):
            m.next = 'InitOp'
            m.d.comb += self.currentControllerState.eq(SDRAMControllerStates.InitOp)
            with m.If(self.errorState):
               m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
               m.next = 'Error'
            with m.Else():
               with m.If(self.cmdIndex == 0):
                  self.applySDRAMCommand(m, Command.DeviceDeSelect)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += repeatRefresh.eq(0)
                     m.d.clkSDRAM += self.powerUpCounter.eq(self.PowerUPcycles)
                     m.d.clkSDRAM += self.cmdIndex.eq(1)
                     m.d.clkSDRAM += self.sdramClkEn.eq(0)
                     m.d.clkSDRAM += self.sdramRASn.eq(1)
                     m.d.clkSDRAM += self.sdramCASn.eq(1)
                     m.d.clkSDRAM += self.sdramWEn.eq(1)
               with m.If(self.cmdIndex == 1):
                  with m.If(self.powerUpCounter > 0):
                     m.d.clkSDRAM += self.powerUpCounter.eq(self.powerUpCounter - 1)
                  with m.Else():
                     m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
                     m.d.clkSDRAM += self.cmdIndex.eq(0)
                     m.d.clkSDRAM += self.delayCounter.eq(-1)
                     m.next = 'ConfigurationOp'
         with m.State('ConfigurationOp'):
            m.next = 'ConfigurationOp'                
            m.d.comb += self.currentControllerState.eq(SDRAMControllerStates.ConfigurationOp)
            with m.If(self.errorState):
               m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
               m.next = 'Error'
            with m.Else():
               with m.If(self.cmdIndex == 0):                 
                  self.applySDRAMCommand(m, Command.PreChargeAll)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += repeatRefresh.eq(1)
                     m.d.clkSDRAM += self.cmdIndex.eq(1)                     
               with m.If(self.cmdIndex == 1):
                  self.applySDRAMCommand(m, Command.DeviceDeSelect)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.cmdIndex.eq(2)
               with m.If(self.cmdIndex == 2):                     
                  self.applySDRAMCommand(m, Command.ModeRegisterSet)
                  with m.If(self.cmdRemainingCyclesCounter == 0):
                     self.setModeConfig(m, BurstLengthField.BurstLength_FullPage, BurstTypeField.Burst_Sequential, \
                                           self.CASLatency, TestModeField.NormalMode, BurstWriteMode.BurstReadBurstWrite)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.burstWritesMode.eq(1)
                     m.d.clkSDRAM += self.cmdIndex.eq(3)
               with m.If(self.cmdIndex == 3):
                  self.applySDRAMCommand(m, Command.AutoRefresh)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.cmdIndex.eq(4)
               with m.If(self.cmdIndex == 4):
                  self.applySDRAMCommand(m, Command.DeviceDeSelect)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.cmdIndex.eq(5)
                     m.d.clkSDRAM += self.delayCounter.eq(self.RCcycles - 1)
               with m.If(self.cmdIndex == 5):
                  with m.If(self.delayCounter > 0):
                     m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
                  with m.Elif(self.delayCounter == 0):
                     m.d.clkSDRAM += self.delayCounter.eq(-1)
                     m.d.clkSDRAM += self.cmdIndex.eq(6)
               with m.If(self.cmdIndex == 6):
                  for bankIndex in range(self.sdramBanks):
                     m.d.clkSDRAM += self.bankControllers[bankIndex].bankState.eq(BankControllerStates.Idle)               
                  with m.If(repeatRefresh):
                     m.d.clkSDRAM += repeatRefresh.eq(0)
                     m.d.clkSDRAM += self.cmdIndex.eq(3)
                  with m.Else():
                     m.d.clkSDRAM += self.ctrlReady.eq(1)
                     m.d.clkSDRAM += self.cmdIndex.eq(0)
                     m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
                     m.next = 'Idle'
         with m.State('Idle'):
            m.next = 'Idle'
            m.d.comb += self.currentControllerState.eq(SDRAMControllerStates.Idle)
            self.applySDRAMCommand(m, Command.NoOperation)
            with m.If(self.errorState):
               m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
               m.next = 'Error'
            with m.Elif(self.cmdCompleted):            
               with m.If(self.banksShouldRefresh):
                  m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
                  m.next = 'RefreshOp'
               with m.Elif(self.ctrlRd):
                  m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
                  m.d.clkSDRAM += self.ctrlReady.eq(0)
                  m.d.clkSDRAM += self.ctrlAddress.eq(self.ctrlRdAddress)
                  with m.If(~self.ctrlReady):
                     m.next = 'ReadOp'
               with m.Elif(self.ctrlWr & self.burstWritesMode):
                  m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
                  m.d.clkSDRAM += self.ctrlReady.eq(0)
                  m.d.clkSDRAM += self.ctrlAddress.eq(self.ctrlWrAddress)
                  with m.If(~self.ctrlReady):
                     m.next = 'WriteBurstOp'
               #with m.Elif(self.ctrlWr & !self.burstWritesMode):
               #   m.next += 'Write'
         with m.State('ReadOp'):
            #NOTE: The PreCharge command can be moved two cycles earlier (CASLat=2), or three cycles earlier (CASLat=3), discarding the DQM masking
            #As this would minimize the Burst read operation timing and DQM masking is not needed before applying the Pre-Charge (see AS4C16M16SA, page 10).
            m.d.comb += self.currentControllerState.eq(SDRAMControllerStates.ReadOp)
            m.next = 'ReadOp'
            with m.If(self.errorState):
               m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
               m.next = 'Error'
            with m.Else():
               with m.If(self.cmdIndex == 0):
                  #272  = 256 clocks data + RCDcycles(3) + WRCycles(1) + RPCycles(3) + Activate(2) + Write(1) + Precharge(1) + Waits(3) + NOPs(2)
                  #528  = 256 * 2 clocks data + RCDcycles(3) + WRCycles(1) + RPCycles(3) + Activate(2) + Write(1) + Precharge(1) + Waits(3) + NOPs(2)
                  #1040 = 256 * 4 clocks data + RCDcycles(3) + WRCycles(1) + RPCycles(3) + Activate(2) + Write(1) + Precharge(1) + Waits(3) + NOPs(2)
                  m.d.clkSDRAM += self.refreshRequired.eq(self.targetBankRefreshCounter < (((self.pageWords - self.targetColumnAddress) << self.suspendCyclesShiftBits) + 15 + 5))
                  m.d.clkSDRAM += self.cmdIndex.eq(1)
               with m.If(self.cmdIndex == 1):
                  # = #DataClocks + 14
                  with m.If(self.refreshRequired):
                     #Preventive refresh... to ensure that the maximum average refresh interval REFI is not exceeded
                     m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
                     m.next = 'RefreshOp'
                  with m.Elif(self.targetBankCanActivate):
                     self.applySDRAMCommand(m, Command.BankActivate)
                     with m.If(self.cmdCompleted):
                        m.d.clkSDRAM += self.sdramDataMasks.eq(self.targetMask)
                        m.d.clkSDRAM += self.cmdIndex.eq(2)
               with m.If(self.cmdIndex == 2):
                  #Implement RAS to CAS delay with device de-selection
                  self.applySDRAMCommand(m, Command.DeviceDeSelect)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.delayCounter.eq(self.RCDcycles - 1)
                     m.d.clkSDRAM += self.cmdIndex.eq(3)
               with m.If(self.cmdIndex == 3):   
                  with m.If(self.delayCounter > 0):
                     m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
                  with m.Elif(self.delayCounter == 0):
                     #Delay completed start read operation
                     m.d.clkSDRAM += self.delayCounter.eq(-1)
                     #Note: Read and AutoPreCharge will not perform the PreCharge for full page bursts,
                     #so we use the Read instead.
                     self.applySDRAMCommand(m, Command.Read)
                     with m.If(self.cmdCompleted):
                        m.d.clkSDRAM += self.pageColumnIndex.eq(self.targetColumnAddress)
                        m.d.clkSDRAM += self.cmdIndex.eq(4)
               with m.If(self.cmdIndex == 4):
                  self.applySDRAMCommand(m, Command.DeviceDeSelect)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.cmdIndex.eq(5)
               with m.If(self.cmdIndex == 5):
                  if self.CASLatency == CASLatencyField.CAS_2Clocks:
                     m.d.clkSDRAM += self.ctrlRdInProgress.eq(1)
                     m.d.clkSDRAM += self.cmdIndex.eq(7)
                     if self.nrOfReadWriteSuspendCycles > 0:
                        m.d.clkSDRAM += self.suspendedCyclesCounter.eq(0)
                        m.d.clkSDRAM += self.sdramCKEn.eq(0) #Suspend the read at the second cycle from now
                  else:
                     m.d.clkSDRAM += self.cmdIndex.eq(6)
               with m.If(self.cmdIndex == 6):
                  #Case for CAS latency of 3 clocks only
                  m.d.clkSDRAM += self.ctrlRdInProgress.eq(1)
                  m.d.clkSDRAM += self.cmdIndex.eq(7)                
                  if self.nrOfReadWriteSuspendCycles > 0:
                     m.d.clkSDRAM += self.suspendedCyclesCounter.eq(0)
                     m.d.clkSDRAM += self.sdramClkEn.eq(0) #Suspend the read at the second cycle from now
               with m.If(self.cmdIndex == 7):
                  #Process the data read operation
                  #Both the 2 and 3 CAS latency read cases merge here                  
                  m.d.clkSDRAM += self.ctrlRdIncAddress.eq(1)
                  if self.nrOfReadWriteSuspendCycles > 0:
                     with m.If(self.suspendedCyclesCounter == 0):
                        for i in range(self.nrOfReadWriteSuspendCycles + 1):
                           if i == 0:
                              m.d.comb += self.ctrlRdDataOut.eq(self.sdramDqOut[i*self.dataByteWidth*8:(i+1)*self.dataByteWidth*8])
                           else:
                              m.d.clkSDRAM += self.rdDataRegisters[i-1].eq(self.sdramDqOut[i*self.dataByteWidth*8:(i+1)*self.dataByteWidth*8])                             
                     with m.Else():
                        m.d.comb += self.ctrlRdDataOut.eq(self.rdDataRegisters[self.suspendedCyclesCounter - 1])

                     with m.If(self.suspendedCyclesCounter == self.nrOfReadWriteSuspendCycles - 1):
                           #Since applySDRAMCommand also takes care of setting sdramClkEn, and we don't want
                           #to override, otherwise the applySDRAMCommand will be partially skipped.
                           m.d.clkSDRAM += self.sdramClkEn.eq(1) #Resume the read command at the last cycle (next cycle)
                     with m.If(self.suspendedCyclesCounter == self.nrOfReadWriteSuspendCycles):
                        #If we have suspendedCycles, that means that we either have 2 or 4 read cycles,
                        #thus we can be sure that the Output disable has to be asserted in the last
                        #word, at last cycle self.nrOfReadWriteSuspendCycles, so that the next cycle, is DQM masked.
                        #Given the conditions this should complete in one cycle...
                        with m.If(self.pageColumnIndex == self.pageWords - 1):
                           self.applySDRAMCommand(m, Command.DataMask_OutputDisable)
                        with m.Else():
                           m.d.clkSDRAM += self.sdramClkEn.eq(0) #Suspend the read command at the first cycle (next cycle)
                        
                     with m.If(self.suspendedCyclesCounter < self.nrOfReadWriteSuspendCycles):
                        m.d.clkSDRAM += self.suspendedCyclesCounter.eq(self.suspendedCyclesCounter + 1)
                     with m.Else():
                        with m.If(self.pageColumnIndex < self.pageWords - 1):
                           m.d.clkSDRAM += self.pageColumnIndex.eq(self.pageColumnIndex + 1)
                           m.d.clkSDRAM += self.suspendedCyclesCounter.eq(0)
                        with m.Else():
                           m.d.clkSDRAM += self.suspendedCyclesCounter.eq(0)
                           m.d.clkSDRAM += self.pageColumnIndex.eq(0)
                        #with m.If(self.pageColumnIndex == self.pageWords - 1 - 1):
                           m.d.clkSDRAM += self.cmdIndex.eq(8)
                  else:
                     m.d.comb += self.ctrlRdDataOut.eq(self.sdramDqOut[:self.dataByteWidth*8])
                     with m.If(self.pageColumnIndex < self.pageWords - 1):
                        m.d.clkSDRAM += self.pageColumnIndex.eq(self.pageColumnIndex + 1)
                     with m.Else():
                        m.d.clkSDRAM += self.cmdIndex.eq(8)
                        m.d.clkSDRAM += self.pageColumnIndex.eq(0)
                     with m.If(self.pageColumnIndex == self.pageWords - 1 - 1):
                        self.applySDRAMCommand(m, Command.DataMask_OutputDisable)           
               with m.If(self.cmdIndex == 8):
                  m.d.comb += self.ctrlRdDataOut.eq(self.sdramDqOut[:self.dataByteWidth*8])
                  m.d.clkSDRAM += self.ctrlRdInProgress.eq(0)
                  m.d.clkSDRAM += self.ctrlRdIncAddress.eq(0)
                  #Start Pre-Charge
                  with m.If(self.targetBankCanPreCharge):
                     self.applySDRAMCommand(m, Command.BankPreCharge)
                     with m.If(self.cmdCompleted):
                        m.d.clkSDRAM += self.cmdIndex.eq(9)
               with m.If(self.cmdIndex == 9):
                  #De-Select the SDRAM bus, i.e., do not isue any new command
                  self.applySDRAMCommand(m, Command.DeviceDeSelect)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.delayCounter.eq(self.RPcycles - 1)
                     m.d.clkSDRAM += self.cmdIndex.eq(10)
               with m.If(self.cmdIndex == 10):
                  #Wait for the Pre-Charge time to elapse
                  with m.If(self.delayCounter > 0):
                     m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
                  with m.If(self.delayCounter == 0):
                     m.d.clkSDRAM += self.delayCounter.eq(-1)
                     m.d.clkSDRAM += self.cmdIndex.eq(0)
                     m.d.clkSDRAM += self.ctrlReady.eq(1)                     
                     m.next = 'Idle'                              
         with m.State('WriteBurstOp'):
            '''
            Amaranth does not allow bidirectional inout bus.
            '''
            m.d.comb += self.currentControllerState.eq(SDRAMControllerStates.WriteBurstOp)
            m.next = 'WriteBurstOp'
            with m.If(self.errorState):
               m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
               m.next = 'Error'
            with m.Else():            
               with m.If(self.cmdIndex == 0):
                  #141 = 128 clocks data + RCDcycles(3) + WRCycles(1) + RPCycles(3) + Activate(1) + Write(1) + Precharge(1) + NOPs(2)) + 4 cycles margin
                  m.d.clkSDRAM += self.refreshRequired.eq(self.targetBankRefreshCounter < ((self.pageWords - self.targetColumnAddress) << self.suspendCyclesShiftBits) + 13 + 5)
                  m.d.clkSDRAM += self.cmdIndex.eq(1)
               with m.If(self.cmdIndex == 1):
                  with m.If(self.refreshRequired):
                     #Preventive refresh... to ensure that the maximum average refresh interval REFI is not exceeded
                     m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
                     m.next = 'RefreshOp'
                  with m.Elif(self.targetBankCanActivate):
                     self.applySDRAMCommand(m, Command.BankActivate)
                     with m.If(self.cmdCompleted):
                        m.d.clkSDRAM += self.sdramDataMasks.eq(self.targetMask)
                        m.d.clkSDRAM += self.cmdIndex.eq(2)
               with m.If(self.cmdIndex == 2):
                  self.applySDRAMCommand(m, Command.NoOperation)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.delayCounter.eq(self.RCDcycles - 1)
                     m.d.clkSDRAM += self.cmdIndex.eq(3)
               with m.If(self.cmdIndex == 3):   
                  if (self.nrOfReadWriteSuspendCycles > 0):
                     with m.If(self.delayCounter > 0):
                        m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
                        m.d.clkSDRAM += self.suspendedCyclesCounter.eq(0)
                     with m.If(self.delayCounter == 0):
                        #We must accumulate the remaining words in the registers
                        m.d.clkSDRAM += self.ctrlWrInProgress.eq(1)
                        with m.If(self.suspendedCyclesCounter < self.nrOfReadWriteSuspendCycles):
                           m.d.clkSDRAM += self.pageColumnIndex.eq(self.targetColumnAddress)
                           m.d.clkSDRAM += self.wrDataRegisters[self.suspendedCyclesCounter].eq(self.ctrlWrDataIn)       
                           m.d.clkSDRAM += self.suspendedCyclesCounter.eq(self.suspendedCyclesCounter + 1)
                        with m.Else():
                           m.d.clkSDRAM += self.delayCounter.eq(-1)
                           m.d.clkSDRAM += self.cmdIndex.eq(4)                              
                           m.d.clkSDRAM += self.suspendedCyclesCounter.eq(0)
                           m.d.clkSDRAM += self.sdramDqWRn.eq(1) #Set SDRAM helper mux to write mode
                           for i in range(self.nrOfReadWriteSuspendCycles):
                              m.d.clkSDRAM += self.sdramDqIn[i * self.dataByteWidth * 8 : (i+1) * self.dataByteWidth * 8].eq(self.wrDataRegisters[i])
                           m.d.clkSDRAM += self.sdramDqIn[self.nrOfReadWriteSuspendCycles * self.dataByteWidth * 8 : (self.nrOfReadWriteSuspendCycles+1) * self.dataByteWidth * 8].eq(self.ctrlWrDataIn)
                           m.d.clkSDRAM += self.pageColumnIndex.eq(self.pageColumnIndex + 1)
                           self.applySDRAMCommand(m, Command.Write)
                           with m.If (~self.cmdCompleted):
                              m.d.clkSDRAM += self.errorState.eq(1)
                              m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
                              m.next = 'Error'
                  else:
                     with m.If(self.delayCounter > 0):
                        m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
                        with m.If(self.delayCounter == 2):
                            m.d.clkSDRAM += self.ctrlWrIncAddress.eq(1)
                     with m.If(self.delayCounter == 0):
                        m.d.clkSDRAM += self.sdramDqWRn.eq(1) #Set to write mode
                        #Delay completed; start the read operation
                        m.d.clkSDRAM += self.delayCounter.eq(-1)
                        #Note: Write and AutoPreCharge will not perform the PreCharge for full page bursts,
                        #so we use the Write instead.
                        m.d.clkSDRAM += self.sdramDqIn[:self.dataByteWidth * 8].eq(self.ctrlWrDataIn)
                        self.applySDRAMCommand(m, Command.Write)
                        m.d.clkSDRAM += self.ctrlWrInProgress.eq(1)
                        m.d.clkSDRAM += self.pageColumnIndex.eq(self.targetColumnAddress)
                        with m.If(self.cmdCompleted):
                            m.d.clkSDRAM += self.cmdIndex.eq(4)                                                      
               with m.If(self.cmdIndex == 4):
                  #Writing page into SDRAM
                  if (self.nrOfReadWriteSuspendCycles > 0):
                     with m.If((self.pageColumnIndex == self.targetColumnAddress) &
                               (self.suspendedCyclesCounter == 0)):
                        self.applySDRAMCommand(m, Command.DeviceDeSelect)
                  else:
                     with m.If(self.pageColumnIndex == 0): #self.targetColumnAddress):
                        self.applySDRAMCommand(m, Command.DeviceDeSelect)
                        
                  if self.nrOfReadWriteSuspendCycles > 0:
                     #We must accumulate the remaining words in the registers
                     with m.If(self.suspendedCyclesCounter < self.nrOfReadWriteSuspendCycles):
                        #Also suspend the write for the nrOfReadWriteSuspendCycles
                        m.d.clkSDRAM += self.sdramClkEn.eq(0)
                        m.d.clkSDRAM += self.suspendedCyclesCounter.eq(self.suspendedCyclesCounter + 1)
                        m.d.clkSDRAM += self.wrDataRegisters[self.suspendedCyclesCounter].eq(self.ctrlWrDataIn)
                     with m.Else():
                        m.d.clkSDRAM += self.sdramClkEn.eq(1)
                        with m.If(self.ctrlWrIncAddress == 1):
                           m.d.clkSDRAM += self.suspendedCyclesCounter.eq(0)
                           for i in range(self.nrOfReadWriteSuspendCycles):
                              m.d.clkSDRAM += self.sdramDqIn[i * self.dataByteWidth * 8 : (i+1) * self.dataByteWidth * 8].eq(self.wrDataRegisters[i])
                           m.d.clkSDRAM += self.sdramDqIn[self.nrOfReadWriteSuspendCycles * self.dataByteWidth * 8 : 
                                                        (self.nrOfReadWriteSuspendCycles+1) * self.dataByteWidth * 8].eq(self.ctrlWrDataIn)
                        with m.If(self.pageColumnIndex < self.pageWords - 1):
                           m.d.clkSDRAM += self.pageColumnIndex.eq(self.pageColumnIndex + 1)                       
                        with m.Else():
                           m.d.clkSDRAM += self.sdramDqWRn.eq(0) #End write by making sure that we tri-state the input latch
                           m.d.clkSDRAM += self.ctrlWrInProgress.eq(0)
                           with m.If(self.ctrlWrInProgress == 0):
                              self.applySDRAMCommand(m, Command.DataMask_OutputDisable)
                              with m.If(self.cmdCompleted):
                                 m.d.clkSDRAM += self.cmdIndex.eq(5)
                                 m.d.clkSDRAM += self.delayCounter.eq(self.WRcycles - 1)
                        with m.If(self.pageColunIndex == self.pageWords - 4):
                           m.d.clkSDRAM += self.ctrlWrIncAddress.eq(0)
                  else:
                     m.d.clkSDRAM += self.sdramDqIn[:self.dataByteWidth * 8].eq(self.ctrlWrDataIn)
                     with m.If(self.pageColumnIndex < self.pageWords - 1):
                        m.d.clkSDRAM += self.pageColumnIndex.eq(self.pageColumnIndex + 1)
                     with m.Else():
                        m.d.clkSDRAM += self.sdramDqWRn.eq(0) #End write by making sure that we tri-state the input latch
                        self.applySDRAMCommand(m, Command.DataMask_OutputDisable)
                        with m.If(self.cmdCompleted):                           
                           m.d.clkSDRAM += self.ctrlWrInProgress.eq(0)
                           m.d.clkSDRAM += self.cmdIndex.eq(5)
                           m.d.clkSDRAM += self.delayCounter.eq(self.WRcycles - 1)
                     with m.If(self.pageColumnIndex == self.pageWords - 4):
                        m.d.clkSDRAM += self.ctrlWrIncAddress.eq(0)
               with m.If(self.cmdIndex == 5):
                  self.applySDRAMCommand(m, Command.DeviceDeSelect)
                  with m.If(self.delayCounter > 0):
                     m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.cmdIndex.eq(6)
               with m.If(self.cmdIndex == 6):
                  with m.If(self.delayCounter > 0):
                     m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
                  with m.If(self.delayCounter == 0):
                     with m.If(self.targetBankCanPreCharge):
                        self.applySDRAMCommand(m, Command.BankPreCharge)
                        m.d.clkSDRAM += self.delayCounter.eq(self.RPcycles - 1)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.cmdIndex.eq(7)
               with m.If(self.cmdIndex == 7):
                  self.applySDRAMCommand(m, Command.DeviceDeSelect)
                  with m.If(self.delayCounter > 0):
                     m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
                  with m.If(self.delayCounter == 0):
                     m.d.clkSDRAM += self.delayCounter.eq(-1)
                     m.d.clkSDRAM += self.cmdIndex.eq(0)
                     m.d.clkSDRAM += self.ctrlReady.eq(1)
                     m.next = 'Idle'
         with m.State('RefreshOp'):
            m.d.comb += self.currentControllerState.eq(SDRAMControllerStates.RefreshOp)         
            m.next = 'RefreshOp'
            with m.If(self.errorState):
               m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
               m.next = 'Error'
            with m.Else():
               with m.If(self.refreshCmdIndex == 0):
                  self.applySDRAMCommand(m, Command.AutoRefresh)
                  with m.If(self.cmdCompleted):
                     m.d.clkSDRAM += self.refreshCmdIndex.eq(1)
                     m.d.clkSDRAM += self.delayCounter.eq(self.RCcycles - 1)
               with m.If(self.refreshCmdIndex == 1):
                     self.applySDRAMCommand(m, Command.DeviceDeSelect)
                     with m.If(self.delayCounter > 0):
                        m.d.clkSDRAM += self.delayCounter.eq(self.delayCounter - 1)
                     with m.Elif((self.delayCounter == 0) & self.cmdCompleted):
                        m.d.clkSDRAM += self.delayCounter.eq(-1)
                        m.d.clkSDRAM += self.refreshCmdIndex.eq(0)
                        m.d.clkSDRAM += self.refreshRequired.eq(0)
                        for bankIndex in range(self.sdramBanks):
                           m.d.clkSDRAM += self.bankControllers[bankIndex].bankState.eq(BankControllerStates.Idle)               
                        m.d.clkSDRAM += self.previousControllerState.eq(self.currentControllerState)
                        with m.If(self.previousControllerState == SDRAMControllerStates.WriteBurstOp):
                           m.next = 'WriteBurstOp'
                        with m.If(self.previousControllerState == SDRAMControllerStates.ReadOp):
                           m.next = 'ReadOp'
                        with m.Else():
                           m.next = 'Idle'               
         with m.State('Error'):
            m.d.comb += self.currentControllerState.eq(SDRAMControllerStates.Error)
            m.next = 'Error'
            pass  
      
      return m
      
   def ports(self) -> List[Signal]:
        ports = [
                 self.sdramClk,
                 self.sdramClkEn,
                 self.sdramRASn,
                 self.sdramCASn,
                 self.sdramWEn,
                 self.sdramCSn,
                 self.sdramAddress,
                 self.sdramBank,
                 self.sdramDqOut,
                 self.sdramDqIn,
                 self.sdramDqWRn,
                 self.sdramDataMasks, 
                 self.ctrlReady,
                 self.ctrlWrAddress,
                 self.ctrlWr,
                 self.ctrlWrDataIn,
                 self.ctrlWrIncAddress,
                 self.ctrlRdAddress,
                 self.ctrlRd,
                 self.ctrlRdDataOut,
                 self.ctrlRdIncAddress
               ]
               
        if simulate:
           ports.append(self.totalCyclesCounter)
           
        return ports


#Simple Bank Controller takes care of the current bank state
#Currently for the overall SDRAM controller there won't be more than
#one bank active at a time. Also, since we are doing full page bursts,
#we will always need to activate another row, for the next access.
#
#In a more complete desgin this would take care of the refresh times,
#for each bank, to issue a refresh command. Refresh command requires
#all banks to be in idle state.
class SimpleBankController(Elaboratable):
   def __init__(self, REFIcycles : int, RAScycles : int, RCcycles : int, RRDcycles : int, systemClockFrequency = 100e6):
       self.REFIcycles = REFIcycles
       self.RAScycles = RAScycles
       self.RCcycles = RCcycles
       self.RRDcycles = RRDcycles
   
       self.bankREFIcyclesCounter = Signal(range(REFIcycles)) #Bank Refresh Interval average clock cycles
       self.bankRAScyclesCounter = Signal(range(RAScycles))
       self.bankActivatedCounter = Signal(range(RCcycles))
       self.otherBankActivatedCounter = Signal(range(RRDcycles))
       self.bankState = Signal(BankControllerStates, reset=BankControllerStates.NotReady)
       self.bankShouldRefresh = Signal() #Whether this bank is needing a Refresh
       self.bankCanPreCharge = Signal()  #Whether this bank can be PreCharged
       self.bankCanActivate = Signal(reset = 1)   #Whether this bank can be Activated
       self.bankActivated = Signal()     #Strobe this line to indicate that this bank was activated
       self.otherBankActivated = Signal()#Strobe this line to indicate that another bank was activated
       
   #NOTES: 
   #When a bank is activated it has to wait tRCD cycles before doing a Read Or Write, and the wait time must be filled with No Operations
   #When the same bank is to be activated again it has to wait at least tRC cycles between sucessive activations and a 
   #Precharge command must have been done before accessing the next row.
   #When activating a different bank it must wait at least tRRD cycles due to activation shared logic between banks
   
   def elaborate(self, platform : Platform):
       m = Module()
       
       with m.If((self.bankState == BankControllerStates.Idle) | (self.bankState == BankControllerStates.Active) | (self.bankState == BankControllerStates.ActiveBurst)):
          with m.If(self.bankREFIcyclesCounter > 0):
             m.d.clkSDRAM += self.bankREFIcyclesCounter.eq(self.bankREFIcyclesCounter - 1)
          with m.If(self.bankREFIcyclesCounter <= 3):
             m.d.clkSDRAM += self.bankShouldRefresh.eq(1) #This wastes one cycle so (REFIcycles - 1)
       with m.Else():
          m.d.clkSDRAM += self.bankREFIcyclesCounter.eq(self.REFIcycles - 1)
          m.d.clkSDRAM += self.bankShouldRefresh.eq(0)

       with m.If((self.bankState == BankControllerStates.Active) | (self.bankState == BankControllerStates.ActiveBurst)):
          with m.If(self.bankRAScyclesCounter < self.RAScycles - 1):
             m.d.clkSDRAM += self.bankCanPreCharge.eq(0)
             m.d.clkSDRAM += self.bankRAScyclesCounter.eq(self.bankRAScyclesCounter + 1)
          with m.Else():
             m.d.clkSDRAM += self.bankCanPreCharge.eq(1)
       with m.Else():
          m.d.clkSDRAM += self.bankCanPreCharge.eq(1)
          m.d.clkSDRAM += self.bankRAScyclesCounter.eq(0)

       m.d.comb += self.bankCanActivate.eq((self.bankActivatedCounter == 0) & (self.otherBankActivatedCounter == 0))

       with m.If(self.bankActivated):
          m.d.clkSDRAM += self.bankActivatedCounter.eq(self.RCcycles - 1)
       with m.Elif(self.bankActivatedCounter > 0):
          m.d.clkSDRAM += self.bankActivatedCounter.eq(self.bankActivatedCounter - 1)
       
       with m.If(self.otherBankActivated):
          m.d.clkSDRAM += self.otherBankActivatedCounter.eq(self.RRDcycles - 1)
       with m.Elif(self.otherBankActivatedCounter > 0):
          m.d.clkSDRAM += self.otherBankActivatedCounter.eq(self.otherBankActivatedCounter - 1)
       
       return m

def printSimulateArgumentHelp():
    print('Simulation agurments help table')
    print('')
    print('When more than one simulation argument, use \',\' (comma) as an option sepparator')
    print('')
    print('--simulate <simulationType>')
    print('   <simulationType> can be either Refresh, Read, Write, ReadRefresh or WriteRefresh')
    print('--simulate dataWidth=<byteWidth>, where byteWidth can be either 1, 2, 3 or 4')
               
class SimulationTypeEnum(IntEnum):
   Refresh = 0,
   Read = 1,
   Write = 2,
   ReadRefresh = 3,
   WriteRefresh = 4
   
               
if __name__ == "__main__":
    parser = main_parser()
    parser.add_argument("--simulate")
    args = parser.parse_args()

    simulate = False
    simulationType = SimulationTypeEnum.Refresh
    dataByteWidth = 3
    if args.simulate is not None:
        simulate = True
        inputArgs = str.lower(args.simulate.strip())
        if inputArgs.startswith('help'):
            printSimulateArgumentHelp()
            sys.exit()
        optionsSplitted = inputArgs.split(',')
        for option in optionsSplitted:
           if option == 'read':
              simulationType = SimulationTypeEnum.Read
           elif option == 'write':
              simulationType = SimulationTypeEnum.Write
           elif option == 'readrefresh':
              simulationType = SimulationTypeEnum.ReadRefresh
           elif option == 'writerefresh':
              simulationType = SimulationTypeEnum.WriteRefresh
           elif option == 'refresh':
              simulationType = SimulationTypeEnum.Refresh
           elif option.startswith('datawidth='):
              try:
                 dataByteWidth = int(option[len('datawidth=')])
              except ValueError:
                 print('Invalid dataByteWidth specified (expecting an integer number of 1, 2, 3 or 4')
                 printSimulateArgumentHelp()
                 sys.exit()
              if dataByteWidth < 1 or dataByteWidth > 4:
                 print('Invalid dataByteWidth specified (expecting an integer number of 1, 2, 3 or 4')
                 printSimulateArgumentHelp()
                 sys.exit()
              dataByteWidth = dataByteWidth
           else:
              print('Unknown option: ' + option)
              print()
              printSimulateArgumentHelp()
              sys.exit()
    
        print('Starting simulation with parameters:')
        print('Simulation type: ' + str(simulationType))
        print('Data byte width: ' + str(dataByteWidth))
    
    m = Module()
    m.domains.clkSDRAM = clkDom = ClockDomain('clkSDRAM')
    
    if simulate:
        m.submodules.sdramController = sdramController = SimpleSDRAMController(dataByteWidth=dataByteWidth, simulate = True)
        
        #IMPORTANT NOTE: Memory simulation code and other simulation conditions must be set before instantiating the Simulator, otherwise they
        #take no effect.

        #NOTE: The sim prefix should be appended to all simulation only signals, since due the hierarchy flattening,
        #      originated by the sdramDq bidirectional signal, there will be more trouble to identify whether the signal
        #      is a simulation helper signal or a real signal.
        simPreviousRdDataMasks=Array([Signal(len(sdramController.sdramDataMasks), name='simPreviousRdDataMasks0')])#, 
                                   #Signal(len(sdramController.sdramDataMasks), name='simPreviousRdDataMasks1')])
        #m.d.clkSDRAM += simPreviousRdDataMasks[1].eq(sdramController.sdramDataMasks)
        #m.d.clkSDRAM += simPreviousRdDataMasks[0].eq(simPreviousRdDataMasks[1])
        m.d.clkSDRAM += simPreviousRdDataMasks[0].eq(sdramController.sdramDataMasks)

        maxAddress = sdramController.pageWords - 1

        # Fake memory
        mem = {
            0x0000: 0xAABBCCDD,
            0x0001: 0xBEEFFEEB,
            0x0002: 0xCAAABBBC,
        }

        for i in range(3, maxAddress-1):
           mem[i] = i

        mem[maxAddress - 1] = 0xFEEFDAAD
        mem[maxAddress    ] = 0xAFBFCFDF

        #IMPORTANT: This condition is crucial for the Writes to work, since m.d.comb will assign sdramDq data lines as a permanent output,
        #and will cause the simulation to fail
        if simulationType == SimulationTypeEnum.Read or simulationType == SimulationTypeEnum.ReadRefresh:        
           simMemMask = Signal(sdramController.sdramDataWidth)
           with m.If(sdramController.ctrlRdIncAddress):
              #NOTE: Instead of setting masked values to FF, this should instead make those bits tri-stated,
              #      however Amaranth/nMigen does not support tri-stating a Signal, only Pins. 
              for i in range(2**len(sdramController.sdramDataMasks)):
                 buildMemMask = 0
                 with m.Switch(simPreviousRdDataMasks[0]):
                    with m.Case(i):
                       if i & 0x01:
                          buildMemMask |= 0xff
                       if i & 0x02:
                          buildMemMask |= 0xff00
                       if i & 0x04:
                          buildMemMask |= 0xff0000
                       if i & 0x08:
                          buildMemMask |= 0xff000000
                       m.d.comb += simMemMask.eq(buildMemMask)

              with m.Switch(sdramController.pageColumnIndex):
                 for addr, data in mem.items():
                    with m.Case(addr):
                       m.d.comb += sdramController.sdramDqOut.eq(data | simMemMask | (sdramController.targetBankAddress << (sdramController.sdramColumnAddressWidth - 1)))
                 with m.Default():
                    m.d.comb += sdramController.sdramDqOut.eq(0x00 | simMemMask)
            
    
        #IMPORTANT NOTE: Memory simulation code and other simulation conditions must be set before instantiating the Simulator, otherwise they
        #take no effect. So make sure to not insert simulation conditions after this line.
    
        sim = Simulator(m)
        sim.add_clock(1.0/100e6, domain='clkSDRAM')
        
        #Default SDRAM configuration (3 bytes of data width)
        # 0 bits for byte mask
        # 7 bits Column address
        #11 bits Row address
        # 2 bits Bank address
        #Total: 20 bits -> 2^21 = 2 Mega words de 24/32bits
        
        powerUpDelayAndConfigCycles = sdramController.PowerUPcycles + sdramController.RPcycles + 2*sdramController.RCcycles + sdramController.MRDcycles
        
        def process():
            columnAddressMask = 0
            for i in range(sdramController.sdramColumnAddressWidth):
               columnAddressMask  |= 1 << i

            readAddress = 0x101 << sdramController.maskBitOffset
            readColumnAddress = (readAddress >> sdramController.maskBitOffset) & columnAddressMask

            writeAddress = 0x101 << sdramController.maskBitOffset
            writeColumnAddress = (writeAddress >> sdramController.maskBitOffset) & columnAddressMask

            readCycles = (((sdramController.pageWords - readColumnAddress) << sdramController.suspendCyclesShiftBits) + 14)
            
            wrCounter = writeColumnAddress * (sdramController.nrOfReadWriteSuspendCycles+1)
            
            if simulationType == SimulationTypeEnum.Read:
               for index in range(powerUpDelayAndConfigCycles + 800):
                  if (index == powerUpDelayAndConfigCycles + 20):
                     yield sdramController.ctrlRdAddress.eq(0x101 << sdramController.maskBitOffset)
                     yield sdramController.ctrlRd.eq(1)
                  elif (index == powerUpDelayAndConfigCycles + 21):
                     yield sdramController.ctrlRd.eq(0)
                  yield
            elif simulationType == SimulationTypeEnum.ReadRefresh:
               for index in range(powerUpDelayAndConfigCycles + 800):
                  if (index == powerUpDelayAndConfigCycles + 10 + (sdramController.REFIcycles - readCycles + 2)):
                     yield sdramController.ctrlRdAddress.eq(readAddress )
                     yield sdramController.ctrlRd.eq(1)
                  elif (index == powerUpDelayAndConfigCycles + 10 + (sdramController.REFIcycles - readCycles + 2) + 1):
                     yield sdramController.ctrlRd.eq(0)
                  yield                  
            elif simulationType == SimulationTypeEnum.Write:
               for index in range(powerUpDelayAndConfigCycles + 800):
                  if (index == powerUpDelayAndConfigCycles + 20):
                     yield sdramController.ctrlWrAddress.eq(0x101 << sdramController.maskBitOffset)
                     yield sdramController.ctrlWr.eq(1)
                  elif (index == powerUpDelayAndConfigCycles + 21):
                     yield sdramController.ctrlWr.eq(0)
                  elif (index >= powerUpDelayAndConfigCycles + 21 + sdramController.RCDcycles  + 1 and
                        index < powerUpDelayAndConfigCycles + 21 + sdramController.RCDcycles + 1 + 
                               (sdramController.pageWords - writeColumnAddress) * (sdramController.nrOfReadWriteSuspendCycles+1)):   
                     yield sdramController.ctrlWrDataIn.eq(wrCounter)
                     wrCounter = wrCounter + 1
                  yield
            else:
               for index in range(powerUpDelayAndConfigCycles + 800):
                  yield
                        
            
        sim.add_sync_process(process, domain='clkSDRAM')
        
        with sim.write_vcd("test.vcd", "test.gtkw", traces=sdramController.ports() + [sdramController.targetBankCanActivate, 
               sdramController.targetBankCanPreCharge, sdramController.targetBankState, sdramController.banksShouldRefresh, 
               sdramController.allBanksIdle, sdramController.currentCommand, sdramController.nextCommand, 
               sdramController.currentControllerState, sdramController.previousControllerState, sdramController.pageColumnIndex]):
            sim.run()
    else:
        #Generate with:
        #python3 sdram_controller.py generate -t v > sdram_controller.v
        m.submodules.sdramController = sdramController = SimpleSDRAMController(dataByteWidth=3, simulate = False)
        main_runner(parser, args, m, ports=sdramController.ports()) 
