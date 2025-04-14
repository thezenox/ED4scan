//--------------------------------------------------------------------------------
// (c) 2018 by MyLab-odyssey
//
// Licensed under "MIT License (MIT)", see license file for more information.
//
// THIS SOFTWARE IS ONLY INTENDED FOR SCIENTIFIC USAGE
// AND IS PROVIDED BY THE COPYRIGHT HOLDER OR CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//--------------------------------------------------------------------------------
//! \file    canDiagED4.cpp
//! \brief   Library module for retrieving diagnostic data.
//! \date    2018-September
//! \author  MyLab-odyssey
//! \version 0.5.5
//--------------------------------------------------------------------------------
#include "canDiagED4.h"
#include "ED4scan.h"


//--------------------------------------------------------------------------------
//! \brief   Standard constructor / destructor
//--------------------------------------------------------------------------------
canDiag::canDiag() {
}

canDiag::~canDiag() {
  delete[] data;
}

//--------------------------------------------------------------------------------
//! \brief   Manage memory for cell statistics
//--------------------------------------------------------------------------------

void canDiag::reserveMem_CellVoltage() {
  CellVoltage.init(CELLCOUNT);
}

void canDiag::reserveMem_CellCapacity() {
  CellCapacity.init(CELLCOUNT);
  //Serial.print(F("RAM: ")); Serial.println(this->_getFreeRam());
}

void canDiag::freeMem_CellVoltage() {
  CellVoltage.freeMem();
}

void canDiag::freeMem_CellCapacity() {
  CellCapacity.freeMem();
  //Serial.print(F("RAM: ")); Serial.println(this->_getFreeRam());
}

//--------------------------------------------------------------------------------
//! \brief   Memory available between Heap and Stack, works only on UNO!
//--------------------------------------------------------------------------------
int canDiag::_getFreeRam() {
  extern int __heap_start, *__brkval;
  int v;
  //return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
  return ESP.getFreeHeap();
}

//--------------------------------------------------------------------------------
//! \brief   Get method for CellVoltages
//--------------------------------------------------------------------------------
uint16_t canDiag::getCellVoltage(byte n) {
  return CellVoltage.get(n);
}

//--------------------------------------------------------------------------------
//! \brief   Get method for CellCapacities
//--------------------------------------------------------------------------------
uint16_t canDiag::getCellCapacity(byte n) {
  return CellCapacity.get(n);
}

//--------------------------------------------------------------------------------
//! \brief   Initialize CAN-Object and MCP2515 Controller
//--------------------------------------------------------------------------------
void canDiag::begin(MCP_CAN *_myCAN, CTimeout *_myCAN_Timeout) {
  //Set Pointer to MCP_CANobj
  myCAN0 = _myCAN;
  myCAN_Timeout = _myCAN_Timeout;

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters enabled.
  if (myCAN0->begin(MCP_STD, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    DEBUG_UPDATE(F("MCP2515 Init Okay!!\r\n"));
  } else {
    DEBUG_UPDATE(F("MCP2515 Init Failed!!\r\n"));
  }
  this->data = new byte[DATALENGTH];
}

//--------------------------------------------------------------------------------
//! \brief   Clear CAN ID filters.
//--------------------------------------------------------------------------------
void canDiag::clearCAN_Filter() {
  myCAN0->init_Mask(0, 0, 0x00000000);
  myCAN0->init_Mask(1, 0, 0x00000000);
  //delay(100);
  myCAN0->setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
}

//--------------------------------------------------------------------------------
//! \brief   Set all filters to one CAN ID.
//--------------------------------------------------------------------------------
void canDiag::setCAN_Filter(unsigned long filter) {
  this->respID = filter;
  filter = filter << 16;
  myCAN0->init_Mask(0, 0, 0x07FF0000);
  myCAN0->init_Mask(1, 0, 0x07FF0000);
  myCAN0->init_Filt(0, 0, filter);
  myCAN0->init_Filt(1, 0, filter);
  myCAN0->init_Filt(2, 0, filter);
  myCAN0->init_Filt(3, 0, filter);
  myCAN0->init_Filt(4, 0, filter);
  myCAN0->init_Filt(5, 0, filter);
  //delay(100);
  myCAN0->setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
}

//--------------------------------------------------------------------------------
//! \brief   Set request CAN ID and response CAN ID for get functions
//--------------------------------------------------------------------------------
void canDiag::setCAN_ID(unsigned long _respID) {
  if (this->respID != _respID) {
    this->setCAN_Filter(_respID);
  }
  this->respID = _respID;
}
void canDiag::setCAN_ID(unsigned long _rqID, unsigned long _respID) {
  rqID = _rqID;
  if (this->respID != _respID) {
    this->setCAN_Filter(_respID);
  }
  this->respID = _respID;
}

//--------------------------------------------------------------------------------
//! \brief   Set method for capacity readout (As/10 or As/100)
//--------------------------------------------------------------------------------
void canDiag::setCAPmode(byte _mode) {
  CapMode = _mode;
}

//--------------------------------------------------------------------------------
//! \brief   Try to wakeup EV CAN bus ***experimental and not working right now!***
//--------------------------------------------------------------------------------
boolean canDiag::WakeUp() {
  //--- Send WakeUp Pattern ---
  DEBUG_UPDATE(F("Send WakeUp Request\n\r"));
  //myCAN0->sendMsgBuf(0x423, 0, 7, rqWakeUp);    // send data: Request diagnostics data, 423!, 452?, 236?
  return true;
}

//--------------------------------------------------------------------------------
//! \brief   Send diagnostic request to ECU.
//! \param   byte* rqQuery
//! \see     rqBattADCref ... rqBattVolts
//! \return  received lines count (uint16_t) of function #Get_RequestResponse
//--------------------------------------------------------------------------------
uint16_t canDiag::Request_Diagnostics(const byte *_rqQuery) {

  // Detect skip data mode; size of request standard 4 parameter; skip enable (start param 5, stop param 6)
  if (SkipEnable) {
    //Copy request from prog memory with skip start and end
    /*memcpy_P(rqMsg + 1, _rqQuery, 5 * sizeof(byte)); // Fill byte 01 to 04 of rqMsg with rqQuery content (from PROGMEM)
      SkipStart = rqMsg[4];
      SkipEnd = rqMsg[5];
      rqMsg[4] = 0x00; rqMsg[5] = 0x00; // mark elements 5, 6 as unused data for correct query */
    //Serial.print("> "); Serial.print(SkipStart); Serial.print(" > "); Serial.println(SkipEnd);
  } else {
    //Copy request from prog memory and fill up for UDS request size of 8 parameters
    memcpy_P(rqMsg + 1, _rqQuery, 3 * sizeof(byte)); // Fill byte 02 to 04 of rqMsg with rqQuery content (from PROGMEM)
  }

  //Set Length of Query
  if (rqMsg[1] == 0x21) rqMsg[0] = 0x02; //Set Query Length to 2
  if (rqMsg[1] == 0x10) rqMsg[0] = 0x02; //Set Query Length to 2
  if (rqMsg[1] == 0x3E) rqMsg[0] = 0x02; //Set Query Length to 2
  if (rqMsg[1] == 0x22) rqMsg[0] = 0x03; //Set Query Length to 3

  /*Serial.println();
    for (byte i = 0; i < 8; i++) {
    Serial.print(rqMsg[i], HEX); Serial.print(", ");
    }
    Serial.println();*/

  myCAN_Timeout->Reset();                     // Reset Timeout-Timer

  //digitalWrite(CS_SD, HIGH);                // Disable SD card, or other SPI devices if nessesary

  //--- Diag Request Message ---
  DEBUG_UPDATE(F("Send Diag Request\n\r"));
  myCAN0->sendMsgBuf(rqID, 0, 8, rqMsg);      // send data: Request diagnostics data

  return this->Get_RequestResponse();         // wait for response of first frame
}

uint16_t canDiag::Request_Diagnostics(byte* _rqQuery) {

  myCAN_Timeout->Reset();                     // Reset Timeout-Timer

  //digitalWrite(CS_SD, HIGH);                // Disable SD card, or other SPI devices if nessesary

  //--- Diag Request Message ---
  DEBUG_UPDATE(F("Send Diag Request\n\r"));
  myCAN0->sendMsgBuf(rqID, 0, 8, _rqQuery);      // send data: Request diagnostics data

  return this->Get_RequestResponse();         // wait for response of first frame
}

//--------------------------------------------------------------------------------
//! \brief   Wait and read initial diagnostic response
//! \return  lines count (uint16_t) of received lines á 7 bytes
//--------------------------------------------------------------------------------
uint16_t canDiag::Get_RequestResponse() {

  byte i;
  uint16_t items = 0;
  boolean fDataOK = false;

  do {
    //--- Read Frames ---
    if (!digitalRead(MCP_INT))                        // If pin 2 is LOW, read receive buffer
    {
      do {
        myCAN0->readMsgBuf(&rxID, &len, rxBuf);    // Read data: len = data length, buf = data byte(s)

        if (rxID == this->respID) {
          if (rxBuf[0] < 0x10) {
            if ((rxBuf[1] != 0x7F)) {
              for (i = 0; i < len; i++) {       // read data bytes: offset +1, 1 to 7
                data[i] = rxBuf[i + 1];
              }
              DEBUG_UPDATE(F("SF reponse: "));
              DEBUG_UPDATE(rxBuf[0] & 0x0F); DEBUG_UPDATE("\n\r");
              items = 0;
              fDataOK = true;
            } else if (rxBuf[3] == 0x78) {
              DEBUG_UPDATE(F("pending reponse...\n\r"));
            } else {
              DEBUG_UPDATE(F("ERROR\n\r"));
            }
          }
          if ((rxBuf[0] & 0xF0) == 0x10) {
            items = (rxBuf[0] & 0x0F) * 256 + rxBuf[1]; // six data bytes already read (+ two type and length)
            for (i = 0; i < len; i++) {               // read data bytes: offset +1, 1 to 7
              data[i] = rxBuf[i + 1];
            }
            //--- send rqFC: Request for more data ---
            myCAN0->sendMsgBuf(this->rqID, 0, 8, rqFlowControl);
            DEBUG_UPDATE(F("Resp, i:"));
            DEBUG_UPDATE(items - 6); DEBUG_UPDATE("\n\r");
            fDataOK = Read_FC_Response(items - 6);
          }
        }
      } while (!digitalRead(MCP_INT) && !myCAN_Timeout->Expired(false) && !fDataOK);
    }
  } while (!myCAN_Timeout->Expired(false) && !fDataOK);

  this->SkipEnable = false;

  if (fDataOK) {
    return (items + 7) / 7;
    DEBUG_UPDATE(F("success!\n\r"));
  } else {
    DEBUG_UPDATE(F("Event Timeout!\n\r"));
    this->ClearReadBuffer();
    return 0;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Read remaining data and sent corresponding Flow Control frames
//! \param   items still to read (int)
//! \return  fDiagOK (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::Read_FC_Response(int16_t items) {
  myCAN_Timeout->Reset();

  byte i;
  int16_t n = 7;
  uint16_t rspLine = 0;
  int16_t FC_count = 0;
  byte FC_length = rqFlowControl[1];
  boolean fDiagOK = false;

  do {
    //--- Read Frames ---
    if (!digitalRead(MCP_INT))                        // If pin 2 is LOW, read receive buffer
    {
      do {
        myCAN0->readMsgBuf(&rxID, &len, rxBuf);    // Read data: len = data length, buf = data byte(s)
        if ((rxBuf[0] & 0xF0) == 0x20) {
          FC_count++;
          items = items - len + 1;
          for (i = 0; i < len; i++) {           // copy each byte of the rxBuffer to data-field
            if ((n < (DATALENGTH - 6)) && (i < 7)) {
              data[n + i] = rxBuf[i + 1];
            }
          }
          //--- FC counter -> then send Flow Control Message ---
          if (FC_count % FC_length == 0 && items > 0) {
            // send rqFC: Request for more data
            myCAN0->sendMsgBuf(this->rqID, 0, 8, rqFlowControl);
            DEBUG_UPDATE(F("FCrq\n\r"));
          }
          //--- Skip read data by using a write pointer (n) and a line counter (rspLine)
          if (this->SkipEnable && (rspLine + 1) >= this->SkipStart && (rspLine + 1) <= this->SkipEnd) {
            rspLine = rspLine + 1;
          } else {
            rspLine = rspLine + 1;
            n = n + 7;
          }
        }
      } while (!digitalRead(MCP_INT) && !myCAN_Timeout->Expired(false) && items > 0);
    }
  } while (!myCAN_Timeout->Expired(false) && items > 0);
  if (!myCAN_Timeout->Expired(false)) {
    fDiagOK = true;
    DEBUG_UPDATE(F("Items left: ")); DEBUG_UPDATE(items); DEBUG_UPDATE("\n\r");
    DEBUG_UPDATE(F("FC count: ")); DEBUG_UPDATE(FC_count); DEBUG_UPDATE("\n\r");
  } else {
    fDiagOK = false;
    DEBUG_UPDATE(F("Event Timeout!\n\r"));
  }
  this->ClearReadBuffer();
  return fDiagOK;
}

//--------------------------------------------------------------------------------
//! \brief   Output read buffer
//! \param   lines count (uint16_t)
//--------------------------------------------------------------------------------
void canDiag::PrintReadBuffer(uint16_t lines) {
  if (VERBOSE_ENABLE) {
    uint16_t pos;
    Serial.println(lines);
    for (uint16_t i = 0; i < lines; i++) {
      Serial.print(F("Data: "));
      for (byte n = 0; n < 7; n++)              // Print each byte of the data.
      {
        pos = n + 7 * i;
        if (pos <= DATALENGTH) {
          if (data[pos] < 0x10)            // If data byte is less than 0x10, add a leading zero.
          {
            Serial.print('0');
          }
          Serial.print(data[pos], HEX);
          Serial.print(" ");
        }
      }
      Serial.println();
    }
  }
}

//--------------------------------------------------------------------------------
//! \brief   Cleanup after switching filters
//--------------------------------------------------------------------------------
boolean canDiag::ClearReadBuffer() {
  if (!digitalRead(MCP_INT)) {                       // still messages? pin 2 is LOW, clear the two rxBuffers by reading
    for (byte i = 1; i <= 2; i++) {
      myCAN0->readMsgBuf(&rxID, &len, rxBuf);
    }
    DEBUG_UPDATE(F("Buffer cleared!\n\r"));
    return true;
  }
  return false;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate actual EV key state
//! \param   enable verbose / debug output (boolean)
//! \return  report state (on/off = true/false)
//--------------------------------------------------------------------------------
boolean canDiag::getKeyState(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  boolean fKey = false;

  this->setCAN_ID(rqID_EVC, respID_EVC);
  items = this->Request_Diagnostics(rqKeyState);

  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    fKey = data[3];
    myBMS->KeyState = fKey;
    return true;
  }
  return false;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate actual DC/DC state
//! \param   enable verbose / debug output (boolean)
//! \return  report state (on/off = true/false)
//--------------------------------------------------------------------------------
boolean canDiag::getDCDC_State(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;
  boolean fOK = false;

  this->setCAN_ID(rqID_EVC, respID_EVC);

  items = this->Request_Diagnostics(rqDCDC_Amps);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myBMS->LV_DCDC_amps = data[3];
    fOK = true;
  } else {
    fOK = false;
  }

  items = this->Request_Diagnostics(rqDCDC_State);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myBMS->LV_DCDC_state = data[3];
    fOK &= true;
  } else {
    fOK &= false;
  }

  items = this->Request_Diagnostics(rqDCDC_Load);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myBMS->LV_DCDC_load = data[3];
    if (myBMS->LV_DCDC_load == 0xFE) myBMS->LV_DCDC_load = 0; //0xFE DC/DC is OFF
    fOK &= true;
  } else {
    fOK &= false;
  }

  items = this->Request_Diagnostics(rqDCDC_Power);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 3, 1);
    myBMS->LV_DCDC_power = value;
    fOK &= true;
  } else {
    fOK &= false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Store two byte data in temperature array
//--------------------------------------------------------------------------------
void canDiag::ReadBatteryTemperatures(BatteryDiag_t *myBMS, byte data_in[], uint16_t highOffset, uint16_t length) {

  for (uint16_t n = 0; n < (length * 2); n = n + 2) {
    int16_t value = data_in[n + highOffset] * 256 + data_in[n + highOffset + 1];
    if (n > 4) {
      //Test for negative min. module temperature and apply offset
      if (myBMS->Temps[1] & 0x8000) {
        value = value - 0xA00; //minus offset
      }
    }
    myBMS->Temps[n / 2] = value;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Store two byte data in CellCapacity obj
//--------------------------------------------------------------------------------
void canDiag::ReadCellCapacity(byte data_in[], uint16_t highOffset, uint16_t length) {
  for (uint16_t n = 0; n < (length * 2); n = n + 2) {
    CellCapacity.push((data_in[n + highOffset] * 256 + data_in[n + highOffset + 1]));
  }
}

//--------------------------------------------------------------------------------
//! \brief   Store two byte data in CellVoltage obj
//--------------------------------------------------------------------------------
void canDiag::ReadCellVoltage(byte data_in[], uint16_t highOffset, uint16_t length) {
  uint16_t CV;
  for (uint16_t n = 0; n < (length * 2); n = n + 2) {
    CV = (data_in[n + highOffset] * 256 + data_in[n + highOffset + 1]);
    CV = CV / 1024.0 * 1000;
    CellVoltage.push(CV);
  }
}

//--------------------------------------------------------------------------------
//! \brief   Store two byte data
//! \param   address to output data array (uint16_t)
//! \param   address to input data array (uint16_t)
//! \param   start of first high byte in data array (uint16_t)
//! \param   length of data submitted (uint16_t)
//--------------------------------------------------------------------------------
void canDiag::ReadDiagWord(uint16_t data_out[], byte data_in[], uint16_t highOffset, uint16_t length) {
  for (uint16_t n = 0; n < (length * 2); n = n + 2) {
    data_out[n / 2] = data_in[n + highOffset] * 256 + data_in[n + highOffset + 1];
  }
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate battery temperatures (values / 64 in deg C)
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBatteryTemperature(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;

  this->setCAN_ID(rqID_BMS, respID_BMS);

  //Read nine temperatures per module (á 32 cells)
  items = this->Request_Diagnostics(rqBattTemperatures);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    //Serial.print(data[3], HEX); Serial.println(data[4], HEX);
    this->ReadBatteryTemperatures(myBMS, data, 3, 31);
    //Serial.println((float) myBMS->Temps[0] / 64, 1);
    //myBMS->Temps[33] = data[69]; //always 0xFF
    return true;
  } else {
    return false;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate battery and BMS production date
//! \brief   and date of factory acceptacnce test
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBatteryDate(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;

  this->setCAN_ID(rqID_BMS, respID_BMS);

  //Get date of Factory Acceptance Testing (FAT)
  items = this->Request_Diagnostics(rqBattProdDate);

  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myBMS->ProdYear = data[4];
    myBMS->ProdMonth = data[5];
    myBMS->ProdDay = data[6];
    return true;
  }
  return false;
}

//--------------------------------------------------------------------------------
//! \brief   Read the VIN stored in the battery and compare it to myVIN def.
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
/*boolean canDiag::getBatteryVIN(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;

  this->setCAN_ID(0x7E7, 0x7EF); // Currently from old ED_BMSdiag - do not use!
  //items = this->Request_Diagnostics(rqBattVIN);

  byte OKcount = 0;
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    for (byte n = 0; n < 17; n++) {
      //myBMS->BattVIN[n] =  data[n + 4];
      //if (myBMS->BattVIN[n] == myVIN[n]) OKcount++;
    }
    //return true if data completely matches
    if (OKcount == 17) {
      return true;
    } else {
      return false;
    }
  }
  return false;
}*/

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate battery high voltage status
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBalancingStatus(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;
  byte balState = 0;

  this->setCAN_ID(rqID_BMS, respID_BMS);
  items = this->Request_Diagnostics(rqBattBalancing);

  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    for (byte i = 0; i < CELLCOUNT; i++) {
      balState = balState ^ data[i + 3];
    }
    myBMS->BattBalXOR = balState;
    this->ReadDiagWord(&value, data, 99, 1);
    myBMS->BattBalState = value;
    return true;
  } else {
    return false;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate battery isolation resistance
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getIsolationValue(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;

  this->setCAN_ID(rqID_BMS, respID_BMS);
  items = this->Request_Diagnostics(rqBattIsolation);

  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 2, 1);
    myBMS->Isolation = (signed) value;
    myBMS->DCfault = data[4]; //Flags for isolation measurement
    return true;
  } else {
    return false;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate capacity data
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBatteryCapacity(BatteryDiag_t *myBMS, boolean debug_verbose) {
  debug_verbose = debug_verbose & VERBOSE_ENABLE;
  uint16_t items;
  uint16_t value;
  boolean fOK = false;

  this->setCAN_ID(rqID_BMS, respID_BMS);

  items = this->Request_Diagnostics(rqBattMeas_Capacity);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 2, 1);
    myBMS->CapMeas = value;
    fOK = true;
  } else {
    fOK = false;
  }

  items = this->Request_Diagnostics(rqBattCapacity_dSOC_P1);
  delay(1000);
  items = this->Request_Diagnostics(rqBattCapacity_dSOC_P1);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }

    myBMS->fSOH = data[9];

    if (CapMode == 1) {
      CellCapacity.clear();
      this->ReadCellCapacity(data, 16, (CELLCOUNT / 2)); //data starting at #16, but two mean values pushed before!
    }

    fOK &= true;
  } else {
    fOK &= false;
  }

  items = this->Request_Diagnostics(rqBattCapacity_dSOC_P2);
  delay(1000);
  items = this->Request_Diagnostics(rqBattCapacity_dSOC_P2);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    uint16_t value;
    this->ReadDiagWord(&value, data, 99, 1);
    myBMS->Cap_combined_quality = value / 65535.0;
    this->ReadDiagWord(&myBMS->LastMeas_days, data, 101, 1);
    this->ReadDiagWord(&value, data, 103, 1);
    myBMS->Cap_meas_quality = value / 65535.0;

    if (CapMode == 1) {
      this->ReadCellCapacity(data, 3, CELLCOUNT / 2);

      myBMS->Ccap_As.min = CellCapacity.minimum(&myBMS->CAP_min_at); //(int16_t *)
      //CellCapacity.push(myBMS->Ccap_As.min); CellCapacity.push(myBMS->Ccap_As.min);
      myBMS->Ccap_As.max = CellCapacity.maximum(&myBMS->CAP_max_at);
      myBMS->Ccap_As.mean = CellCapacity.mean();
    }

    fOK &= true;
  } else {
    fOK &= false;
  }

  //*** Read second capacity values (can be in As/100, depending of BMS rev.) ***
  items = this->Request_Diagnostics(rqBattCapacity_P1);
  delay(1000);
  items = this->Request_Diagnostics(rqBattCapacity_P1);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&myBMS->CAP2_mean, data, 3, 1);
    
    if (CapMode == 2) {
      CellCapacity.clear();
      this->ReadCellCapacity(data, 5, CELLCOUNT / 2);
    }

    fOK &= true;
  } else {
    fOK &= false;
  }

  //Determine capacity factor, depending on BMS rev.
  if (CellCapacity.mean() > 3000) {
    myBMS->CAP_factor = 1;
  } else {
    myBMS->CAP_factor = 10;
  }
  if (myBMS->CAP2_mean < 3000) {
    myBMS->CAP2_mean = myBMS->CAP2_mean * 10;
  }  

  items = this->Request_Diagnostics(rqBattCapacity_P2);
  delay(1000);
  items = this->Request_Diagnostics(rqBattCapacity_P2);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    if (CapMode == 2) {
      this->ReadCellCapacity(data, 3, CELLCOUNT / 2);
      myBMS->Ccap_As.min = CellCapacity.minimum(&myBMS->CAP_min_at) * myBMS->CAP_factor;//(int16_t *)
      myBMS->Ccap_As.max = CellCapacity.maximum(&myBMS->CAP_max_at) * myBMS->CAP_factor;
      myBMS->Ccap_As.mean = CellCapacity.mean() * myBMS->CAP_factor;
    }

    fOK &= true;
  } else {
    fOK &= false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate voltage data
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBatteryVoltage(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  boolean fOK = false;

  this->setCAN_ID(rqID_BMS, respID_BMS);
  items = this->Request_Diagnostics(rqBattVoltages_P1);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    CellVoltage.clear();
    this->ReadCellVoltage(data, 3, CELLCOUNT / 2);
    fOK = true;
  } else {
    fOK = false;
  }

  items = this->Request_Diagnostics(rqBattVoltages_P2);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadCellVoltage(data, 3, CELLCOUNT / 2);
    myBMS->Cvolts.min = CellVoltage.minimum(&myBMS->CV_min_at); //(int16_t *)
    myBMS->Cvolts.max = CellVoltage.maximum(&myBMS->CV_max_at);
    myBMS->Cvolts.mean = CellVoltage.mean();
    myBMS->Cvolts_stdev = CellVoltage.stddev();
    fOK &= true;
  } else {
    fOK &= false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief  Evaluate voltage data for distribution, calc. percentiles & outliners
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBatteryVoltageDist(BatteryDiag_t *myBMS) {
  byte _Count = CellVoltage.getCount();

  //Sort voltges in ascending order
  CellVoltage.bubble_sort();

  //Get quartiles
  myBMS->Cvolts.p25 = CellVoltage.percentile(_Count / 4);
  myBMS->Cvolts.median = CellVoltage.percentile(_Count / 2);
  myBMS->Cvolts.p75 = CellVoltage.percentile(_Count * 3 / 4);

  //Get outliners in the IQR-FACTOR range, excluding the min- and max-values
  uint16_t p3IQR = (myBMS->Cvolts.p75 - myBMS->Cvolts.p25) * IQR_FACTOR;
  byte p25_Out = 0;
  for (byte n = 1; n < (CellVoltage.getCount() / 4); n++) {
    if (CellVoltage.get(n) < ( myBMS->Cvolts.p25 - p3IQR)) {
      p25_Out++;
      //Serial.println(ave.get(n));
    }
  }
  byte p75_Out = 0;
  for (byte n = (CellVoltage.getCount() * 3 / 4); n < (CellVoltage.getCount() - 1); n++) {
    if (CellVoltage.get(n) > ( myBMS->Cvolts.p75 + p3IQR)) {
      p75_Out++;
      //Serial.println(ave.get(n));
    }
  }
  myBMS->Cvolts.p25_out_count = p25_Out;
  myBMS->Cvolts.p75_out_count = p75_Out;

  return true;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate actual battery state / data
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBatteryState(BatteryDiag_t *myBMS, boolean debug_verbose) {
  boolean fOK = false;
  uint16_t items;
  uint16_t value;

  this->setCAN_ID(rqID_BMS, respID_BMS);
  items = this->Request_Diagnostics(rqBattState);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 3, 1);
    myBMS->CV_Range.min = (uint16_t) value / 1024.0 * 1000.0;
    this->ReadDiagWord(&value, data, 5, 1);
    myBMS->CV_Range.max = (uint16_t) value / 1024.0 * 1000.0;
    myBMS->CV_Range.mean = (myBMS->CV_Range.max + myBMS->CV_Range.min) / 2;
    this->ReadDiagWord(&value, data, 7, 1);
    myBMS->BattLinkVoltage = value / 64.0;
    this->ReadDiagWord(&value, data, 9, 1);
    myBMS->BattCV_Sum = value / 64.0;
    this->ReadDiagWord(&value, data, 11, 1);
    myBMS->BattPower.voltage = value;
    this->ReadDiagWord(&value, data, 13, 1);
    myBMS->BattPower.current = value;
    myBMS->BattPower.power = myBMS->BattPower.voltage / 64.0 * myBMS->BattPower.current / 32.0 / 1000.0;

    myBMS->HVcontactState = data[15];
    myBMS->HV = myBMS->BattCV_Sum;
    myBMS->EVmode = data[24];
    myBMS->LV = data[25] / 8.0;

    myBMS->Amps = myBMS->BattPower.current;
    myBMS->Amps2 = myBMS->BattPower.current / 32.0;
    CalcPower(myBMS);
    fOK = true;
  } else {
    fOK = false;
  }

  this->setCAN_ID(rqID_EVC, respID_EVC);
  items = this->Request_Diagnostics(rqHV_Energy);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 3, 1);
    myBMS->Energy = value;
    fOK &= true;
  } else {
    fOK &= false;
  }

  items = this->Request_Diagnostics(rqSOC_EVC);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 3, 1);
    myBMS->SOC_EVC = value / 50.0;
    fOK &= true;
  } else {
    fOK &= false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate actual battery SOC and other core data values
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBatterySOC(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;
  bool fOK = false;

  this->setCAN_ID(rqID_BMS, respID_BMS);
  items = this->Request_Diagnostics(rqBattSOC);

  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    //this->ReadDiagWord(&value,data,3,1);
    //myBMS->BattOCV = value;;
    //Serial.println(myBMS->BattOCV / 100.0,3);
    this->ReadDiagWord(&value, data, 5, 1);
    myBMS->realSOC.min = value;
    this->ReadDiagWord(&value, data, 7, 1);
    myBMS->realSOC.max = value;
    //this->ReadDiagWord(&value,data,11,1);
    //myBMS->CapLoss = value;
    this->ReadDiagWord(&value, data, 13, 1);
    myBMS->CapInit = value;
    this->ReadDiagWord(&value, data, 15, 1);
    myBMS->CapEstimate = value;

    fOK = true;
  } else {
    fOK = false;
  }

  items = this->Request_Diagnostics(rqBattSOCrecal);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 3, 1);
    myBMS->realSOC.mean  = value;
    myBMS->SOCrecalState = data[2];
    fOK &= true;
  } else {
    fOK &= false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate actual battery limiting values
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBatteryLimits(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;

  this->setCAN_ID(rqID_BMS, respID_BMS);
  items = this->Request_Diagnostics(rqBattLimits);

  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myBMS->BattLimit.MaxChargeAmps = ((uint32_t) data[3] * 16777216 + (uint32_t) data[4] * 65535 + (uint16_t) data[5] * 256 + data[6]) / 1024.0;
    myBMS->BattLimit.MaxDischargeAmps = ((int32_t) ((uint32_t) data[7] * 16777216 + (uint32_t) data[8] * 65535 + (uint16_t) data[9] * 256 + data[10])) / 1024.0;
    this->ReadDiagWord(&value, data, 11, 1);
    myBMS->BattLimit.MaxCV = (uint16_t) value / 1024.0 * 1000.0;
    this->ReadDiagWord(&value, data, 13, 1);
    myBMS->BattLimit.MinCV = (uint16_t) value / 1024.0 * 1000.0;
    this->ReadDiagWord(&value, data, 15, 1);
    myBMS->SOC = (float) value / 50.0;
    this->ReadDiagWord(&value, data, 17, 1);
    myBMS->BattLimit.Zcharge = value;
    this->ReadDiagWord(&value, data, 19, 1);
    myBMS->BattLimit.Zdischarge = value;
    myBMS->BattLimit.BattPowerMax = data[21];
    myBMS->BattLimit.BattPowerGen = data[22];
    myBMS->BattLimit.BattPowerCharge = data[23];

    return true;
  } else {
    return false;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate battery health status
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getBatterySOH(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;

  this->setCAN_ID(rqID_BMS, respID_BMS);
  items = this->Request_Diagnostics(rqBattHealth);

  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myBMS->SOH = data[11]; //SOH value (x/2) done in PRN module
    this->ReadDiagWord(&value, data, 7, 1);
    if (value == 0xFFFF) { //Data filed longer in new BMS rev. -> warp for next entry
      this->ReadDiagWord(&value, data, 9, 1);
    }
    myBMS->CAPusable_max = value;
    //this->ReadDiagWord(&value,data,18,1);
    //myBMS->CAPusable = value;

    return true;
  } else {
    return false;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Read total distance / ODO from Dash
//! \brief   When accessing Dash get also ambient temperature
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getODOcount(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  uint16_t value = 0;
  bool fOK = false;

  this->setCAN_ID(rqID_DASH, respID_DASH);

  items = this->Request_Diagnostics(rqDashODO);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myBMS->ODO = (unsigned long) data[3] * 65536 + (uint16_t) data[4] * 256 + data[5];
    fOK = true;
  } else {
    fOK = false;
  }

  items = this->Request_Diagnostics(rqDashTemperature);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 5, 1);
    myBMS->AmbientTemp = (value - 400);
    fOK &= true;
  } else {
    fOK &= false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Read available Range / ECO from EVC
//! \brief   When accessing Dash get also ambient temperature
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getRange(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;
  uint16_t value = 0;
  bool fOK = false;

  this->setCAN_ID(rqID_EVC, respID_EVC);

  items = this->Request_Diagnostics(rqEV_Range);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 3, 1);
    myBMS->EVrange = value;
    fOK = true;
  } else {
    fOK = false;
  }

  rqMsg[3] = 0x46;
  items = this->Request_Diagnostics(rqMsg);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myBMS->EVeco = data[3];
    fOK &= true;
  } else {
    fOK &= false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate High Voltage contractor state
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getHVcontactorCount(BatteryDiag_t *myBMS, boolean debug_verbose) {

  uint16_t items;

  this->setCAN_ID(rqID_BMS, respID_BMS);
  items = this->Request_Diagnostics(rqBattHVContactorCycles);

  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myBMS->HVcontactCyclesLeft = (unsigned long) data[4] * 65536 + (uint16_t) data[5] * 256 + data[6];
    myBMS->HVcontactCyclesMax = (unsigned long) data[8] * 65536 + (uint16_t) data[9] * 256 + data[10];
    return true;
  } else {
    return false;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Get SOC calibration table based on OCV values
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::printOCVtable(BatteryDiag_t *myBMS, boolean debug_verbose) {
  (void) myBMS;

  uint16_t items;
  uint16_t value;
  bool fOK = false;

  this->setCAN_ID(rqID_BMS, respID_BMS);

  items = this->Request_Diagnostics(rqBattOCV_Cal);
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
    Serial.println(F("# ; mV"));
    for (byte n = 4; n <= (21 * 4); n += 4) {
      this->ReadDiagWord(&value, data, (n + 1), 1);
      if ((n / 4) < 10) Serial.print('0');
      Serial.print(n / 4);
      PRINT_CSV; Serial.println((uint16_t) (value / 4096.0 * 1000.0));
    }
    Serial.println();
    fOK = true;
  } else {
    fOK = false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Get cell resistance factors, used for capacity measurement
//! \brief   Values are directly printed to serial
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::printRESfactors(BatteryDiag_t *myBMS, boolean debug_verbose) {
  (void) myBMS;

  uint16_t items;
  byte offset = 0;
  uint16_t value;
  bool fOK = false;

  this->setCAN_ID(rqID_BMS, respID_BMS);

  items = this->Request_Diagnostics(rqBattCellResistance_P1);
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
    Serial.println(F("# ; Ri"));
    for (byte n = 1; n <= (49 * 2); n += 2) {
      this->ReadDiagWord(&value, data, (n + 2), 1);
      if ((n / 2) < 10) Serial.print('0');
      Serial.print(n / 2);
      PRINT_CSV; Serial.println((float) (value / 8192.0), 4);
      offset = n / 2;
    }
    fOK = true;
  } else {
    fOK = false;
  }

  items = this->Request_Diagnostics(rqBattCellResistance_P2);
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
    for (byte n = 2; n <= (48 * 2); n += 2) {
      this->ReadDiagWord(&value, data, (n + 1), 1);
      Serial.print(n / 2 + offset);
      PRINT_CSV; Serial.println((float) (value / 8192.0), 4);
    }
    Serial.println();
    fOK = true;
  } else {
    fOK = false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Get mission history data of BMS
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::printBMSlog(BatteryDiag_t *myBMS, boolean debug_verbose) {
  (void) myBMS;

  uint16_t items;
  bool fOK = false;

  this->setCAN_ID(rqID_BMS, respID_BMS);

  for (byte n = 0; n < 5; n++) {
    if (n == 0) {
      items = this->Request_Diagnostics(rqBattLogData_P1);
      if (items > 0) Serial.println(F("# ; Data"));
    } else {
      rqMsg[2] += 1;
      items = this->Request_Diagnostics(rqMsg);
    }
    if (items) {
      if (debug_verbose) {
        PrintReadBuffer(items);
      }
      if (n < 4) {
        this->printBMSlogSet(122);
      } else {
        this->printBMSlogSet(34);
      }
      fOK = true;
    } else {
      fOK = false;
    }
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Print function for mission history data
//! \param   length of data field (byte)
//--------------------------------------------------------------------------------
void canDiag::printBMSlogSet(byte _length) {
  for (byte n = 3; n < _length; n += 8) {
    if (n / 8 < 10) Serial.print('0');
    Serial.print(n / 8); PRINT_CSV;
    Serial.print(data[n] & 0x07); PRINT_CSV;          //Status Balancing?
    Serial.print(data[n + 1], DEC); PRINT_CSV;        //SOC %
    Serial.print(data[n + 2], DEC); PRINT_CSV;        //SOC %
    Serial.print(data[n + 3], HEX); PRINT_CSV;        //???
    Serial.print(data[n + 4], HEX); PRINT_CSV;        //???
    Serial.print((char) data[n + 5], DEC); PRINT_CSV; //Temperature Data degC?
    Serial.print((char) data[n + 6], DEC); PRINT_CSV; //Temperature Data degC?
    Serial.print(data[n + 7], HEX);                   //Status-Flags?
    Serial.println();
  }
}

//--------------------------------------------------------------------------------
//! \brief   Get Charging History Data
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::printCHGlog(boolean debug_verbose) {

  uint16_t items;
  bool fOK = false;

  //Structure for Log-Dataset
  typedef struct {
    byte chgFlag;
    byte chgStatus;
    uint16_t chgTime;
    uint16_t soc;
    uint16_t power;
    long odo;
    byte temp;
  } ChgLog_t;

  byte log_size = 10;
  ChgLog_t *ChgLog_P;

  //Reserve memory
  ChgLog_P = (ChgLog_t *) malloc(sizeof(ChgLog_t) * log_size);

  this->setCAN_ID(rqID_EVC, respID_EVC);

  for (byte n = 0; n < 8; n++) {
    if (n == 0) {
      items = this->Request_Diagnostics(rqEV_ChgLog_P1);
    } else if (n < 6) {
      rqMsg[3] += 1;
      items = this->Request_Diagnostics(rqMsg);
    } else if (n == 6) {
      rqMsg[2] = 0x34; rqMsg[3] = 0xE4;
      items = this->Request_Diagnostics(rqMsg);
    } else if (n == 7) {
      rqMsg[2] = 0x34; rqMsg[3] = 0x12;
      items = this->Request_Diagnostics(rqMsg);
    }
    if (items) {
      if (debug_verbose) {
        PrintReadBuffer(items);
      }
      for (byte i = 0; i < log_size; i++) {
        switch (n) {
          case 0:
            ChgLog_P[i].odo = data[4 + (i * 3)] * 65536 + data[5 + (i * 3)] * 256 + data[6 + (i * 3)];
            break;
          case 1:
            ChgLog_P[i].chgStatus = data[4 + i];
            break;
          case 2:
            ChgLog_P[i].chgFlag = data[4 + i];
            break;
          case 3:
            ChgLog_P[i].soc = data[4 + (i * 2)] * 256 + data[5 + (i * 2)];
            break;
          case 4:
            ChgLog_P[i].temp = data[4 + i];
            break;
          case 5:
            ChgLog_P[i].chgTime = data[4 + (i * 2)] * 256 + data[5 + (i * 2)];
            break;
          case 6:
            ChgLog_P[i].power = data[4 + (i * 2)] * 256 + data[5 + (i * 2)];
            break;
        }
      }
      fOK = true;
    } else {
      fOK = false;
    }
  }

  if (fOK) {
    Serial.println(F("#; km; kW; t/min; SOC%; degC; State; Flag"));
    for (byte i = 0; i < log_size; i++) {
      Serial.print(i); PRINT_CSV;
      Serial.print(ChgLog_P[i].odo); PRINT_CSV;
      Serial.print(ChgLog_P[i].power / 10.0, 1); PRINT_CSV;     
      Serial.print(ChgLog_P[i].chgTime); PRINT_CSV;
      Serial.print(ChgLog_P[i].soc / 5); PRINT_CSV;     
      Serial.print(ChgLog_P[i].temp - 40); PRINT_CSV;
      Serial.print(ChgLog_P[i].chgStatus); PRINT_CSV;
      Serial.println(ChgLog_P[i].chgFlag, HEX); 
    }
  }

  //Free memory
  free(ChgLog_P);

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Test if a OBL slow charger is installed by reading HW-Rev.
//! \param   enable verbose / debug output (boolean)
//! \return  report success (char) 7kW (true), 22kW (false), fail (-1)
//--------------------------------------------------------------------------------
int8_t canDiag::OBL_7KW_Installed(ChargerDiag_t *myOBL, boolean debug_verbose) {
  (void) myOBL;

  uint16_t items;

  this->setCAN_ID(rqID_OBL, respID_OBL);
  items = this->Request_Diagnostics(rqIDpart);
  
  if (items){
    if (debug_verbose) {
       PrintReadBuffer(items);
    }
    if (memcmp_P(data + 3, ID_7KW, 4*sizeof(char)) == 0) {
      return true;
    } else if (memcmp_P(data + 3, ID_22KW, 4*sizeof(char)) == 0) {
      return false;
    } else {
      return -1;
    }
  } else {
    return -1;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Get HW / SW revision (print directly to screen to save memory)
//! \brief   *** Use CAN_IDs set method before call ***
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::printECUrev(boolean debug_verbose, byte _type[]) {

  uint16_t items;
  bool fOK = false;

  items = this->Request_Diagnostics(rqPartNo);
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
    byte n = 3;
    byte revCount = 0;
    Serial.print(F("HW: "));
    do {
      if (n > 4 && (data[n] == _type[0] && data[n + 1] == _type[1] && data[n + 2] == _type[2])) {
        Serial.print(F(" SW: "));
        revCount++;
        /*if (revCount%2 == 0) {
          Serial.println();
          }*/
      }
      Serial.print((char)data[n]);
    } while ((data[++n] != 0x00) && (n < 23) && (n < items * 7));
    fOK = true;
  } else {
    fOK = false;
  }

  items = this->Request_Diagnostics(rqIDpart);
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
    byte n = 17;
    Serial.print(',');
    do {
      if (data[n] < 0x10) {
        Serial.print('0');
      }
      Serial.print(data[n], HEX);
    } while ((++n < 21) && (n < items * 7));
    Serial.println();
    
    n = 3;
    Serial.print(F("ID: "));
    do {
      if (data[n] < 0x20) {
        Serial.print(data[n], HEX);
      } else {
        Serial.print((char)data[n]);
      }
    } while ((++n < 17) && (n < items * 7));
    //Serial.println();
    fOK &= true;
  } else {
    fOK &= false;
  }

  rqMsg[2] = 0xF0;
  items = this->Request_Diagnostics(rqMsg);
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
    byte n = 3;
    Serial.print(',');
    do {
      if (data[n] < 0x20) {
        Serial.print(data[n], HEX);
      } else {
        Serial.print((char)data[n]);
      }
    } while ((++n < 17) && (n < items * 7));
    Serial.println();
    fOK &= true;
  } else {
    fOK &= false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Get the cooling method of the battery:
//! \brief   (with chiller option or only water cooled)
//! \return  report value (int8_t)
//--------------------------------------------------------------------------------
int8_t canDiag::getBattCoolingType(boolean debug_verbose) {
  uint16_t items;
  int8_t CoolingType = 0;

  this->setCAN_ID(rqID_EVC, respID_EVC);

  items = this->Request_Diagnostics(rqEV_BattCooling);
  
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
    CoolingType = data[3];
  }
  
  return CoolingType;
}

//--------------------------------------------------------------------------------
//! \brief   Get the heating method of the battery:
//! \brief   (water PTC installed?)
//! \return  report value (int8_t)
//--------------------------------------------------------------------------------
int8_t canDiag::getBattHeaterType(boolean debug_verbose) {
  uint16_t items;
  int8_t HeatingType = 0;

  this->setCAN_ID(rqID_EVC, respID_EVC);

  items = this->Request_Diagnostics(rqEV_BattHeating);
  
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
    HeatingType = data[3];
  }
  
  return HeatingType;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate charger temperatures (values - 40 in deg C)
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getChargerTemperature(ChargerDiag_t *myOBL, boolean debug_verbose) {

  uint16_t items;
  boolean fOK = false;

  this->setCAN_ID(rqID_OBL, respID_OBL);  //ECU address of OBL and JB2 are the same

  //OBL slow charger variant
  if (!FASTCHG) { 
    items = this->Request_Diagnostics(rqChargerTemperatures);
  
    if (items) {
      if (debug_verbose) {
        PrintReadBuffer(items);
      }
      myOBL->InTemp = data[3];
      myOBL->OutTemp = data[4];
      myOBL->InternalTemp = data[5];
      fOK = true;
    } else {
      fOK = false;
    }
  
    this->setCAN_ID(rqID_CHGCTRL, respID_CHGCTRL);
    items = this->Request_Diagnostics(rqCoolantTemp);
  
    if (items) {
      if (debug_verbose) {
        PrintReadBuffer(items);
      }
      myOBL->CoolantTemp = data[3];
      fOK &= true;
    } else {
      fOK &= false;
    }
  
  //JB2 fast charger variant
  } else {
    items = this->Request_Diagnostics(rqJB2Temp_SYS);
    if (items) {
      if (debug_verbose) {
        PrintReadBuffer(items);
      }
      myOBL->SysTemp = data[3];
      fOK = true;
    } else {
      fOK = false;
    }
    
    items = this->Request_Diagnostics(rqJB2Temp_HOT);
    if (items) {
      if (debug_verbose) {
        PrintReadBuffer(items);
      }
      myOBL->InternalTemp = data[3];
      fOK &= true;
    } else {
      fOK &= false;
    }
  
    items = this->Request_Diagnostics(rqJB2Temp_COOL);
    if (items) {
      if (debug_verbose) {
        PrintReadBuffer(items);
      }
      myOBL->CoolantTemp = data[3];
      fOK &= true;
    } else {
      fOK &= false;
    }
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate values from charger-controller for standard OBL
//! \brief   Setpoint for max current, mains coding of SC and cable, max avail.power
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getChargerCtrlValues(ChargerDiag_t *myOBL, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;
  bool fOK = false;

  //OBL slow charger variant
  if (!FASTCHG) { 
    this->setCAN_ID(rqID_CHGCTRL, respID_CHGCTRL);
    items = this->Request_Diagnostics(rqChargerSelCurrent);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      myOBL->Amps_setpoint = data[3] / 4; //Get data (x/4)
      fOK = true;
    } else {
      fOK = false;
    }
  
    items = this->Request_Diagnostics(rqMainsMaxCurrent);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      myOBL->AmpsChargingpoint = data [3];
      fOK &= true;
    } else {
      fOK &= false;
    }
  
    items = this->Request_Diagnostics(rqConnectorCoding);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      if (value < 1800) {
        myOBL->AmpsCableCode = value;
      } else {
        myOBL->AmpsCableCode = 0;
      }
      fOK &= true;
    } else {
      fOK &= false;
    }
  
    items = this->Request_Diagnostics(rqMaxPowerAvail);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->CHGpower[2] = value;
      fOK &= true;
    } else {
      fOK &= false;
    }
  
    items = this->Request_Diagnostics(rqChargerPilotState);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      myOBL->PilotState = data[3];
      fOK &= true;
    } else {
      fOK &= false;
    }
  
    this->setCAN_ID(rqID_OBL, respID_OBL);
    items = this->Request_Diagnostics(rqChargerState);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      myOBL->ChargerState = data[3];
      fOK &= true;
    } else {
      fOK &= false;
    }
  //JB2 fast charger variant
  } else {
    this->setCAN_ID(rqID_OBL, respID_OBL);
    items = this->Request_Diagnostics(rqJB2SelCurrent);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      myOBL->Amps_setpoint = data[3] / 4; //Get data (x/4)
      fOK = true;
    } else {
      fOK = false;
    }

    items = this->Request_Diagnostics(rqJB2Pilot_DUTY);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      if (data[3] / 2 <= 85) {
        myOBL->AmpsChargingpoint = (data [3] / 2.0) * 0.6;
      } else if (data[3] / 2 <= 97) {
        myOBL->AmpsChargingpoint = ((data [3] / 2.0) - 64) * 2.5;
      } else {
        myOBL->AmpsChargingpoint = 0;
      }
      fOK &= true;
    } else {
      fOK &= false;
    }

    items = this->Request_Diagnostics(rqJB2Pilot_V);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      byte pilotV = (data[3] / 4.0) - 15;
      if (pilotV >= 11) {
        myOBL->PilotState = 0;
      } else if (pilotV >= 8) {
        myOBL->PilotState = 1;
      } else if (pilotV >= 5) {
        myOBL->PilotState = 2;
      } else if (pilotV >= 2) {
        myOBL->PilotState = 3;
      } else {
        myOBL->PilotState = 4;
      }
      fOK &= true;
    } else {
      fOK &= false;
    }

    this->setCAN_ID(rqID_OBL, respID_OBL);
    items = this->Request_Diagnostics(rqJB2State);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      switch (data[3]) {
        case 1:
          myOBL->ChargerState = 0; //CHG  
          break;
        case 4:
          myOBL->ChargerState = 1; //ON  
          break;
        case 5:
          myOBL->ChargerState = 2; //STBY 
          break;
      }
      fOK &= true;
    } else {
      fOK &= false;
    }
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate charger voltages and currents
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getChargerDC(ChargerDiag_t *myOBL, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;
  bool fOK = false;  

  this->setCAN_ID(rqID_OBL, respID_OBL);

  //OBL slow charger variant
  if (!FASTCHG) {
    items = this->Request_Diagnostics(rqChargerDC);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 4, 1);
      if (value < 0xEA00) {  //OBL showing only valid data while charging
        myOBL->DC_HV = value;
      } else {
        myOBL->DC_HV = 0;
      }
      this->ReadDiagWord(&value, data, 10, 1);
      if (value < 0xEA00) {  //OBL showing only valid data while charging
        myOBL->DC_Current = value;
      } else {
        myOBL->DC_Current = 0;
      }
  
      items = this->Request_Diagnostics(rqChargerLV);
      if (items) {
        if (debug_verbose) {
          this->PrintReadBuffer(items);
        }
        this->ReadDiagWord(&value, data, 3, 1);
        myOBL->LV = value;
      }
  
      return true;
    } else {
      return false;
    }
  //JB2 fast charger variant
  } else {
    items = this->Request_Diagnostics(rqJB2DC_V);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->DC_HV = (value - 1023) * 10;
      fOK = true;
    } else {
      fOK = false;
    }

    items = this->Request_Diagnostics(rqJB2DC_A);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      if (value > 0x0C80) {  //OBL showing only valid data while charging
        myOBL->DC_Current = (value * 0.625) - 2000;
      } else {
        myOBL->DC_Current = 0;
      }
      fOK &= true;
    } else {
      fOK &= false;
    }

    items = this->Request_Diagnostics(rqJB2LV);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->LV = value;
      fOK &= true;
    } else {
      fOK &= false;
    }

    return fOK;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Read and evaluate charger amps (AC and DC currents)
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getChargerAC(ChargerDiag_t *myOBL, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;
  bool fOK = false;

  this->setCAN_ID(rqID_OBL, respID_OBL);

  //OBL slow charger variant
  if (!FASTCHG) {  
    items = this->Request_Diagnostics(rqChargerAC);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      //Get AC Currents (two rails from >= 20A, sum up for total current)
      this->ReadDiagWord(&value, data, 10, 1);
      if (value < 0xEA00) {  //OBL showing only valid data while charging
        myOBL->MainsAmps[0] = value;
      } else {
        myOBL->MainsAmps[0] = 0;
      }
      this->ReadDiagWord(&value, data, 12, 1);
      if (value < 0xEA00) {  //OBL showing only valid data while charging
        myOBL->MainsAmps[1] = value;
      } else {
        myOBL->MainsAmps[1] = 0;
      }
      myOBL->MainsAmps[2] = 0;
  
      //Get AC Voltages
      this->ReadDiagWord(&value, data, 4, 1);
      if (value < 0xEA00) {  //OBL showing only valid data while charging
        myOBL->MainsVoltage[0] = value;
  
      } else {
        myOBL->MainsVoltage[0] = 0;
      }
      myOBL->MainsVoltage[1] = 0; myOBL->MainsVoltage[2] = 0;
      if (myOBL->MainsAmps[0] > 0 || myOBL->MainsAmps[1] > 0) {
        myOBL->MainsFreq = data[15];
      } else {
        myOBL->MainsFreq = 0;
      }
  
      //Get AC Power
      this->ReadDiagWord(&value, data, 16, 1);
      if (value < 0xEA00) {  //OBL showing only valid data while charging
        myOBL->CHGpower[0] = value;
  
      } else {
        myOBL->CHGpower[0] = 0;
      }
      this->ReadDiagWord(&value, data, 18, 1);
      if (value < 0xEA00) {  //OBL showing only valid data while charging
        myOBL->CHGpower[1] = value;
  
      } else {
        myOBL->CHGpower[1] = 0;
      }
  
      return true;
    } else {
      return false;
    }
  //JB2 fast charger variant
  } else {
    items = this->Request_Diagnostics(rqJB2AC_Ph12_RMS_V);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->MainsVoltage[0] = value; //(x/2) done in PRN func.
      fOK = true;
    } else {
      fOK = false;
    }

    items = this->Request_Diagnostics(rqJB2AC_Ph23_RMS_V);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->MainsVoltage[1] = value; //(x/2) done in PRN func.
      fOK &= true;
    } else {
      fOK &= false;
    }

    items = this->Request_Diagnostics(rqJB2AC_Ph31_RMS_V);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->MainsVoltage[2] = value; //(x/2) done in PRN func.
      fOK &= true;
    } else {
      fOK &= false;
    }

    items = this->Request_Diagnostics(rqJB2AC_Ph1_RMS_A);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->MainsAmps[0] = (value * 0.625) - 2000; //(x/10) 
      fOK &= true;
    } else {
      fOK &= false;
    }

    items = this->Request_Diagnostics(rqJB2AC_Ph2_RMS_A);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->MainsAmps[1] = (value * 0.625) - 2000; //(x/10) 
      fOK &= true;
    } else {
      fOK &= false;
    }

    items = this->Request_Diagnostics(rqJB2AC_Ph3_RMS_A);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->MainsAmps[2] = (value * 0.625) - 2000; //(x/10) 
      fOK &= true;
    } else {
      fOK &= false;
    }

    items = this->Request_Diagnostics(rqJB2AC_Power);
    if (items) {
      if (debug_verbose) {
        this->PrintReadBuffer(items);
      }
      this->ReadDiagWord(&value, data, 3, 1);
      myOBL->CHGpower[0] = value - 20000; //Mains consumed power in W 
      fOK &= true;
    } else {
      fOK &= false;
    }

    return fOK;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Set charger ac_max current
//! \param   current only in the range of 6 to 20 A (byte)
//! \brief   >>> FOR TEST PURPOSE ONLY! DO NOT USE WHILE CHARGING <<<
//! \brief   >>> DO NOT USE ON US and UK cars, as they already use with 32A max<<< 
//! \brief   >>> with entering [set ac_max 20 -yes] you ACCEPT ALL CONSEQUENCES 
//! \brief   >>> AND THE USAGE IS SOLEY AT YOUR OWN RISK! <<<
//! \brief   >>> LOSS OF WARRANTY, DAMAGE(s), VIOLATION OF REGULATIVE RULES <<<
//! \brief   >>> NO LIABILITY FOR THIS SOFTWARE - SEE LICENSE STATEMENT! <<<
//--------------------------------------------------------------------------------
boolean canDiag::setACmax(ChargerDiag_t *myOBL, boolean debug_verbose) {

  myOBL->newAmps_setpoint = constrain(myOBL->newAmps_setpoint, 6, 20);
  //Serial.println(myOBL->newAmps_setpoint);

  byte _rqMsg[8] = {0x04, 0x2E, 0x61, 0x41, 0x50, 0x00, 0x00, 0x00};
  _rqMsg[4] = myOBL->newAmps_setpoint * 4;  //Define new AC max current

  boolean fOK = false;
  uint16_t items = 0;

  this->setCAN_ID(rqID_CHGCTRL, respID_CHGCTRL);

  items = Request_Diagnostics(rqTesterPresent);
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
  }

  delay(10);
  items = Request_Diagnostics(rqDiagSessionExt);
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
  }
  
  delay(10);  
  items = this->Request_Diagnostics(_rqMsg);
  if (items) {
    if (debug_verbose) {
      PrintReadBuffer(items);
    }
    if (data[0] == (_rqMsg[1] + 0x40) && data[1] == _rqMsg[2] && data[2] == _rqMsg[3]) {
      fOK = true;
    } else {
      fOK = false;
    }
  } else {
    fOK = false;
  }

  return fOK;
}


//--------------------------------------------------------------------------------
//! \brief   Read and evaluate data of the Tele-Communications Unit
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getTCUdata(TCUdiag_t *myTCU, boolean debug_verbose) {

  uint16_t items;
  uint16_t value;
  boolean fOK = false;
  uint32_t TimeData = 0;

  this->setCAN_ID(rqID_TCU, respID_TCU);

  items = this->Request_Diagnostics(rqTCUtime);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }

    TimeData = data[3] * 16777216 + data[4] * 65536 + data[5] * 256 + data[6];

    //Decode time data set
    myTCU->TCUtime.second = TimeData & 0x3F;
    TimeData = TimeData >> 6;
    myTCU->TCUtime.minute = TimeData & 0x3F;
    TimeData = TimeData >> 6; 
    myTCU->TCUtime.hour = TimeData & 0x1F;
    TimeData = TimeData >> 5;
    myTCU->TCUtime.day = TimeData & 0x1F;
    TimeData = TimeData >> 5;
    myTCU->TCUtime.month = TimeData & 0xF;
    TimeData = TimeData >> 4;
    myTCU->TCUtime.year = TimeData;

    fOK = true;
  } else {
    fOK = false;
  }

  items = this->Request_Diagnostics(rqTCUrssi);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myTCU->Rssi = data[3];
    fOK &= true;
  } else {
    fOK &= false;
  }

  items = this->Request_Diagnostics(rqTCUstate);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myTCU->State = data[3];
    fOK &= true;
  } else {
    fOK &= false;
  }

  items = this->Request_Diagnostics(rqTCUnetType);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    myTCU->NetType = data[3];
    fOK &= true;
  } else {
    fOK &= false;
  }

  items = this->Request_Diagnostics(rqTCUcounter);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    this->ReadDiagWord(&value, data, 5, 1);
    myTCU->Counter = value;
    fOK &= true;
  } else {
    fOK &= false;
  }

  //Correction for Bug in Ficosa TCU: missing trailing zero for 12:00 to 15:00 UTC
  if (myTCU->TCUtime.hour >= 24) {
    myTCU->TCUtime.hour -= 16;
    myTCU->TCUtime.day += 1;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Read and print network name TCU is connected with
//! \param   enable verbose / debug output (boolean)
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::getTCUnetwork(TCUdiag_t *myTCU, boolean debug_verbose) {
  (void) myTCU;

  uint16_t items;
  boolean fOK = false;

  this->setCAN_ID(rqID_TCU, respID_TCU);

  items = this->Request_Diagnostics(rqTCUnetName);
  if (items) {
    if (debug_verbose) {
      this->PrintReadBuffer(items);
    }
    byte n = 4;
    do {
      Serial.print((char)data[n]);
    } while ((data[++n] != 0x20) && (n < items * 7));
    Serial.println();
    fOK = true;
  } else {
    Serial.println('-');
    fOK = false;
  }

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Calculate power reading
//! \return  report success (boolean)
//--------------------------------------------------------------------------------
boolean canDiag::CalcPower(BatteryDiag_t *myBMS) {
  myBMS->Power = myBMS->HV * myBMS->Amps2 / 1000.0;
  return true;
}




//--------------------------------------------------------------------------------
//! \brief   Callback to set diagnostic parameters
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void set_cmd(uint8_t arg_cnt, char **args) {
  int8_t cmdOK = -1;
  if (arg_cnt == 3) {
    if (strcmp(args[1], "cap_mode") == 0) {
      myDevice.CapMeasMode = constrain((byte) cmdStr2Num(args[2], 10), 1, 2);
      DiagCAN.setCAPmode(myDevice.CapMeasMode);
      cmdOK = 1;
    } else if (strcmp(args[1], "cv") == 0 && strcmp(args[2], "on") == 0) {
      myDevice.verbose = 1;
      cmdOK = 1;
    } else if (strcmp(args[1], "cv") == 0 && strcmp(args[2], "off") == 0) {
      myDevice.verbose = 0;
      cmdOK = 1;
    } else if (strcmp(args[1], "ecu_list") == 0 && strcmp(args[2], "on") == 0) {
      myDevice.ecu_list = 1;
      cmdOK = 1;
    } else if (strcmp(args[1], "ecu_list") == 0 && strcmp(args[2], "off") == 0) {
      myDevice.ecu_list = 0;
      cmdOK = 1;
    }
  } else if (arg_cnt > 3) {
    //** >>> FOR TEST PURPOSE ONLY! DO NOT USE WHILE CHARGING! DO NOT USE ON US and UK cars, as they already work with 32A max<<< **
    //** >>> with entering [set ac_max 20 -yes] you ACCEPT ALL CONSEQUENCES AND THE USAGE IS SOLEY AT YOUR OWN RISK! <<< **
    //** >>> LOSS OF WARRANTY, DAMAGE(s), VIOLATION OF REGULATIVE RULES, NO LIABILITY FOR THIS SOFTWARE - SEE LICENSE STATEMENT! <<< **
    if (OBL.OBL7KW && (BMS.KeyState == 0) && strcmp(args[1], "ac_max") == 0 && strcmp(args[3], "-yes") == 0) {
      OBL.newAmps_setpoint = constrain((byte) cmdStr2Num(args[2], 10), 6, 20);
      if (DiagCAN.setACmax(&OBL, false)) {
        if (DiagCAN.getChargerCtrlValues(&OBL, false)) {
          if (OBL.Amps_setpoint == OBL.newAmps_setpoint) {
            Serial.print(F("OK: AC_max= ")); Serial.print(OBL.Amps_setpoint);
            Serial.println(F(" A"));
            cmdOK = 1;
          } else {
            cmdOK = 0;
          }
        } else {
          cmdOK = 0;
        }
      } else{
        cmdOK = 0;
      }
    }
  }
  if (cmdOK == 0) Serial.println(F("CMD: failed"));
  if (cmdOK < 0) Serial.println(F("CMD: wrong format"));

}

void init_cmd_prompt() {
  set_cmd_display("");            //reset command prompt to "CMD >>" 
}

//--------------------------------------------------------------------------------
//! \brief   Callback to activate main menu
//! \brief   reset command promt
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void main_menu (uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  myDevice.menu = MAIN;
  init_cmd_prompt();
}

//--------------------------------------------------------------------------------
//! \brief   Get length of __FlashStringHelper
//--------------------------------------------------------------------------------
size_t getLength(const __FlashStringHelper *ifsh)
{
  PGM_P p = reinterpret_cast<PGM_P>(ifsh);
  size_t n = 0;
  while (1) {
    unsigned char c = pgm_read_byte(p++);
    if (c == 0) break;
    n++;
  }
  return n;
}

//--------------------------------------------------------------------------------
//! \brief   Output a space-line for separation of datasets
//! \brief   with and without titel
//--------------------------------------------------------------------------------
void PrintSPACER() {
  for (byte i = 0; i < 41; i++) Serial.print('-');
  Serial.println();
}

void PrintSPACER(const __FlashStringHelper* titel) {
  size_t len = getLength(titel) + 4;
  byte start = 4; //(42 - len) / 6;

  Serial.println();
  for (byte i = 0; i <= 41; i++) {
    if (i == start) {
      Serial.print(F(">>")); Serial.print(titel); Serial.print(F("<<"));
      i += len;
    } else {
      Serial.print('-');
    }
  }
  Serial.println();
}


//--------------------------------------------------------------------------------
//! \brief   Output header data as welcome screen - wait for CAN-Bus to be ready
//--------------------------------------------------------------------------------
void printWelcomeScreen() {
  byte vLength = strlen(version);
  PrintSPACER();
  Serial.println(F("--- ED4 Smart CAN  --  EV Diagnostics ---"));
  Serial.print(F("--- v")); Serial.print(version);
  for (byte i = 0; i < (41 - 5 - vLength - 3); i++) {
    Serial.print(' ');
  }
  Serial.println(F("---"));
  PrintSPACER();

  Serial.println(F("Connect to OBD port - Waiting for CAN-Bus"));
  do {
    Serial.print('.');
    delay(1000);
  } while (digitalRead(MCP_INT));
  Serial.println(F("CONNECTED"));
  PrintSPACER();
}

//--------------------------------------------------------------------------------
//! \brief   Output header data
//--------------------------------------------------------------------------------
void printHeaderData() {
  Serial.print(F("ODO  : ")); Serial.print(BMS.ODO); Serial.print(F(" km, Amb.T: "));
  Serial.print(BMS.AmbientTemp / 10.0, 1); Serial.println(F(" degC"));
}

//--------------------------------------------------------------------------------
//! \brief   Output battery production data and battery status SOH flag
//--------------------------------------------------------------------------------
void printBatteryProductionData(boolean fRPT) {
  (void) fRPT;  // Avoid unused param warning
  
  Serial.print(F("SOH  : ")); Serial.print(BMS.SOH / 2.0, 1); Serial.print(F("%, "));
  if (BMS.fSOH == 0xFF) {
    Serial.println(MSG_OK);
  } else if (BMS.fSOH == 0) {
    Serial.println(F("FAULT"));
  } else {
    Serial.print(F("UNDEF")); Serial.print(F(", ")); Serial.println(BMS.fSOH, HEX);
  }
  Serial.print(F("Y/M/D: ")); Serial.print(2000 + BMS.ProdYear); Serial.print('/');
  Serial.print(BMS.ProdMonth); Serial.print('/'); Serial.println(BMS.ProdDay);
  int8_t CStype = DiagCAN.getBattCoolingType(false);
  Serial.print(F("BCS  : ")); Serial.print(BATTCOOL[CStype]);
  CStype = DiagCAN.getBattHeaterType(false);
  Serial.print(F(", BHS: ")); Serial.println(BATTHEAT[CStype]);
  
  byte type[3] = {0x37, 0x38, 0x39};  //set main descriptor for rev query
  DiagCAN.setCAN_ID(rqID_BMS, respID_BMS);
  DiagCAN.printECUrev(false, type);
}

//--------------------------------------------------------------------------------
//! \brief   Output standard dataset
//--------------------------------------------------------------------------------
void printStandardDataset() {
  Serial.print(F("SOC  : ")); Serial.print(BMS.SOC,1); Serial.print(F(", "));
  Serial.print(BMS.SOC_EVC,1); Serial.print(F(" % = "));
  Serial.print(BMS.Energy / 200.0, 2); Serial.print(F(" kWh = "));
  if (BMS.EVrange < 0x3FF) {
    Serial.print(BMS.EVrange);
  } else {
    Serial.print('-');
  }
  Serial.print(F(" km"));
  if (BMS.EVeco) {
    Serial.println(F(" ECO"));
  } else {
    Serial.println();
  }
  Serial.print(F("rSOC : ")); Serial.print((float) BMS.realSOC.min / 16.0, 1); Serial.print(F(", "));
  Serial.print(BMS.realSOC.mean / 16.0, 1); Serial.print(F(", ")); Serial.print(BMS.realSOC.max / 16.0, 1); Serial.println(F(" %"));
  Serial.print(F("HV   : ")); Serial.print(BMS.HV,1); Serial.print(F(" V, "));
  Serial.print((float) BMS.Amps2, 2); Serial.print(F(" A, "));
  if (BMS.Power != 0) {
    Serial.print((float) BMS.Power, 2);
  } else {
    Serial.print(F("0.00"));
  }
  Serial.println(F(" kW"));   
  Serial.print(F("LV   : ")); Serial.print(BMS.LV, 1); Serial.print(F(" V, "));
  Serial.print(BMS.LV_DCDC_amps / 10.0, 1); Serial.print(F(" A, "));
  Serial.print(BMS.LV_DCDC_power,1); Serial.print(F(" W, "));
  Serial.print(BMS.LV_DCDC_load / 256.0 * 100.0,0); Serial.println(F(" %"));
  Serial.print(F("EV   : "));
  Serial.print(ON_OFF[BMS.KeyState]); Serial.print(F(", ")); 
  if (BMS.EVmode <= 3 || BMS.EVmode == 5) {
    Serial.print(EVMODES[BMS.EVmode]);
  } else {
    Serial.print(BMS.EVmode, HEX);
  }
  Serial.print(F(", "));
  Serial.print(BMS.BattLimit.BattPowerMax / 2.0, 1); Serial.print(F(", "));
  Serial.print(BMS.BattLimit.BattPowerGen / 2.0, 1); Serial.print(F(", "));
  Serial.print(BMS.BattLimit.BattPowerCharge / 2.0, 1); Serial.println(F(" kW"));
}

//--------------------------------------------------------------------------------
//! \brief   Output BMS cell voltages
//--------------------------------------------------------------------------------
void printBMS_CellVoltages() {
  Serial.print(F("CV mean  : ")); Serial.print(BMS.CV_Range.mean); Serial.print(F(" mV"));
  Serial.print(F(", dV = ")); Serial.print(BMS.CV_Range.max - BMS.CV_Range.min); Serial.println(F(" mV"));
  Serial.print(F("CV min   : ")); Serial.print(BMS.CV_Range.min); Serial.print(F(" mV, LIM: "));
  Serial.print(BMS.BattLimit.MinCV); Serial.println(F(" mV"));
  Serial.print(F("CV max   : ")); Serial.print(BMS.CV_Range.max); Serial.print(F(" mV, LIM: "));
  Serial.print(BMS.BattLimit.MaxCV); Serial.println(F(" mV"));
  Serial.print(F("Ri in,out: ")); Serial.print(BMS.BattLimit.Zcharge); Serial.print(F(", "));
  Serial.print(BMS.BattLimit.Zdischarge); Serial.println(F(" mOhm"));
  Serial.print(F("Balancing: ")); Serial.print(BMS.BattBalState, HEX);
  Serial.print(F(", XOR: ")); Serial.println(BMS.BattBalXOR, HEX);
}

//--------------------------------------------------------------------------------
//! \brief   Output BMS capacity estimation
//--------------------------------------------------------------------------------
void printBMS_CapacityEstimate() {
  Serial.print(F("Measured : ")); Serial.print(BMS.CAP2_mean / 360.0, 3); Serial.print(F(", "));
  Serial.print(BMS.CapMeas / 360.0, 3); Serial.println(F(" (SOH), "));
  Serial.print(F("Estimate : ")); Serial.print(BMS.Cap_combined_quality,3); Serial.print(F(" of ")); 
  Serial.print(BMS.Cap_meas_quality,3); Serial.print(F(", "));
  Serial.print(BMS.LastMeas_days); Serial.println(F(" day(s)"));
  Serial.print(F("Capacity : ")); Serial.print(BMS.CapInit / 360.0, 3); Serial.print(F(" (LIM), "));
  Serial.print(BMS.CapEstimate / 360.0, 3); Serial.println(F(" (EST)"));
}

//--------------------------------------------------------------------------------
//! \brief   Output SOH state and capacity estimation
//--------------------------------------------------------------------------------
void printBMS_SOHstate() {
  Serial.print(F("SOH : ")); Serial.print(BMS.SOH / 2.0, 1); Serial.print(F(" %, "));
  Serial.print(BMS.CAPusable_max / 360.0, 3); Serial.println(F(" Ah"));
}

//--------------------------------------------------------------------------------
//! \brief   Output HV contactor state and DC isolation
//--------------------------------------------------------------------------------
void printHVcontactorState() {
  Serial.print(F("HV contr.: "));
  if (BMS.HVcontactState == 0x02) {
    Serial.println(F("ON"));
  } else if (BMS.HVcontactState == 0x00) {
    Serial.println(F("OFF"));
  }
  Serial.print(F("Cycles   : ")); Serial.print(BMS.HVcontactCyclesLeft);
  Serial.print(F(" of ")); Serial.println(BMS.HVcontactCyclesMax);
  Serial.print(F("DC iso.  : >")); Serial.print(BMS.Isolation); Serial.print(F(" kOhm, MF:"));
  Serial.println(BMS.DCfault, BIN); //Print Flag-Bits for isolation measurement
}

//--------------------------------------------------------------------------------
//! \brief   Output BMS temperatures
//--------------------------------------------------------------------------------
void printBMStemperatures() {
  Serial.print(F("mean: ")); Serial.print((float) BMS.Temps[2] / 64.0, 0);
  Serial.print(F(", min : ")); Serial.print((float) BMS.Temps[1] / 64.0, 0);
  Serial.print(F(", max : ")); Serial.println((float) BMS.Temps[0] / 64.0, 0);
  
  for (byte n = 3; n < 28; n = n + 9) {
    Serial.print(F("M ")); Serial.print((n / 9) + 1); Serial.print(F(": "));
    for (byte i = 0; i < 9; i++) {
      float temp = BMS.Temps[n + i] / 64.0;
      if (temp >= 0) Serial.print(' ');
      Serial.print((float) temp, 0);
      if ( i < 8) {
        Serial.print(',');
      } else {
        Serial.println();
      }
    }
  }
}

//--------------------------------------------------------------------------------
//! \brief   Output individual cell data and statistics
//--------------------------------------------------------------------------------
void printIndividualCellData() {
  Serial.print(F("# ;mV  ;As/10"));
  if (myDevice.CapMeasMode == 1) {
    Serial.println(F(" dSOC"));
  } else {
    Serial.println();
  }
  for(int16_t n = 0; n < CELLCOUNT; n++){
    if (n < 9) Serial.print('0');
    Serial.print(n+1); Serial.print(F(";")); Serial.print(DiagCAN.getCellVoltage(n));
    Serial.print(F(";")); Serial.print(DiagCAN.getCellCapacity(n) * BMS.CAP_factor);
    Serial.println();
  }
  PrintSPACER(F("Cell Statistics"));
  Serial.print(F("CV mean : ")); Serial.print(BMS.Cvolts.mean,0); Serial.print(F(" mV"));
  Serial.print(F(", dV= ")); Serial.print(BMS.Cvolts.max - BMS.Cvolts.min); Serial.print(F(" mV"));
  Serial.print(F(", s= ")); Serial.print(BMS.Cvolts_stdev); Serial.println(F(" mV"));
  Serial.print(F("CV min  : ")); Serial.print(BMS.Cvolts.min); Serial.print(F(" mV, # ")); Serial.println(BMS.CV_min_at + 1);
  Serial.print(F("CV max  : ")); Serial.print(BMS.Cvolts.max); Serial.print(F(" mV, # ")); Serial.println(BMS.CV_max_at + 1);
  PrintSPACER();
  Serial.print(F("CAP mean: ")); Serial.print(BMS.Ccap_As.mean, 0); Serial.print(F(" As/10, ")); Serial.print(BMS.Ccap_As.mean / 360.0,1); Serial.println(F(" Ah"));
  Serial.print(F("CAP min : ")); Serial.print(BMS.Ccap_As.min); Serial.print(F(" As/10, ")); Serial.print(BMS.Ccap_As.min / 360.0,1); Serial.print(F(" Ah, # ")); Serial.println(BMS.CAP_min_at + 1);
  Serial.print(F("CAP max : ")); Serial.print(BMS.Ccap_As.max); Serial.print(F(" As/10, ")); Serial.print(BMS.Ccap_As.max / 360.0,1); Serial.print(F(" Ah, # ")); Serial.println(BMS.CAP_max_at + 1);
}

//--------------------------------------------------------------------------------
//! \brief   Visualize voltage distribution of cell data and statistics
//--------------------------------------------------------------------------------
void printVoltageDistribution() {
  uint16_t CVmin = BMS.Cvolts.min;
  uint16_t CVmax = BMS.Cvolts.max;
  uint16_t CVp25 = BMS.Cvolts.p25;
  uint16_t CVp50 = BMS.Cvolts.median;
  uint16_t CVp75 = BMS.Cvolts.p75;
  uint16_t CVp3IQR = (BMS.Cvolts.p75 - BMS.Cvolts.p25) * IQR_FACTOR;
  
  byte bp_p25 = map(CVp25, CVmin, CVmax, 0, 40);
  byte bp_p50 = map(CVp50, CVmin, CVmax, 0, 40);
  byte bp_p75 = map(CVp75, CVmin, CVmax, 0, 40);
  byte bp_p3IQR_low = map(CVp25 - CVp3IQR, CVmin, CVmax, 0, 40);
  byte bp_p3IQR_high = map(CVp75 + CVp3IQR, CVmin, CVmax, 0, 40);
 
  Serial.print(F("Voltage Distribution (dV= ")); Serial.print(CVmax - CVmin); Serial.println(F(" mV)"));

  Serial.print('*');
  for (byte n = 1; n < 40; n++) {
    if (n < bp_p25) {
      if (n == bp_p3IQR_low) {
        Serial.print('>');
      } else {
        Serial.print('-');
      }
    } else if (n == bp_p25) {
      Serial.print('[');
    } else if (n < bp_p50) {
      Serial.print('=');
    } else if (n == bp_p50) {
      Serial.print('|');
    } else if (n > bp_p50 && n < bp_p75) {
      Serial.print('=');
    } else if (n == bp_p75) {
      Serial.print(']');
    } else {
      if (n == bp_p3IQR_high) {
        Serial.print('<');
      } else {
        Serial.print('-');
      }
    }
  }
  Serial.println('*');

  Serial.print(CVmin);
  Serial.print(F("   "));
  if (BMS.Cvolts.p25_out_count < 10) Serial.print('0');
  Serial.print(BMS.Cvolts.p25_out_count); Serial.print(F(" > "));
  Serial.print('[');
  Serial.print(CVp25); Serial.print(F("; "));
  Serial.print(CVp50); Serial.print(F("; "));
  Serial.print(CVp75); Serial.print(']');
  Serial.print(F(" < "));
  if (BMS.Cvolts.p75_out_count < 10) Serial.print('0');
  Serial.print(BMS.Cvolts.p75_out_count); Serial.print(' ');
  Serial.print(' '); Serial.print(CVmax);

  Serial.println();
  Serial.print(F("min"));
  for (byte n = 0; n <= 8; n++) Serial.print(' ');
  Serial.print(F("[p25; median; p75]"));
  for (byte n = 0; n <= 7; n++) Serial.print(' ');
  Serial.println(F("max"));
}

//--------------------------------------------------------------------------------
//! \brief   Output On-Board Charger voltages and currents AC and DC
//--------------------------------------------------------------------------------
void printOBL_Status() {
  Serial.print(F("EVSE status: ")); Serial.print(OBL.AmpsChargingpoint); Serial.print(F(" A, State: "));
  if (OBL.PilotState <= 4) {
    Serial.println(PILOT_STATE[OBL.PilotState]);
  } else {
    Serial.println(OBL.PilotState, HEX);
  }
  if (!FASTCHG) {
    Serial.print(F("Cable code : ")); 
    if (OBL.AmpsCableCode > 0) {
      Serial.print(OBL.AmpsCableCode); Serial.println(F(" Ohm"));
    } else {
      Serial.println('-');
    }
    Serial.print(F("Charger max: ")); Serial.print(OBL.Amps_setpoint); Serial.print(F(" A, State: "));
    if (OBL.ChargerState <= 4) {
      Serial.println(OBL_STATE [(OBL.ChargerState / 2)]);
    } else {
      Serial.println(OBL.ChargerState, HEX);
    }
    Serial.print(F("AC L1: ")); Serial.print(OBL.MainsVoltage[0] / 10.0, 1); Serial.print(F(" V, "));
    Serial.print((OBL.MainsAmps[0] + OBL.MainsAmps[1]) / 10.0, 1); Serial.print(F(" A, "));
    Serial.print(OBL.MainsFreq, 1); Serial.println(F(" Hz"));
    Serial.print(F("AC R1: ")); Serial.print(OBL.CHGpower[0] / 2000.0, 2); Serial.print(F(" kW, R2: "));
    Serial.print(OBL.CHGpower[1] / 2000.0, 2); Serial.print(F(" kW, max: "));
    Serial.print(OBL.CHGpower[2] / 64.0, 2); Serial.println(F(" kW"));
  } else {
    Serial.print(F("Charger max: ")); Serial.print(OBL.Amps_setpoint); Serial.print(F(" A, State: "));
    Serial.println(OBL_STATE[OBL.ChargerState]);
    Serial.print(F("AC L1-2-3: ")); Serial.print(OBL.MainsVoltage[0] / 2.0, 1); Serial.print(F(", "));
    Serial.print(OBL.MainsVoltage[1] / 2.0, 1); Serial.print(F(", "));
    Serial.print(OBL.MainsVoltage[2] / 2.0, 1); Serial.println(F(" V"));
    Serial.print(F("           ")); Serial.print(OBL.MainsAmps[0] / 10.0, 1); Serial.print(F(", "));
    Serial.print(OBL.MainsAmps[1] / 10.0, 1); Serial.print(F(", "));
    Serial.print(OBL.MainsAmps[2] / 10.0, 1); Serial.print(F(" A; "));
    Serial.print(OBL.CHGpower[0] / 1000.0, 3); Serial.println(F(" kW"));
  }
  Serial.print(F("DC HV: ")); Serial.print(OBL.DC_HV / 10.0, 1); Serial.print(F(" V, "));
  Serial.print(OBL.DC_Current / 10.0, 2); Serial.println(F(" A"));
  if (!FASTCHG) {
    Serial.print(F("DC LV: ")); Serial.print(OBL.LV / 100.0, 1); Serial.println(F(" V"));
  } else {
    Serial.print(F("DC LV: ")); Serial.print(OBL.LV / 64.0, 1); Serial.println(F(" V"));
  }
}

//--------------------------------------------------------------------------------
//! \brief   Output OBL charger temperatures
//--------------------------------------------------------------------------------
void printOBLtemperatures() {
  if (!FASTCHG) {
    Serial.print(F("In     : ")); 
    if (OBL.InTemp < 0xFF) {
      Serial.println(OBL.InTemp - TEMP_OFFSET, DEC);
    } else {
      Serial.println('-');
    }
  } else {
    Serial.print(F("System : ")); Serial.print(OBL.SysTemp, DEC); Serial.println(F(" %"));
  }
  if (!FASTCHG) {
    Serial.print(F("Intern.: ")); 
    if (OBL.InternalTemp < 0xFF) {
      Serial.println(OBL.InternalTemp - TEMP_OFFSET, DEC);
    } else {
      Serial.println('-');
    }
    Serial.print(F("Out    : ")); 
    if (OBL.OutTemp < 0xFF) {
      Serial.println(OBL.OutTemp - TEMP_OFFSET, DEC);
    } else {
      Serial.println('-');
    }
  } else {
    Serial.print(F("Hotspot: ")); 
    if (OBL.InternalTemp < 0xFF) {
      Serial.println(OBL.InternalTemp - TEMP_OFFSET + 10, DEC);
    }
  }
  Serial.print(F("Coolant: ")); 
  if (OBL.OutTemp < 0xFF) {
    Serial.println(OBL.CoolantTemp - TEMP_OFFSET + 10, DEC);
  } else {
    Serial.println('-');
  }
}

//--------------------------------------------------------------------------------
//! \brief   Output OBL charger HW/SW revisions
//--------------------------------------------------------------------------------
void printOBLrevision() {
  byte type[3] = {0x34,0x35,0x33};
  DiagCAN.setCAN_ID(rqID_OBL, respID_OBL);
  DiagCAN.printECUrev(false, type); //get HW, SW revisons and send to serial
}

//--------------------------------------------------------------------------------
//! \brief   Output TCU data
//--------------------------------------------------------------------------------
void printTCU_Status() {
  Serial.print(F("Date/Time : "));
  Serial.print(1990 + TCU.TCUtime.year); Serial.print('/'); Serial.print(TCU.TCUtime.month); Serial.print('/'); Serial.print(TCU.TCUtime.day); Serial.print(' ');
  if (TCU.TCUtime.hour <= 9) Serial.print('0'); Serial.print(TCU.TCUtime.hour); Serial.print(':');
  if (TCU.TCUtime.minute <= 9) Serial.print('0'); Serial.print(TCU.TCUtime.minute); Serial.print(':'); 
  if (TCU.TCUtime.second <= 9) Serial.print('0'); Serial.print(TCU.TCUtime.second); Serial.println(F(" UTC"));
  Serial.print(F("RSSI      : ")); Serial.print((TCU.Rssi * 2.0) - 111.0, 0); Serial.print(F(" dBm, "));
  Serial.print(((TCU.Rssi * 4.0)), 0); Serial.println('%');
  Serial.print(F("State     : ")); Serial.println(TCU_MODE [TCU.State]);
  Serial.print(F("NetType   : ")); 
  if (TCU.NetType > 0) {
    Serial.print(TCU.NetType + 1); Serial.println('G');
  } else {
    Serial.println(F("offline"));
  }
  Serial.print(F("Network   : ")); DiagCAN.getTCUnetwork(&TCU, false);
  Serial.print(F("SMS count : ")); Serial.println(TCU.Counter, DEC);
}

//--------------------------------------------------------------------------------
//! \brief   Output status data as splash screen
//--------------------------------------------------------------------------------
void printSplashScreen() {
  Serial.println(); PrintSPACER();
  printHeaderData();
  PrintSPACER();
  printStandardDataset();
  PrintSPACER();
  Serial.println(F("ENTER command (? for help)"));
}

//--------------------------------------------------------------------------------
//! \brief   Output BMS dataset
//--------------------------------------------------------------------------------
void printBMSdata() {
  Serial.println(MSG_OK);
  PrintSPACER(F("BMS Status"));
  printHeaderData();
  PrintSPACER();
  printStandardDataset();
  PrintSPACER();
  printBatteryProductionData(false);
  PrintSPACER();
  printBMS_CellVoltages();
  PrintSPACER();
  printHVcontactorState();
  PrintSPACER(F("Battery T/degC"));
  printBMStemperatures();
  PrintSPACER(F("Battery C/Ah"));
  printBMS_CapacityEstimate();
  PrintSPACER();
  if (myDevice.verbose) {
    printIndividualCellData();
    PrintSPACER();
  }
  if (BOXPLOT) {
    DiagCAN.getBatteryVoltageDist(&BMS);  //Sort cell voltages rising up and calc. quartiles
                                          //!!! after sorting track of individual cells is lost -> redo ".getBatteryVoltages" !!!
    printVoltageDistribution();           //Print statistic data as boxplot
    PrintSPACER();
  }
}

//--------------------------------------------------------------------------------
//! \brief   Output OBL dataset
//--------------------------------------------------------------------------------
void printOBLdata() {
  Serial.println(MSG_OK);
  PrintSPACER(F("OBC Status"));
  printOBL_Status();
  PrintSPACER(F("OBC T/degC"));
  printOBLtemperatures();
  PrintSPACER();
}

//--------------------------------------------------------------------------------
//! \brief   Output Cooling- and Subsystem dataset
//--------------------------------------------------------------------------------
void printTCUdata() {
  Serial.println(MSG_OK);
  PrintSPACER(F("TCU Status"));
  printTCU_Status();
  PrintSPACER();
}

//--------------------------------------------------------------------------------
//! \brief   Memory available between Heap and Stack
//--------------------------------------------------------------------------------
int getFreeRam () {
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

//--------------------------------------------------------------------------------
//! \brief   Read all queued charachers to clear input buffer.
//--------------------------------------------------------------------------------
void clearSerialBuffer() {
  do {                                            // Clear serial input buffer
      delay(10);
  } while (Serial.read() >= 0);
}

//--------------------------------------------------------------------------------
//! \brief   Callback to get temperature values depending on the active menu
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void get_temperatures (uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  
  switch (myDevice.menu) {
    case subBMS:
      if (DiagCAN.getBatteryTemperature(&BMS, false)){
        printBMStemperatures();
      }
      break;
    case subOBL:
      if (DiagCAN.getChargerTemperature(&OBL, false)){
        printOBLtemperatures();
      }
      break;
    case subTCU:
      break;
    case MAIN:
      break;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Callback to get OCV calibration table of BMS
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void get_OCVtable (uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  
  switch (myDevice.menu) {
    case subBMS:
        PrintSPACER(F("OCV Lookup"));
        DiagCAN.printOCVtable(&BMS, false);
      break;
    case subOBL:
    case subTCU:
    case MAIN:
      break;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Callback to get Cell Resistance factor of BMS
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void get_RESfactors (uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  
  switch (myDevice.menu) {
    case subBMS:
        PrintSPACER(F("Cell Resistance"));
        DiagCAN.printRESfactors(&BMS, false);
      break;
    case subOBL:
    case subTCU:
    case MAIN:
      break;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Callback to get Log data of BMS
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void get_BMSlog (uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  
  switch (myDevice.menu) {
    case subBMS:
        PrintSPACER(F("BMS Log"));
        DiagCAN.printBMSlog(&BMS, false);
      break;
    case subOBL:
    case subTCU:
    case MAIN:
      break;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Callback to get Charge-Log data of BMS
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void get_CHGlog (uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  
  switch (myDevice.menu) {
    case subBMS:
    case subOBL:
        PrintSPACER(F("OBC Log"));
        DiagCAN.printCHGlog(false);
      break;
    case subTCU:
    case MAIN:
      break;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Callback to show a help page depending on the active menu
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
#ifdef HELP
void help(uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  
  switch (myDevice.menu) {
    case MAIN:
      Serial.println(F("Main:"));
      Serial.println(F(" BMS, OBC, TCU"));
      Serial.println(F(" all"));
      Serial.println(F(" info show config"));
      Serial.println(F(" log  on|off [t]"));
      Serial.println(F(" set  edit config"));
      Serial.println(F(" #    EV status"));
      break;
    case subBMS:
      Serial.println(F("BMS:"));
      Serial.println(MSG_ALL);
      Serial.println(MSG_V);
      Serial.println(MSG_T);
      break;
    case subOBL:
      Serial.println(F("OBC:"));
      Serial.println(MSG_ALL);
      Serial.print(MSG_V); Serial.println(MSG_A_STATUS);
      Serial.println(MSG_T);
      break;
    case subTCU:
      Serial.println(F("TCU:"));
      Serial.println(MSG_ALL);
      break;
  }   
  if ( myDevice.menu != MAIN) Serial.println(MSG_BACK);
}
#endif


//--------------------------------------------------------------------------------
//! \brief   Callback to get all datasets depending on the active menu
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void show_info(uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  
  //Serial.print(F("Usable Memory: ")); Serial.println(getFreeRam());
  //Serial.print(F("Menu: ")); Serial.println(myDevice.menu);
  Serial.print(F("OBC     : ")); 
  if (OBL.OBL7KW > -1) {   
    if (OBL.OBL7KW ^ FASTCHG) { 
      Serial.print(OBL_ID [OBL.OBL7KW]);
      Serial.println(F(" inst."));
    } else {
      Serial.print(F("Recompile for "));
      Serial.println(OBL_ID [(!OBL.OBL7KW)]);
    }
  } else {
    Serial.println(FAILURE);
  }
  Serial.print(F("CV outp.: ")); Serial.println(ON_OFF [myDevice.verbose]);
  Serial.print(F("ECU_list: ")); Serial.println(ON_OFF [myDevice.ecu_list]);
  Serial.print(F("CAP_mode: ")); Serial.println(CAPMODES [(myDevice.CapMeasMode-1)]);
  Serial.print(F("Log dt/s: ")); Serial.print(myDevice.timer, DEC); Serial.print(F(", "));
  Serial.println(ON_OFF [myDevice.logging]);
  if (myDevice.ecu_list) {
    Serial.println();
    Serial.println(F("*BMS"));
    byte type[3] = {0x37, 0x38, 0x39};
    DiagCAN.setCAN_ID(rqID_BMS, respID_BMS);
    DiagCAN.printECUrev(false, type);
    Serial.println(F("*OBC"));
    type[0] = 0x34; type[1] = 0x35; type[2] = 0x33;
    DiagCAN.setCAN_ID(rqID_OBL, respID_OBL);
    DiagCAN.printECUrev(false, type);
    Serial.println(F("*OBC-CTRL"));
    DiagCAN.setCAN_ID(rqID_CHGCTRL, respID_CHGCTRL);
    DiagCAN.printECUrev(false, type);
    Serial.println(F("*TCU"));
    DiagCAN.setCAN_ID(rqID_TCU, respID_TCU);
    DiagCAN.printECUrev(false, type);
    Serial.println(F("*DASH"));
    DiagCAN.setCAN_ID(rqID_DASH, respID_DASH);
    DiagCAN.printECUrev(false, type);
    Serial.println(F("*EVC"));
    DiagCAN.setCAN_ID(rqID_EVC, respID_EVC);
    DiagCAN.printECUrev(false, type);
  }
}


//--------------------------------------------------------------------------------
//! \brief   Callback to start logging and / or set parameters
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void set_logging(uint8_t arg_cnt, char **args) {
  if (arg_cnt > 2) {
    myDevice.timer = (unsigned int) cmdStr2Num(args[2], 10);
  } 
  if (arg_cnt > 1) {
    if (strcmp(args[1], "on") == 0) {
      myDevice.logging = true;
      LOG_Timeout.Reset(myDevice.timer * 1000);
      myDevice.logCount = 0;
    }
    if (strcmp(args[1], "off") == 0) {
      myDevice.logging = false;
    }
  } else {
    if (arg_cnt == 1) {
      show_info(arg_cnt, args);
    }
  }
}

//--------------------------------------------------------------------------------
//! \brief   Get status for BMS relevant core data
//! \param   selected items for task (byte array), length of array
//--------------------------------------------------------------------------------
boolean getState_BMS(byte *selected, byte len) {
  boolean fOK = false;

  //Read CAN-messages
  byte testStep = 0;
  do {
    switch (selected[testStep]) {
      case BMSstate:
         fOK = DiagCAN.getBatteryState(&BMS, false);
         break;
      case BMSsoc:
         fOK = DiagCAN.getBatterySOC(&BMS, false);
         break;
      case BMSlimit:
         fOK = DiagCAN.getBatteryLimits(&BMS, false);
         break;
      case BMSbal:
         fOK = DiagCAN.getBalancingStatus(&BMS, false);
         break;
      case EVkey:
         fOK = DiagCAN.getKeyState(&BMS, false);
         break;
      case EVdcdc:
         fOK = DiagCAN.getDCDC_State(&BMS, false);
         break;
      case EVodo:
         fOK = DiagCAN.getODOcount(&BMS, false);
         break;
      case EVrange:
         fOK = DiagCAN.getRange(&BMS, false);
         break;
    }
    if (!myDevice.logging && testStep < BMSCOUNT) {
      if (fOK) {
        if (myDevice.progress) Serial.print(MSG_DOT);
      } else {
        Serial.print(MSG_FAIL);Serial.print('#'); Serial.print(selected[testStep]);
      }
    }
    testStep++;
  } while (testStep < len);
  //myDevice.progress = false;

  return fOK;
}

//--------------------------------------------------------------------------------
//! \brief   Get BMS datasets
//! \param   selected items for task (byte array), length of array
//--------------------------------------------------------------------------------
boolean getBMSdata(byte *selected, byte len) {
  boolean fOK = false;
  
  //Get diagnostics data
  DiagCAN.setCAN_ID(rqID_BMS, respID_BMS);

  byte testStep = 0;
  do {
    switch (selected[testStep]) {
      case 0:
         fOK = DiagCAN.getBatteryVoltage(&BMS, false);
         break;
      case 1:
         fOK = DiagCAN.getBatteryCapacity(&BMS, false);
         break;
      case 2:
         fOK = DiagCAN.getBalancingStatus(&BMS, false);
         break;
      case 5:
         fOK = DiagCAN.getBatterySOH(&BMS, false);
         break;
      case 6:
         fOK = DiagCAN.getBatteryDate(&BMS, false);
         break;
      case 8:
         fOK = DiagCAN.getBatteryTemperature(&BMS, false);
         break;
      case 9:
         fOK = DiagCAN.getHVcontactorCount(&BMS, false);
         break;
      case 10:
         fOK = DiagCAN.getODOcount(&BMS, false);
         break;
      case 11:
         fOK = DiagCAN.getIsolationValue(&BMS, false);
         break;
    }
    if (!myDevice.logging && testStep < 12) {
      if (fOK) {
        if (myDevice.progress) Serial.print(MSG_DOT);
      } else {
        Serial.print(MSG_FAIL);Serial.print('#'); Serial.print(selected[testStep]);
      }
    }
    testStep++;
  } while (fOK && testStep < len);
  
  return fOK;
}


//--------------------------------------------------------------------------------
//! \brief   Callback to show a splash screen for startup or by command
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void show_splash(uint8_t arg_cnt, char **args) {
  boolean fOK;
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  byte selected[] = {BMSstate, BMSsoc, BMSlimit, EVkey, EVdcdc, EVodo, EVrange};
  fOK = getState_BMS(selected, sizeof(selected)); 
  if (myDevice.progress & fOK) Serial.println(MSG_OK);
  printSplashScreen();
}

//--------------------------------------------------------------------------------
//! \brief   Callback to get SOH state of the battery
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void get_SOHstate (uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  
  byte bms_sel[] = {5};
  switch (myDevice.menu) {
    case subBMS:
      PrintSPACER(F("Battery SOH"));
      getBMSdata(bms_sel, sizeof(bms_sel));
      printBMS_SOHstate();
      break;
    case subOBL:
    case subTCU:
    case MAIN:
      break;
  }
}

//--------------------------------------------------------------------------------
//! \brief   Get OBL datasets
//--------------------------------------------------------------------------------
boolean getOBLdata() {
  boolean fOK = false;
  
  //Get diagnostics data
  DiagCAN.setCAN_ID(rqID_OBL, respID_OBL);

  byte testStep = 0;
  do {
    switch (testStep) {
      case OBLdc:
         fOK = DiagCAN.getChargerDC(&OBL, false);
         break;
      case OBLac:
         fOK = DiagCAN.getChargerAC(&OBL, false);
         break;
      case OBLctrl:
         fOK = DiagCAN.getChargerCtrlValues(&OBL, false);
         break;
      case OBLt:
         fOK = DiagCAN.getChargerTemperature(&OBL, false);
         break;
    }
    if (!myDevice.logging && testStep < OBLCOUNT) {
      if (fOK) {
        if (myDevice.progress) Serial.print(MSG_DOT);
      } else {
        Serial.print(MSG_FAIL);Serial.print('#'); Serial.print(testStep);
      }
    }
    fOK = true;
    testStep++;
  } while (fOK && testStep < OBLCOUNT);
  
  return fOK;
}


//--------------------------------------------------------------------------------
//! \brief   Callback to get voltages depending on the active menu
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void get_voltages (uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  byte bms_sel[] = {BMSstate, BMSlimit, BMSbal};
  
  switch (myDevice.menu) {
    case subBMS:
      getState_BMS(bms_sel, sizeof(bms_sel));
      Serial.println();
      printBMS_CellVoltages();
      break;
    case subOBL:
      if (getOBLdata()){
        Serial.println();
        printOBL_Status();
      }
      break;
    case subTCU:
      break;
    case MAIN:
      break;
  }
}


//--------------------------------------------------------------------------------
//! \brief   Get TCU datasets
//--------------------------------------------------------------------------------
boolean getTCUdata() {
  boolean fOK = false;
  
  //Get diagnostics data
  DiagCAN.setCAN_ID(rqID_TCU, respID_TCU);

  fOK = DiagCAN.getTCUdata(&TCU, false);   

  if (!myDevice.logging) {
    if (fOK) {
      if (myDevice.progress) Serial.print(MSG_DOT);
    } else {
      Serial.print(MSG_FAIL);Serial.print(F("#0"));
    }
  }
  return fOK;
}


//--------------------------------------------------------------------------------
//! \brief   Get all BMS data and output them
//! \brief   Dynamic memory allocation for CellVoltages and -Capacities
//! \brief   The allocated memory will be released after the data output
//--------------------------------------------------------------------------------
void printBMSall() {
  byte selected[12];   //hold list for selected tasks
  
  //Read all CAN-Bus IDs related to BMS
  for (byte i = 0; i < BMSCOUNT; i++) {
    selected[i] = i;
  }
  Serial.print(MSG_READ);
  myDevice.progress = true;
  getState_BMS(selected, BMSCOUNT);
  
  //Reserve memory
  DiagCAN.reserveMem_CellVoltage();
  DiagCAN.reserveMem_CellCapacity();
  
  //Get all diagnostics data of BMS
  for (byte i = 0; i < 12; i++) {
    selected[i] = i;
  }
  if (getBMSdata(selected, 12)) {
    printBMSdata();
  } else {
    Serial.println();
    Serial.println(FAILURE);
  }
  myDevice.progress = false;
    
  //Free allocated memory
  DiagCAN.freeMem_CellVoltage();
  DiagCAN.freeMem_CellCapacity();
}

//--------------------------------------------------------------------------------
//! \brief   Get all OBL data and output them
//--------------------------------------------------------------------------------
void printOBLall() {
  Serial.print(MSG_READ);
  if (getOBLdata()) {
    printOBLdata();
  } else {
    Serial.println();
    Serial.println(FAILURE);
  }
}

//--------------------------------------------------------------------------------
//! \brief   Get all Cooling- and Subsystem data and output them
//--------------------------------------------------------------------------------
void printTCUall() {
  Serial.print(MSG_READ);
  if (getTCUdata()) {
    printTCUdata();
  } else {
    Serial.println();
    Serial.println(FAILURE);
  }
}

//--------------------------------------------------------------------------------
//! \brief   Callback to get all datasets depending on the active menu
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void get_all (uint8_t arg_cnt, char **args) {
  (void) arg_cnt, (void) args;  // Avoid unused param warning
  
  switch (myDevice.menu) {
    case subBMS:
      printBMSall();
      break;
    case subOBL:
      printOBLall();
      break;
    case subTCU:
      printTCUall();
      break;
    case MAIN:
      printBMSall();
      printOBLall();
      printTCUall();
      break;
  }
}



//--------------------------------------------------------------------------------
//! \brief   Callback to switch to the BMS sub-menu and / or evaluate commands
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void bms_sub (uint8_t arg_cnt, char **args) {
  myDevice.menu = subBMS;
  set_cmd_display("BMS >>");
  if (arg_cnt == 2) {
    if (strcmp(args[1], "all") == 0) {
      get_all(arg_cnt, args);
    }
    if (strcmp(args[1], "t") == 0) {
      get_temperatures(arg_cnt, args);
    }
    if (strcmp(args[1], "v") == 0) {
      get_voltages(arg_cnt, args);
    }
    if (strcmp(args[1], "ocv") == 0) {
      get_OCVtable(arg_cnt, args);
    }
    if (strcmp(args[1], "res") == 0) {
      get_RESfactors(arg_cnt, args);
    }
    if (strcmp(args[1], "log") == 0) {
      get_BMSlog(arg_cnt, args);
    }
    if (strcmp(args[1], "soh") == 0) {
      get_SOHstate(arg_cnt, args);
    }  
  } else {

  }
}

//--------------------------------------------------------------------------------
//! \brief   Callback to switch to the OBL (Onboard Loader / std. charger)
//! \brief   sub-menu and / or evaluate commands
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void obl_sub (uint8_t arg_cnt, char **args) {
  myDevice.menu = subOBL;
  set_cmd_display("OBC >>");
  if (arg_cnt == 2) {
    if (strcmp(args[1], "all") == 0) {
      get_all(arg_cnt, args);
    }
    if (strcmp(args[1], "t") == 0) {
      get_temperatures(arg_cnt, args);
    }
    if (strcmp(args[1], "v") == 0) {
      get_voltages(arg_cnt, args);
    }
    if (strcmp(args[1], "log") == 0) {
      get_CHGlog(arg_cnt, args);
    }
  } else {

  }
}

//--------------------------------------------------------------------------------
//! \brief   Callback to switch to the Cooling sub-menu and / or evaluate commands
//! \param   Argument count (int) and argument-list (char*) from Cmd.h
//--------------------------------------------------------------------------------
void tcu_sub (uint8_t arg_cnt, char **args) {
  myDevice.menu = subTCU;
  set_cmd_display("TCU >>");
  if (arg_cnt == 2) {
    if (strcmp(args[1], "all") == 0) get_all(arg_cnt, args);
  } else {

  }
}

//--------------------------------------------------------------------------------
//! \brief   Logging data. Call queryfunctions and output the data
//--------------------------------------------------------------------------------
void logdata(){
  byte bms_sel[] = {BMSstate, BMSsoc, BMSlimit, BMSbal};
  getState_BMS(bms_sel, sizeof(bms_sel));
  getOBLdata();

  if (myDevice.logCount == 0) {
    //Print Header
    Serial.println();
    Serial.print(F("dt/s: "));Serial.println(myDevice.timer);
    Serial.print(F("#;SOC;rSOC,av;min;max;kWh;A;kW;V;Vc,min;max;Bal;"));
    if (!FASTCHG) {
      Serial.println(F("L1/V;L1/A;HV/V;HV/A;R1/kW;R2/kW;Ti/C;Tint/C;To/C;Tc/C"));
    } else {
      Serial.println(F("L1/V;L1/A;L2/V;L2/A;L3/V;L3/A;Pi/kW;HV/V;HV/A;Ts/%;Th/C;Tc/C"));
    }
  }
  myDevice.logCount++;
  //Print logged values
  Serial.print(myDevice.logCount); Serial.print(CSV);
  Serial.print(BMS.SOC,1); Serial.print(CSV);
  Serial.print((float) BMS.realSOC.mean / 16.0, 1); Serial.print(CSV);
  Serial.print((float) BMS.realSOC.min / 16.0, 1); Serial.print(CSV);
  Serial.print((float) BMS.realSOC.max / 16.0, 1); Serial.print(CSV);
  Serial.print(BMS.Energy / 200.0, 2); Serial.print(CSV);
  Serial.print((float) BMS.Amps2, 2); Serial.print(CSV);
  if (BMS.Power != 0) {
    Serial.print((float) BMS.Power, 2); Serial.print(CSV);
  } else {
    Serial.print(F("0.00")); Serial.print(CSV);
  }
  Serial.print(BMS.HV,1); Serial.print(CSV);
  Serial.print(BMS.CV_Range.min); Serial.print(CSV);
  Serial.print(BMS.CV_Range.max); Serial.print(CSV);
  Serial.print(BMS.BattBalState, HEX); Serial.print(CSV);
  if (!FASTCHG) {
    Serial.print(OBL.MainsVoltage[0] / 10.0, 1); Serial.print(CSV);
    Serial.print((OBL.MainsAmps[0] + OBL.MainsAmps[1]) / 10.0, 1); Serial.print(CSV);
    Serial.print(OBL.DC_HV / 10.0, 1); Serial.print(CSV);
    Serial.print(OBL.DC_Current / 10.0, 1); Serial.print(CSV);
    Serial.print(OBL.CHGpower[0] / 2000.0, 3); Serial.print(CSV);
    Serial.print(OBL.CHGpower[1] / 2000.0, 3); Serial.print(CSV);
    Serial.print(OBL.InTemp - TEMP_OFFSET, DEC); Serial.print(CSV);
    Serial.print(OBL.InternalTemp - TEMP_OFFSET, DEC); Serial.print(CSV);
    Serial.print(OBL.OutTemp - TEMP_OFFSET, DEC); Serial.print(CSV);
    Serial.print(OBL.CoolantTemp - TEMP_OFFSET + 10, DEC);
  } else {
    Serial.print(OBL.MainsVoltage[0] / 2.0, 1); Serial.print(CSV);
    Serial.print(OBL.MainsAmps[0] / 10.0, 1); Serial.print(CSV);
    Serial.print(OBL.MainsVoltage[1] / 2.0, 1); Serial.print(CSV);
    Serial.print(OBL.MainsAmps[1] / 10.0, 1); Serial.print(CSV);
    Serial.print(OBL.MainsVoltage[2] / 2.0, 1); Serial.print(CSV);
    Serial.print(OBL.MainsAmps[2] / 10.0, 1); Serial.print(CSV);
    Serial.print(OBL.CHGpower[0] / 1000.0, 3); Serial.print(CSV);
    Serial.print(OBL.DC_HV / 10.0, 1); Serial.print(CSV);
    Serial.print(OBL.DC_Current / 10.0, 2); Serial.print(CSV);
    Serial.print(OBL.SysTemp, DEC); Serial.print(CSV);
    Serial.print(OBL.InternalTemp - TEMP_OFFSET + 10, DEC); Serial.print(CSV);
    Serial.print(OBL.CoolantTemp - TEMP_OFFSET + 10, DEC);
  }
  Serial.println();
}


//--------------------------------------------------------------------------------
//! \brief   Setup menu items
//--------------------------------------------------------------------------------
void setupMenu() {
  cmdInit();

  if (HELP) {  
    cmdAdd("?", help);
  } 
  cmdAdd("..", main_menu);
  cmdAdd("#", show_splash);
  cmdAdd("t", get_temperatures);
  cmdAdd("v", get_voltages);
  cmdAdd("bms", bms_sub);
  cmdAdd("tcu", tcu_sub);
  if (OBL.OBL7KW ^ FASTCHG) cmdAdd("obc", obl_sub);
  cmdAdd("all", get_all);
  cmdAdd("log", set_logging);
  cmdAdd("info", show_info);
  cmdAdd("set", set_cmd);
}


//--------------------------------------------------------------------------------
//! \brief   SETUP()
//--------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial); // while the serial stream is not open, do nothing
  delay(500);

  //Initialize MCP2515 and clear filters 
  pinMode(CS, OUTPUT);
  DiagCAN.begin(&CAN0, &CAN_Timeout);
  DiagCAN.clearCAN_Filter();
  digitalWrite(CS, HIGH);

  //MCP2515 read buffer: setting pin 2 for input, LOW if CAN messages are received
  pinMode(MCP_INT, INPUT);

  //Serial.println(getFreeRam());

  //Print Welcome Screen and wait for CAN-Bus
  printWelcomeScreen();
  delay(1000);
  
  //Get basic BMS data and test installed charger
  DiagCAN.setCAPmode(CAP_MODE);
  OBL.OBL7KW = DiagCAN.OBL_7KW_Installed(&OBL, false);

  Serial.print(MSG_READ); myDevice.progress = true;
  show_splash(0, 0L); myDevice.progress = false;
  
  //Setup CLI, display prompt and local echo
  setupMenu();
  init_cmd_prompt();
  cmd_display();
  set_local_echo(ECHO);
  
}

//--------------------------------------------------------------------------------
//! \brief   LOOP()
//--------------------------------------------------------------------------------
void loop() {
   if (CLI_Timeout.Expired(true)) {
      cmdPoll();                                   //Poll CLI status
   }
   if (myDevice.logging && LOG_Timeout.Expired(true)){
      logdata();
   }
   //Serial.println(millis());
}
