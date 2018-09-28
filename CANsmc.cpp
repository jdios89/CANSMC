#include <Arduino.h>
#include <FlexCAN.h> 
#include "CANsmc.h"
#include <SMC66Registers.h>
#define pi12 1.5707963267949

CANsmc::CANsmc(FlexCAN* CANbus, Stream* port, uint8_t nodesid[4])
{
	// Attach serial port and canbus to class 
	 _serialport = port; 
	 _canport = CANbus; 
   _timeout = 500; 
  memcpy(nodesids, nodesid, sizeof(nodesids));
  _lt = micros(); 
  for(int i=0;i<4;i++) joints_velocity[i] = 0.0;
  offsets[0] = -pi12;
  offsets[1] = -pi12;
  offsets[2] = pi12;
  offsets[3] = pi12;
   
}

void CANsmc::setTimeOutCAN(int timeout)
{
  // is set in micros 
  _timeout = timeout; 
}

void CANsmc::checkTorque(uint8_t nodeid, bool dump) {
  float pertor = getActualTorquePercent(nodeid);
  //  Serial.println(pertor);
  if (pertor < 2) {
    //    Serial.println("zero torque");
    setPassiveMode(nodeid);
    if(dump)
      Serial.println("torquereset");

    setPositionMode(nodeid);
  }
}

void CANsmc::clearErrors(uint8_t nodeid, bool dump){
    uint32_t data = 0; 
    writeToRegister(nodeid, ERR_BITS, data);
    waitForReply(nodeid, ERR_BITS, dump); 
}

void CANsmc::setTorque(uint8_t nodeid, float current_t) {
  // receives the torque in mili amps
  float runncurrent = current_t; // * 1000;
  float percent = 0;
  /////
  if (abs(current_t) > 2.0) 
    checkTorque(nodeid, true);
  //////
  if (abs(runncurrent) > 2000){
    runncurrent = 2000;
    if (current_t < 0)
      runncurrent *= -1; 
  }
  setRunCurrent(nodeid, abs(runncurrent));
  int32_t encopos = getEncoderPosition(nodeid);
  int32_t psoll = encopos;
  if (abs(runncurrent) < 5.9) {
    percent = abs(runncurrent) * 348.89; //(100/5.87)*(2048/100)
    //int32_t start_velocity = 0x01; 
    //SMCCAN.writeToRegister(nodeid, V_START, start_velocity);
    //SMCCAN.waitForReply(nodeid, V_START, false); 
  }
  else {
    ////percent = 2048 * abs(runncurrent); // 2048 * 4; //3000
    if (abs(runncurrent)<20) percent = 5 * 2048 * abs(runncurrent); 
    else if (abs(runncurrent)<100) percent = 1.5 * 2048 * abs(runncurrent); 
    else percent = 716.8 * abs(runncurrent); // 2048/5
    ////percent = 3048 * runncurrent; 
    //int32_t start_velocity = 30000; 
    //SMCCAN.writeToRegister(nodeid, V_START, start_velocity);
    //SMCCAN.waitForReply(nodeid, V_START, false); 
  }

  if (runncurrent < 0)
    psoll -= percent;
  else
    psoll += percent;

  if (runncurrent == 0)
    psoll = encopos;

  writeToRegister(nodeid, P_SOLL, psoll);
  waitForReply(nodeid, P_SOLL, false);
}


void CANsmc::readVariables(){
  // also computes the joint speed of arms 
  _dt = (float(micros()) -float(_lt)) / 1000000.0; //dt in seconds
  // Serial.print(_dt);  
  // Serial.print(" ");
  // Serial.println(micros());
  // Serial.println(_lt);
  _lt = micros();
  
  for(int i=0; i<4; i++) {
    _last_encoder_pos[i] = encoder_pos[i];
    encoder_pos[i] = getEncoderPosition(nodesids[i]);
    follow_err[i] = getFollowError(nodesids[i]);
    actual_torque_percent[i] = getActualTorquePercent(nodesids[i]);
    run_currents[i] = getRunCurrent(nodesids[i]);
    // actual_velocity[i] = getActualVelocity(nodesids[i]);
    // encoder_velocity[i] = getEncoderVelocity(nodesids[i]);
    // Serial.println(encoder_pos[i]);
    // Serial.println(_last_encoder_pos[i]);
    
    joints_velocity[i] = float(encoder_pos[i] - _last_encoder_pos[i])/_dt; 
    joints_velocity[i] = joints_velocity[i] * COUNTS_TO_RAD;
    joints_velocity[i] = joints_velocity[i] / REDUCTION;
    joints_position[i] = float(encoder_pos[i]) * COUNTS_TO_RAD / REDUCTION;
    joints_position[i] = joints_position[i] - offsets[i];

  }
  
  encoder_posd0 = encoder_pos[0];
  encoder_posd1 = encoder_pos[1];
  encoder_posd2 = encoder_pos[2];
  encoder_posd3 = encoder_pos[3];
  
}

// Send a actual position read request
int32_t CANsmc::getPIST(uint8_t nodeid) {
  //returns the positions in counts
  int32_t actual_position = 0;
  readRequestFromRegister(nodeid, P_IST);
  waitForReplyInt32(nodeid, P_IST, &actual_position, false);
  return actual_position;
}
// ------------------------------------------------------------

float CANsmc::getActualTorquePercent(uint8_t nodeid) {
  // returns torque in percent of running current, needs to read running current as well
  // returns torque in Nm
  float percent = 0;
  uint32_t torque_percent = 0;
  readRequestFromRegister(nodeid, ACTUAL_TORQUE);
  waitForReplyuInt32(nodeid, ACTUAL_TORQUE, &torque_percent, false);
  percent = (torque_percent * COUNT_TO_PERCENT ) * 100;
  return percent;
}

// ------------------------------------------------------------
float CANsmc::getTorque(uint8_t nodeid) {
  // returns torque in Nm
  float actual_current = 0.0;
  float torque = 0.0;
  torque = getActualTorquePercent(nodeid);
  uint32_t runcu = 0;
  runcu = getRunCurrent(nodeid);
  //  r_current = read_runcurrent();
  actual_current = (torque * runcu * current_C) / 1000; //divide over 1000 cause result is in mA, we want in A
  torque = actual_current * AMPS_TO_TORQUE;
  return torque;
}
// ------------------------------------------------------------
// Send a encoder position read request
int32_t CANsmc::getEncoderPosition(uint8_t nodeid) {
  //returns encoder position in counts
  int32_t enc_position = 0;
  readRequestFromRegister(nodeid, ENCODER_POS);
  // Serial.println("waiting for message");
  // ttiming = micros();
  waitForReplyInt32(nodeid, ENCODER_POS, &enc_position, false);
  return enc_position;
}

// Send a actual velocity read request
int32_t CANsmc::getActualVelocity(uint8_t nodeid) {
  //returns encoder position in counts
  int32_t actual_velocity = 0;
  readRequestFromRegister(nodeid, V_IST);
  // Serial.println("waiting for message");
  // ttiming = micros();
  waitForReplyInt32(nodeid, V_IST, &actual_velocity, false);
  return actual_velocity;
}
// Send a actual velocity read request
int32_t CANsmc::getEncoderVelocity(uint8_t nodeid) {
  //returns encoder position in counts
  int32_t actual_velocity = 0;
  readRequestFromRegister(nodeid, V_ENCODER);
  // Serial.println("waiting for message");
  // ttiming = micros();
  waitForReplyInt32(nodeid, V_ENCODER, &actual_velocity, false);
  return actual_velocity;
}

// ------------------------------------------------------------
float countsToRad(int32_t counts) {
  return counts * COUNTS_TO_RAD;
}


float CANsmc::getVSOLL(uint8_t nodeid) {
  // returns the velocity in rpm
  int32_t current_velocity = 0;
  readRequestFromRegister(nodeid, V_SOLL);
  waitForReplyInt32(nodeid, V_SOLL, &current_velocity, false);
  return current_velocity * VEL_UNITS;
}

//-------------------------------------------------------------
// Send a velocity read request
float CANsmc::getVelEnc(uint8_t nodeid) {
  // returns the velocity in rpm
  int32_t current_velocity = 0;
  readRequestFromRegister(nodeid, V_ENCODER);
  waitForReplyInt32(nodeid, V_ENCODER, &current_velocity, false);
  return (float) (current_velocity * VEL_UNITS);
}
// ------------------------------------------------------------
// Send a velocity read request
float CANsmc::getVIST(uint8_t nodeid) {
  // returns the velocity in rpm
  int32_t current_velocity = 0;
  readRequestFromRegister(nodeid, V_IST);
  waitForReplyInt32(nodeid, V_IST, &current_velocity, false);
  return (float)(current_velocity * VEL_UNITS);
}

void CANsmc::setStandbyCurrent(uint8_t nodeid, float current) {
  // Set the current in miliamperes, use 2000 for exampel
  // current = current * .3;
  uint32_t current_int = 0;
  if (current < 0) {
    current = -current;
  }
  if (current > 2000) {
    current = 2000;
  }
  current_int = (current / current_C);
  writeToRegister(nodeid, STANDBY_CURRENT, current_int);
  waitForReply(nodeid, STANDBY_CURRENT, false);
}
// ------------------------------------------------------------
// Send a standbycurrent read request
float CANsmc::getStandbyCurrent(uint8_t nodeid) {
  // returns the velocity in mA
  uint32_t current = 0;
  readRequestFromRegister(nodeid, STANDBY_CURRENT);
  waitForReplyuInt32(nodeid, STANDBY_CURRENT, &current, false);
  return current * current_C;
}
// Function to set running current in mA
void CANsmc::setRunCurrent(uint8_t nodeid, float current) {
  //  Serial.println(current);
  // Set the current in miliamperes, use 2000 for exampel
  uint32_t current_int = 0;
  CAN_message_t inMsg;
  if (current < 0) {
    current = -current;
  }
  if (current > 2000) {
    current = 2000;
  }
  current_int = current * C_current;
  writeToRegister(nodeid, RUN_CURRENT, current_int);
  waitForReply(nodeid, RUN_CURRENT, false);
}
// ------------------------------------------------------------

uint32_t CANsmc::getRunCurrent(uint8_t nodeid) {
  uint32_t run_current = 0;
  CAN_message_t inMsg;
  readRequestFromRegister(nodeid, RUN_CURRENT);
  waitForReplyuInt32(nodeid, RUN_CURRENT, &run_current, false);
  return run_current;
}

int32_t CANsmc::getFollowError(uint8_t nodeid) {
  //returns encoder position in counts
  int32_t fllerror = 0;
  readRequestFromRegister(nodeid, FLWERR);
  waitForReplyInt32(nodeid, FLWERR, &fllerror, false);
  return fllerror;
}

void CANsmc::setPSOLL(uint8_t nodeid, int32_t psoll) {
  writeToRegister(nodeid, P_SOLL, psoll);
  waitForReply(nodeid, P_SOLL, false);
}

void CANsmc::setVelocityInt32(uint8_t nodeid, int32_t velocity) {
  // each unit is 0.01 rpm
  writeToRegister(nodeid, V_SOLL, velocity);
  waitForReply(nodeid, V_SOLL, false);
}
void CANsmc::setAsoll(uint8_t nodeid, int32_t asoll) {

  writeToRegister(nodeid, A_SOLL, asoll);
  waitForReply(nodeid, A_SOLL, true);
}

void CANsmc::setVelocityMode(uint8_t nodeid) {
  uint32_t veloccity_mode = 0x00000001;
  writeToRegister(nodeid, MODE_REG, veloccity_mode);
  waitForReply(nodeid, MODE_REG, false);
}

void CANsmc::setPositionMode(uint8_t nodeid) {
  uint32_t possition_mode = 0x00000002;
  writeToRegister(nodeid, MODE_REG, possition_mode);
  waitForReply(nodeid, MODE_REG, true);
}

void CANsmc::setPassiveMode(uint8_t nodeid) {
  uint32_t passsive_mode = 0;
  writeToRegister(nodeid, MODE_REG, passsive_mode);
  waitForReply(nodeid, MODE_REG, false);
}

void CANsmc::wrMsg(
  uint32_t  id, uint8_t len, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, 
  uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
  CAN_message_t msg;
  for (int i = 0; i < 7; i++)
    msg.buf[i] = 0;

  msg.id = id;
  msg.len = len;
  msg.ext = 0;

  msg.buf[0] = d0;
  msg.buf[1] = d1;
  msg.buf[2] = d2;
  msg.buf[3] = d3;
  msg.buf[4] = d4;
  msg.buf[5] = d5;
  msg.buf[6] = d6;
  msg.buf[7] = d7;

  _canport->write(msg);
}


bool CANsmc::waitForReply(uint8_t nodeid, uint8_t subindex, bool dump) {
  //   Function to be called after write to register. Will wait for a
  //  reply from the service called, return wether it was success or not 
  bool replied = false;
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  CAN_message_t inMsg;
  //_timetotimeout = micros();
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump) {
        hexDumpAll(inMsg, _serialport);
      }
      if (/*inMsg.id == id && */inMsg.buf[3] == subindex) {
        replied = true;
      }
    }
  }
  return replied; 
}

bool CANsmc::waitForReplyInt32(uint8_t nodeid, uint8_t subindex, int32_t* number32, bool dump) {
  //  Function to be called after writeToRegister
  // will return a register value of int32
  bool replied = false;
  //uint32_t id = nodeid + WRITE_REQUEST_CAN;
  int32_t value = 0;
  CAN_message_t inMsg;
  //_timetotimeout = micros(); 
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump)
        hexDumpAll(inMsg, _serialport);

      if (inMsg.buf[0] != CAN_ERROR_RESPONSE) {
        //int byte_num = numByte(inMsg.buf[0]);
        if (/*inMsg.id == id && */inMsg.buf[3] == subindex) {
          reconstructInt32(&value, inMsg);
          replied = true;
          // Serial.println("Received position");
        }
      }
    }
  }
  *number32 = value; 
  return replied;
}

bool CANsmc::waitForReplyuInt32(uint8_t nodeid, uint8_t subindex, uint32_t* numberu32, bool dump) {
  //  Function to be called after writeToRegister
  // will return a register value of uint32
  bool replied = false;
  uint32_t value = 0;
  //uint32_t id = nodeid + WRITE_REQUEST_CAN;
  CAN_message_t inMsg;
  //_timetotimeout = micros(); 
  while (!replied /*|| !(micros() - _timetotimeout > _timeout)*/) {
    while (_canport->available()) {
      //Clear out the canbus
      _canport->read(inMsg);
      if (dump)
        hexDumpAll(inMsg, _serialport);

      if (inMsg.buf[0] != CAN_ERROR_RESPONSE) {
        //int byte_num = numByte(inMsg.buf[0]);
        if (/*inMsg.id == id &&*/ inMsg.buf[3] == subindex) {
          reconstructUint32(&value, inMsg);
          replied = true;
          // Serial.println("Received position");
        }
      }
    }
  }
  *numberu32 = value; 
  return replied;
}

// General function to write to register format uint32_t
void  CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, uint32_t datas) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(object_index_32); //register number for 4 bytes
  uint8_t d2 = highByte(object_index_32);
  uint8_t d3 = subindex;

  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
//----------------------------------------------------
// General function to write to register format int32_t
void CANsmc::writeToRegister(uint8_t nodeid, uint8_t subindex, int32_t datas) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(object_index_32); //register number for 4 bytes
  uint8_t d2 = highByte(object_index_32);
  uint8_t d3 = subindex;
  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

// send a read request
void CANsmc::readRequestFromRegister(uint8_t nodeid, uint8_t subindex) {

  uint32_t comobject_id = WRITE_REQUEST_CAN; // This is the R_SDO object id
  uint32_t id = nodeid + comobject_id;
  uint8_t len = 8;
  uint8_t d0 = READ_REQUEST_CAN; //read request
  uint8_t d1 = lowByte(object_index_32);
  uint8_t d2 = highByte(object_index_32); //reguister 2012h in little endian
  uint8_t d3 = subindex; //subindex
  uint8_t d4 = 0; //filling message
  uint8_t d5 = 0;
  uint8_t d6 = 0;
  uint8_t d7 = 0;
  wrMsg(id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

int numByte(uint8_t code) {
  int number = 0;
  switch (code) {
    case CANREAD_1BYTE:
      number = 1;
      break;
    case CANREAD_2BYTE:
      number = 2;
      break;
    case CANREAD_3BYTE:
      number = 3;
      break;
    case CANREAD_4BYTE:
      number = 4;
      break;
  }
  return number;
}


void hexDump(uint8_t dumpLen, uint8_t *bytePtr, Stream* port) {
  uint8_t working;
  uint8_t hex[17] = "0123456789abcdef";
  while ( dumpLen-- ) {
    working = *bytePtr++;
    port->write( hex[ working >> 4 ] );
    port->write( hex[ working & 15 ] );
  }
  port->write('\r');
  port->write('\n');
}

// -------------------------------------------------------------
void hexDumpAll(CAN_message_t msg, Stream* port) {
  port->print(" ID: ");  writeID(msg.id, port);
  port->print(" data: "); hexDump(8, msg.buf, port);
  port->print("  len: "); port->print(msg.len); port->print(" ");
  port->print(" ext: "); port->println(msg.ext);
}
// -------------------------------------------------------------
void writeID(uint32_t id, Stream* port) {
  port->print("0x");
  if (id <= 0xfff)
    port->print("0");
  if (id <= 0xff)
    port->print("0");
  if (id <= 0xf)
    port->print("0");
  port->print(id, HEX);
}

void wrMsg(FlexCAN* CANbus, uint32_t  id, uint8_t len, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7) {
  CAN_message_t msg;
  for (int i = 0; i < 7; i++)
    msg.buf[i] = 0;

  msg.id = id;
  msg.len = len;
  msg.ext = 0;

  msg.buf[0] = d0;
  msg.buf[1] = d1;
  msg.buf[2] = d2;
  msg.buf[3] = d3;
  msg.buf[4] = d4;
  msg.buf[5] = d5;
  msg.buf[6] = d6;
  msg.buf[7] = d7;

  CANbus->write(msg);
}

// General function to write to register format uint32_t
void  writeToRegister(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, uint32_t datas) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(object_index_32); //register number for 4 bytes
  uint8_t d2 = highByte(object_index_32);
  uint8_t d3 = subindex;

  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(CANbus, id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}
//----------------------------------------------------
// General function to write to register format int32_t
void writeToRegister(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, int32_t datas) {
  uint32_t id = nodeid + WRITE_REQUEST_CAN;
  uint8_t len = 8;
  uint8_t d0 = CANWRITE_4BYTE;
  uint8_t d1 = lowByte(object_index_32); //register number for 4 bytes
  uint8_t d2 = highByte(object_index_32);
  uint8_t d3 = subindex;
  uint8_t d4 = datas & 0xFF;
  uint8_t d5 = (datas >> 8) & 0xFF;
  uint8_t d6 = (datas >> 16) & 0xFF;
  uint8_t d7 = (datas >> 24) & 0xFF; //MSB , little endian format
  wrMsg(CANbus, id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

// send a read request
void readRequestFromRegister(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex) {

  uint32_t comobject_id = WRITE_REQUEST_CAN; // This is the R_SDO object id
  uint32_t id = nodeid + comobject_id;
  uint8_t len = 8;
  uint8_t d0 = READ_REQUEST_CAN; //read request
  uint8_t d1 = lowByte(object_index_32);
  uint8_t d2 = highByte(object_index_32); //reguister 2012h in little endian
  uint8_t d3 = subindex; //subindex
  uint8_t d4 = 0; //filling message
  uint8_t d5 = 0;
  uint8_t d6 = 0;
  uint8_t d7 = 0;
  wrMsg(CANbus, id, len, d0, d1, d2, d3, d4, d5, d6, d7);
}

void reconstructInt32(int32_t* number32, CAN_message_t inMsg) {
  // Reconstruct from the last 4 bytes of canmessage, little endian format
  // to a signed integer of 32 bits
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[4], i)) {
      bitSet(*number32, i);
    }
    else {
      bitClear(*number32, i);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[5], i)) {
      bitSet(*number32, i + 8);
    }
    else {
      bitClear(*number32, i + 8);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[6], i)) {
      bitSet(*number32, i + 16);
    }
    else {
      bitClear(*number32, i + 16);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[7], i)) {
      bitSet(*number32, i + 24);
    }
    else {
      bitClear(*number32, i + 24);
    }
  }
}
// ------------------------------------------------------------
void reconstructUint32(uint32_t* number32, CAN_message_t inMsg) {
  // Reconstruct from the last 4 bytes of canmessage, little endian format
  // to a signed integer of 32 bits
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[4], i)) {
      bitSet(*number32, i);
    }
    else {
      bitClear(*number32, i);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[5], i)) {
      bitSet(*number32, i + 8);
    }
    else {
      bitClear(*number32, i + 8);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[6], i)) {
      bitSet(*number32, i + 16);
    }
    else {
      bitClear(*number32, i + 16);
    }
  }
  for (int i = 0; i < 8; i++ ) {
    if (bitRead(inMsg.buf[7], i)) {
      bitSet(*number32, i + 24);
    }
    else {
      bitClear(*number32, i + 24);
    }
  }
}

