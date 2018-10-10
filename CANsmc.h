#ifndef CANsmc_h
#define CANsmc_h
#include <SMC66Registers.h>
#include <FlexCAN.h>
#include <Arduino.h>

void wrMsg(
	FlexCAN* CANbus, uint32_t  id, uint8_t len, uint8_t d0, uint8_t d1, 
	uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
void writeToRegister(
	FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, uint32_t datas);
void writeToRegister(
	FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, int32_t datas);
void readRequestFromRegister(
	FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex);
void reconstructInt32(int32_t* number32, CAN_message_t inMsg);
void reconstructUint32(uint32_t* number32, CAN_message_t inMsg);
void hexDump(uint8_t dumpLen, uint8_t *bytePtr, Stream* port);
void hexDumpAll(CAN_message_t msg, Stream* port);
void writeID(uint32_t id, Stream* port);
int numByte(uint8_t code);
/* void waitForReply(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, bool dump);
int32_t waitForReplyInt32(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, bool dump);
 uint32_t waitForReplyuInt32(FlexCAN* CANbus, uint8_t nodeid, uint8_t subindex, bool dump);*/
extern "C" {
// callback function
    typedef void (*externalFunction)(void);
}

class CANsmc
{
	public:
	    CANsmc(FlexCAN* CANbus, Stream* port, uint8_t nodesid[4]); 
		void wrMsg(
			uint32_t  id, uint8_t len, uint8_t d0, uint8_t d1, uint8_t d2, 
			uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
        void writeToRegister(uint8_t nodeid, uint8_t subindex, uint32_t datas);
        void writeToRegister(uint8_t nodeid, uint8_t subindex, int32_t datas);
        void readRequestFromRegister(uint8_t nodeid, uint8_t subindex);
        bool waitForReply(uint8_t nodeid, uint8_t subindex, bool dump);
		bool waitForReplyInt32(
			uint8_t nodeid, uint8_t subindex, int32_t* number32, bool dump);
		bool waitForReplyuInt32(
			uint8_t nodeid, uint8_t subindex, uint32_t* numberu32, bool dump);
		void setTimeOutCAN(int timeout); 
		void setPSOLL(uint8_t nodeid, int32_t psoll);
		void setVelocityInt32(uint8_t nodeid, int32_t velocity);
		void setAsoll(uint8_t nodeid, int32_t asoll);
		void setVelocityMode(uint8_t nodeid);
		void setPositionMode(uint8_t nodeid);
		void setPassiveMode(uint8_t nodeid);
		void setTorque(uint8_t nodeid, float current_t);
		void checkTorque(uint8_t nodeid, bool dump);  
		void clearErrors(uint8_t nodeid, bool dump); 
		int32_t getActualVelocity(uint8_t nodeid);
		int32_t getEncoderVelocity(uint8_t nodeid);
		int32_t getFollowError(uint8_t nodeid);
		void setRunCurrent(uint8_t nodeid, float current);
		uint32_t getRunCurrent(uint8_t nodeid);
		void setStandbyCurrent(uint8_t nodeid, float current);
		float getStandbyCurrent(uint8_t nodeid);
		float getVSOLL(uint8_t nodeid);
		float getVelEnc(uint8_t nodeid);
		float getVIST(uint8_t nodeid);
		int32_t getPIST(uint8_t nodeid);
		float getActualTorquePercent(uint8_t nodeid);
		float getTorque(uint8_t nodeid);
		int32_t getEncoderPosition(uint8_t nodeid);
		uint8_t nodesids[4];
		int32_t encoder_pos[4]; 
		int32_t follow_err[4];
		int32_t actual_velocity[4];
		int32_t encoder_velocity[4];
		int32_t _last_encoder_pos[4];
		float joints_velocity[4];
		float joints_position[4];
		float offsets[4];
		float actual_torque_percent[4];
		uint32_t run_currents[4]; 
		void readVariables(); 
		float encoder_posd0;
		float encoder_posd1;
		float encoder_posd2;
		float encoder_posd3;
		float offsetscalibration[4];


		
	private: 
	    Stream* _serialport;  
		FlexCAN* _canport; 
		int _timeout;
		unsigned long _timetotimeout; 
		float _dt; 
		uint32_t _lt; 

};

#endif