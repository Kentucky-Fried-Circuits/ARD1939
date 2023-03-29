#ifndef ARD1939_H
#define ARD1939_H

#include "mcp_can.h"
#include "ARD1939_dfs.h" // user config parameters. Unfortunately, there's no good way in the Arduino IDE to move these parameters to your sketch

// J1939 Settings
#if ARD1939VERSION == 0
  #define TRANSPORT_PROTOCOL                    0
  #define J1939_MSGLEN                          8
  #define MSGFILTERS                            10
  #define TRANSPORT_PROTOCOL_XMIT				0
#endif

#if ARD1939VERSION == 1
  #define TRANSPORT_PROTOCOL                    1
  #define J1939_MSGLEN                          256
  #define MSGFILTERS                            10
  #define TRANSPORT_PROTOCOL_XMIT				1
#endif

#if ARD1939VERSION == 2
  #define TRANSPORT_PROTOCOL                    1
  #define J1939_MSGLEN                          1785
  #define MSGFILTERS                            100
  #define TRANSPORT_PROTOCOL_XMIT				1
#endif

// custom version created for ESM 2000
#if ARD1939VERSION == 3
  #define TRANSPORT_PROTOCOL                    1
  #define J1939_MSGLEN                          56 // even though ARD1939 doesn't seem to be able to fill this buffer in an RTSCTS command, we still have to allocate it or RTSCTS will fail with INSUFFICIENT RESOURCES
  #define MSGFILTERS                            1
  #define TRANSPORT_PROTOCOL_XMIT				1
#endif

#define GLOBALADDRESS                    	255
#define NULLADDRESS                      	254

#define RESERVED                            255  // CAN bus reserved or null data byte
// Return Codes
#define ADDRESSCLAIM_INIT                       0
#define ADDRESSCLAIM_INPROGRESS           	1
#define ADDRESSCLAIM_FINISHED             	2
#define NORMALDATATRAFFIC                 	2
#define ADDRESSCLAIM_FAILED               	3

#define J1939_MSG_NONE                   	0
#define J1939_MSG_PROTOCOL               	1
#define J1939_MSG_NETWORKDATA                   2
#define J1939_MSG_APP                   	3

// Compiler Settings
#define OK                                     0
#define ERR                                    1

#ifndef DEBUG_PRINT
#if DEBUG == 1
  #define DEBUG_PRINT(x) mySerial.print(x);
  #define DEBUG_PRINTLN(T) mySerial.println(T);
  #define DEBUG_PRINTLNFMT(x,y) mySerial.println(x,y);
#else
	#define DEBUG_PRINT(x) ;
  #define DEBUG_PRINTLN(T) ;
  #define DEBUG_PRINTLNFMT(x,y) ;
#endif
#endif
#if DEBUG == 1
  extern char sDebug[];
  #define DEBUG_INIT() char sDebug[128];
  #define DEBUG_PRINTHEX(T, v) mySerial.print(T); sprintf(sDebug, "%x\n\r", v); mySerial.print(sDebug);
  #define DEBUG_PRINTDEC(T, v) mySerial.print(T); sprintf(sDebug, "%d\n\r", v); mySerial.print(sDebug);
  #define DEBUG_PRINTARRAYHEX(T, a, l) mySerial.print(T); if(l == 0) mySerial.print("Empty.\n\r"); else {for(int x=0; x<l; x++){sprintf(sDebug, "%x ", a[x]); mySerial.print(sDebug);} mySerial.print("\n\r");}
  #define DEBUG_PRINTARRAYDEC(T, a, l) mySerial.print(T); if(l == 0) mySerial.print("Empty.\n\r"); else {for(int x=0; x<l; x++){sprintf(sDebug, "%d ", a[x]); mySerial.print(sDebug);} mySerial.print("\n\r");}
  #define DEBUG_HALT() while(mySerial.available() == 0); mySerial.setTimeout(1); mySerial.readBytes(sDebug, 1);
#endif

struct sTimer
{
  int timeoutMaybe;
  bool activeMaybe;
  bool expiredMaybe;
};

class ARD1939
{
  public: 
    // Initialization
    ARD1939(byte); // CS_PIN
	byte begin(int nSystemTime, boolean toggleTermination);
    byte Init(int nSystemTime);
    void SetPreferredAddress(byte nAddr);
    void SetAddressRange(byte nAddrBottom, byte nAddrTop);
    void SetNAME(long lIdentityNumber, int nManufacturerCode, byte nFunctionInstance, byte nECUInstance, 
                           byte nFunction, byte nVehicleSystem, byte nVehicleSystemInstance, byte nIndustryGroup, byte nArbitraryAddressCapable);
    byte SetMessageFilter(long lPGN); // seems to apply only to transport PGNs. I.e. 0xEC00 -BA
  
    // Read/Write - Check Status
    byte Operate(byte* nMsgId, long* lPGN, byte* pMsg, int* nMsgLen, byte* nDestAddr, byte* nSrcAddr, byte* nPriority);
    byte Transmit(byte nPriority, long lPGN, byte nSourceAddress, byte nDestAddress, const uint8_t* pData, int nDataLen);
  
    // Other Application Functions
    void Terminate(void);
    byte getSourceAddress(void);
    void DeleteMessageFilter(long lPGN);
    
  private:
	MCP_CAN *_CAN0;
    byte claimAddressMaybe(byte, byte*);
    bool getAnotherMyAddr(void); //TEST
    byte compareIds(byte* theirID, byte* myID);
    byte re_Receive(long*, byte*, int*, byte*, byte*, byte*);
    void f05(void);
    void resetTimeoutMaybe(struct sTimer*);
    bool re_isTransportProtocol(long*, byte*); //???
    bool isPeerToPeer(long);
    bool re_isFilterActive(long);

#if TRANSPORT_PROTOCOL == 1
    void f11(byte);
    void f12(byte);
    byte processTransportProtocol(long PGN, byte*, int, byte, byte, byte);
#endif
#if TRANSPORT_PROTOCOL_XMIT == 1
    byte rtsCtsTransmit(byte, long, byte, byte, byte*, int);
#endif
  
}; // end class ARD1939

#endif
