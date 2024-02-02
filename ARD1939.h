#ifndef ARD1939_H
#define ARD1939_H

#include "ARD1939_dfs.h" // user config parameters. Unfortunately, there's no good way in the Arduino IDE to move these parameters
#ifdef ARD_MCP_CAN
#include "mcp_can.h"
#else
#ifdef ARD_TWAI
#include "driver/twai.h"
#ifndef byte
typedef uint8_t byte;
#endif
#ifndef boolean
typedef bool boolean;
#endif
#else
#error a CAN controller driver must be defined in ARD1939_dfs.h
#endif
#endif

// J1939 Settings
#if ARD1939VERSION == 0
#define TRANSPORT_PROTOCOL 0
#define J1939_MSGLEN 8
#define MSGFILTERS 10
#define TRANSPORT_PROTOCOL_XMIT 0
#endif

#if ARD1939VERSION == 1
#define TRANSPORT_PROTOCOL 1
#define J1939_MSGLEN 256
#define MSGFILTERS 10
#define TRANSPORT_PROTOCOL_XMIT 1
#endif

#if ARD1939VERSION == 2
#define TRANSPORT_PROTOCOL 1
#define J1939_MSGLEN 1785
#define MSGFILTERS 100
#define TRANSPORT_PROTOCOL_XMIT 1
#endif

// custom version created for ESM 2000
#if ARD1939VERSION == 3
#define TRANSPORT_PROTOCOL 1
#define J1939_MSGLEN 56 // even though ARD1939 doesn't seem to be able to fill this buffer in an RTSCTS command, we still have to allocate it or RTSCTS will fail with INSUFFICIENT RESOURCES
#define MSGFILTERS 1
#define TRANSPORT_PROTOCOL_XMIT 1
#endif

#define GLOBALADDRESS 255
#define NULLADDRESS 254

#define RESERVED 255 // CAN bus reserved or null data byte
// Return Codes
#define ADDRESSCLAIM_INIT 0
#define ADDRESSCLAIM_INPROGRESS 1
#define ADDRESSCLAIM_FINISHED 2
#define NORMALDATATRAFFIC 2
#define ADDRESSCLAIM_FAILED 3

#define J1939_MSG_NONE 0
#define J1939_MSG_PROTOCOL 1
#define J1939_MSG_NETWORKDATA 2
#define J1939_MSG_APP 3

// Compiler Settings
#define OK 0
#define ERR 1

#ifndef DEBUG_PRINT
#if DEBUG == 1
#ifdef ARD_MCP_CAN
#define DEBUG_PRINT(x) mySerial.print(x);
#define DEBUG_PRINTLN(T) mySerial.println(T);
#define DEBUG_PRINTLNFMT(x, y) mySerial.println(x, y);
#endif
#ifdef ARD_TWAI
#warning "ARD_TWAI debugging enabled"
// #ifdef F // FIXME can't find a way to nullify the F() macro
// #undef F
// #define F(x) sizeof(x)
// #endif
#define DEBUG_PRINT(x) printf(x);
#define DEBUG_PRINTLN(T) ESP_LOGD("ARD1939d", T);
#define DEBUG_PRINTLNFMT(x, y) ESP_LOGD("ARD1939d", "0x%x", x);
#endif
#else
#define DEBUG_PRINT(x) ;
#define DEBUG_PRINTLN(T) ;
#define DEBUG_PRINTLNFMT(x, y) ;
#endif
#endif
#if DEBUG == 2
#ifdef ARD_TWAI
#define DEBUG_ESPLOG1(a, b) ESP_LOGD(a, b)
#define DEBUG_ESPLOG2(a, b, c) ESP_LOGD(a, b, c)
#define DEBUG_ESPLOG3(a, b, c, d) ESP_LOGD(a, b, c, d)
#define DEBUG_ESPLOG(a, b, c, d, e) ESP_LOGD(a, b, c, d, e)
#else
#define DEBUG_ESPLOG1(a, b)
#define DEBUG_ESPLOG2(a, b, c)
#define DEBUG_ESPLOG3(a, b, c, d)
#define DEBUG_ESPLOG(a, b, c, d, e)
#endif
#endif
#if DEBUG == 1
extern char sDebug[];
#define DEBUG_INIT() char sDebug[128];
#define DEBUG_PRINTHEX(T, v)    \
  mySerial.print(T);            \
  sprintf(sDebug, "%x\n\r", v); \
  mySerial.print(sDebug);
#define DEBUG_PRINTDEC(T, v)    \
  mySerial.print(T);            \
  sprintf(sDebug, "%d\n\r", v); \
  mySerial.print(sDebug);
#define DEBUG_PRINTARRAYHEX(T, a, l) \
  mySerial.print(T);                 \
  if (l == 0)                        \
    mySerial.print("Empty.\n\r");    \
  else                               \
  {                                  \
    for (int x = 0; x < l; x++)      \
    {                                \
      sprintf(sDebug, "%x ", a[x]);  \
      mySerial.print(sDebug);        \
    }                                \
    mySerial.print("\n\r");          \
  }
#define DEBUG_PRINTARRAYDEC(T, a, l) \
  mySerial.print(T);                 \
  if (l == 0)                        \
    mySerial.print("Empty.\n\r");    \
  else                               \
  {                                  \
    for (int x = 0; x < l; x++)      \
    {                                \
      sprintf(sDebug, "%d ", a[x]);  \
      mySerial.print(sDebug);        \
    }                                \
    mySerial.print("\n\r");          \
  }
#define DEBUG_HALT()                \
  while (mySerial.available() == 0) \
    ;                               \
  mySerial.setTimeout(1);           \
  mySerial.readBytes(sDebug, 1);
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
#ifdef ARD_MCP_CAN
  ARD1939(byte); // CS_PIN
#endif
#ifdef ARD_TWAI
  ARD1939(twai_general_config_t *, twai_timing_config_t *);
#endif
  byte begin(int nSystemTime, boolean toggleTermination);
  void DeleteMessageFilter(long lPGN);
  byte getSourceAddress(void);

  /**
   * @brief get the address of the last ADDRESS_CLAIMED message on the J1939 bus
   *
   * @return byte J1939 address. NULL_ADDRESS if no address claim since last reset
   */
  byte GetAddressClaimed();

  /**
   * @brief initialize variables and install CAN driver
   *
   * @param nSystemTime - ms per tick
   * @return byte - result of canInit() or twai_driver_install(). 0/ESP_OK for success.
   */
  byte Init(int nSystemTime);

  /**
   * @brief receive messages and handle j1939 protocol
   * @param byte *nMsgId - J1939 message type (J1939_MSG_APP|J1939_MSG_NETWORKDATA|J1939_MSG_PROTOCOL|J1939_MSG_NONE)
   * J1939_MSG_APP means a normal application message
   * @return J1939 Status
   *
   */
  byte Operate(byte *nMsgId, long *lPGN, byte *pMsg, int *nMsgLen, byte *nDestAddr, byte *nSrcAddr, byte *nPriority);

  /**
   * @brief set the last address claimed back to NULL_ADDRESS
   *
   */
  void ResetAddressClaimed(void);

  void SetPreferredAddress(byte nAddr);
  void SetAddressRange(byte nAddrBottom, byte nAddrTop);
  void SetNAME(long lIdentityNumber, int nManufacturerCode, byte nFunctionInstance, byte nECUInstance,
               byte nFunction, byte nVehicleSystem, byte nVehicleSystemInstance, byte nIndustryGroup, byte nArbitraryAddressCapable);
  byte SetMessageFilter(long lPGN); // seems to apply only to transport PGNs. I.e. 0xEC00 -BA

  // Other Application Functions
  void Terminate(void);
  byte Transmit(byte nPriority, long lPGN, byte nSourceAddress, byte nDestAddress, const uint8_t *pData, int nDataLen);

  /*Check if the twai receive queue is empty*/
#ifdef ARD_TWAI
  bool isRxQueueEmpty();
#endif

private:
  byte nAddressClaimed; // address of last address_claimed message
#ifdef ARD_MCP_CAN
  MCP_CAN *_CAN0;
#endif
#ifdef ARD_TWAI
  twai_general_config_t *_g_config;
  twai_timing_config_t *_t_config;
#endif
  byte claimAddressMaybe(byte, byte *);
  bool getAnotherMyAddr(void); // TEST
  byte compareIds(byte *theirID, byte *myID);
  byte receive(long *, byte *, int *, byte *, byte *, byte *);
  void f05(void);
  void resetTimeoutMaybe(struct sTimer *);
  bool isTransportProtocol(long *, byte *); //???
  bool isPeerToPeer(long);
  bool isFilterActive(long);

#if TRANSPORT_PROTOCOL == 1
  void f11(byte);
  void f12(byte);
  byte processTransportProtocol(long PGN, byte *, int, byte, byte, byte);
#endif

#if TRANSPORT_PROTOCOL_XMIT == 1
  byte rtsCtsTransmit(byte, long, byte, byte, byte *, int);
#endif

}; // end class ARD1939

#endif // ARD1939_H
