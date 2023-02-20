// BA has done some reverse engineering. Variable names starting with "re_" have been reverse engineered
// variables ending in "Maybe" may or may not be named well

#include <arduino.h>
#include <stdlib.h>
#include <inttypes.h>
#include <SPI.h>
#include "mcp_can.h"
#include "ARD1939.h"

#define d49 10

#define d50 8

#define PGN_REQUEST 0x00EA00
//#define PGN_REQUEST_GLOBAL         	0x00EAFF
#define d46 0x00
#define re_PF_REQUEST 0xEA
#define d48 0x00

#define PGN_ADDRESS_CLAIMED 0x00EE00
#define PGN_ADDRESS_CLAIMED_BROADCAST 0x00EEFF  // not used?
#define CANNOT_CLAIM_SA_LSB 0x00
#define CANNOT_CLAIM_SA_2ND 0xEE
#define CANNOT_CLAIM_SA_MSB 0x00
#define priority 6

#define d33 0x00EE00
#define d34 0x00FED8

#define d35 0x00ECFF
#define TPCM_PGN 0x00EC00
#define d37 0xEC
#define d38 7

#define re_TPDT_PGN 0x00EB00  //TP.DT PGN
#define d40 7

#define d41 32
#define TPCM_RTS_CONTROL_BYTE 16              // TP.CM_RTS control byte
#define TPCM_CTS_CONTROL_BYTE 17              // TP.CM_CTS control byte
#define re_TPCM_ENDOFMSG_ACK_CONTROL_BYTE 19  // TP.CM_EndOfMsgACK control byte
#define TP_CONN_ABORT 255                     // TP.Conn_Abort control byte

#define LOWEST_PRIORITY 255

#define NAME_ID 0

#define UNUSED(x) (void)(x)  // this macro is used to supress unused-parameter warnings where I intend to leave them unused

#if DEBUG != 0
#ifdef SOFTWARE_SERIAL_TX_PIN
#include <SoftwareSerial.h>
#warning*** Software Serial ***
SoftwareSerial mySerial(0, SOFTWARE_SERIAL_TX_PIN);
#else
#warning--- hardware serial ---
#define mySerial Serial
#endif
#endif

struct re_structFilter {
  bool bActive;
  long re_lPGN;
};
re_structFilter re_pFilters[MSGFILTERS];

unsigned char j1939Name[] = {
  (byte)(NAME_IDENTITY_NUMBER & 0xFF),
  (byte)((NAME_IDENTITY_NUMBER >> 8) & 0xFF),
  (byte)((((long)NAME_MANUFACTURER_CODE << 5) & 0xFF) | (NAME_IDENTITY_NUMBER >> 16)),
  (byte)(NAME_MANUFACTURER_CODE >> 3),
  (byte)((NAME_FUNCTION_INSTANCE << 3) | NAME_ECU_INSTANCE),
  (byte)(NAME_FUNCTION),
  (byte)(NAME_VEHICLE_SYSTEM << 1),
  (byte)((NAME_ARBITRARY_ADDRESS_CAPABLE << 7) | (NAME_INDUSTRY_GROUP << 4) | (NAME_VEHICLE_SYSTEM_INSTANCE))
};

#define re_NUM_TP_PGNS 4
long re_TP_PGNS[] = {
  PGN_ADDRESS_CLAIMED,
  d34,
  TPCM_PGN,
  re_TPDT_PGN
};

struct j1939Stat_t {
  bool v07;
  byte canOperate;  // should be a boolean, because it's only assigned booleans
  bool nextTimeTryAddrRange;
  bool v10;
  bool addressClaimFailed;
  byte nSAPreferred;
  byte addrBottom;
  byte addrTop;
  bool checkingAddrRange;
  byte myAddr;
  byte nSourceAddress;
};
struct j1939Stat_t j1939Stat;


#if TRANSPORT_PROTOCOL == 1
#define TRANSPORT_PROTOCOL_0 0
#define TRANSPORT_PROTOCOL_1 1
#define TRANSPORT_PROTOCOL_2 2

struct re_j1939_t {
  byte TRANSPORT_PROTOCOL_MODE;
  bool running;
  bool v22;
  bool v23;
  bool v24;
  long re_lPGN;
  byte v26;
  byte v27;
  byte v28;
  byte v29;
  byte re_pFullMessage[J1939_MSGLEN];
  int re_nMsgLen;
  byte v32;
};
re_j1939_t v33;
re_j1939_t v34;

#define d04 32
#define d07 19

// TP CONNECTION ABORT REASONS
#define TP_NODE_IN_ANOTHER_SESSION 1
#define TP_LACKING_RESOURCES 2
#define TP_TIMEOUT 3

#define d12 0
#define d13 1

byte msgBuf[] = { 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00 };
byte v63[8];
#endif

struct sTimer addressClaimTimer;
struct sTimer timer2;

#if TRANSPORT_PROTOCOL == 1
struct sTimer tpTimer1;
struct sTimer tpTimer2;

struct sTimer minPacketsTimer;
struct sTimer tpTimer4;
struct sTimer tpTimer5;
struct sTimer tpTimer6;
struct sTimer tpTimer7;
struct sTimer tpTimer8;
struct sTimer tpTimer9;
#endif

int timeout250msTics;
int v50;

#if TRANSPORT_PROTOCOL == 1
int v51;
int v52;

int minPacketsTics;
int responseTics;
int holdingTics;
int betweenPacketsTics;
int responseAfterCtsTics;
int responseAfterRtsTics;
int v59;
#endif

#define timeout250ms 250
#define d15 100

#if TRANSPORT_PROTOCOL == 1
#define d16 50
#define d17 750

#define minPacketsTime 50
#define responseTime 200
#define holdingTime 500
#define BetweenPacketsTime 750
#define ResponseAfterCtsTime 1250
#define ResponseAfterRtsTime 1250
#define d24 1050
#endif

int v60[] = { 8, 9, 10, 12, 15 };
byte v60index;

byte v64;
extern byte canInit(MCP_CAN*);
extern byte canCheckError(MCP_CAN*);
extern byte canTransmit(long, unsigned char*, int, MCP_CAN*);
extern byte canReceive(long*, unsigned char*, int*, MCP_CAN*);

ARD1939::ARD1939(byte _CS) {
  _CAN0 = new MCP_CAN(_CS);  // Set CS to pin 9/10
  /*	SPICS = _CS;  // the MCP_CAN constructor already does this. why do it again?
    pinMode(SPICS, OUTPUT);
    MCP2515_UNSELECT();
	*/
}

byte ARD1939::Init(int nSystemTime) {
  int re_i;
  mySerial.begin(MONITOR_BAUD_RATE);
  _resetTimeout(&addressClaimTimer);
  _resetTimeout(&timer2);
  timeout250msTics = timeout250ms / nSystemTime;
  v50 = d15 / nSystemTime;
  v60index = 0;
  v60[0] = (int)8 / nSystemTime;
  v60[1] = (int)9 / nSystemTime;
  v60[2] = (int)10 / nSystemTime;
  v60[3] = (int)12 / nSystemTime;
  v60[4] = (int)15 / nSystemTime;
  j1939Stat.v07 = false;
  j1939Stat.canOperate = false;
  j1939Stat.nextTimeTryAddrRange = false;
  j1939Stat.v10 = false;
  j1939Stat.addressClaimFailed = false;
  j1939Stat.nSAPreferred = NULLADDRESS;
  j1939Stat.addrBottom = NULLADDRESS;
  j1939Stat.addrTop = NULLADDRESS;
  j1939Stat.checkingAddrRange = false;
  j1939Stat.myAddr = SA_PREFERRED;
  j1939Stat.nSourceAddress = NULLADDRESS;
  for (re_i = 0; re_i < MSGFILTERS; re_i++) {
    re_pFilters[re_i].bActive = false;
    re_pFilters[re_i].re_lPGN = 0;
  }

#if TRANSPORT_PROTOCOL == 1
  v51 = d16 / nSystemTime;
  v52 = d17 / nSystemTime;

  minPacketsTics = minPacketsTime / nSystemTime;
  responseTics = responseTime / nSystemTime;
  holdingTics = holdingTime / nSystemTime;
  betweenPacketsTics = BetweenPacketsTime / nSystemTime;
  responseAfterCtsTics = ResponseAfterCtsTime / nSystemTime;
  responseAfterRtsTics = ResponseAfterRtsTime / nSystemTime;
  v59 = d24 / nSystemTime;
  f11(d12);
  f12(d12);
#endif

  v64 = 0;
  return canInit(_CAN0);
}

byte ARD1939::j1939Init() {
  sTimer faultTimer = { 3000, true, true };
  byte status;
  byte nMsgId = 0;
  long lPGN = 0x00L;
  byte pMsg[8] = { RESERVED, RESERVED, RESERVED, RESERVED, RESERVED, RESERVED, RESERVED, RESERVED };
  int nMsgLen = 0;
  byte nDestAddr = NULLADDRESS;
  byte nSrcAddr = NULLADDRESS;
  byte nPriority = 0xFF;  // lowest wins
  DEBUG_PRINT("enter j1939Init (9I)");
  SetPreferredAddress(SA_PREFERRED);
  SetAddressRange(ADDRESSRANGEBOTTOM, ADDRESSRANGETOP);
  SetNAME(NAME_IDENTITY_NUMBER,
                NAME_MANUFACTURER_CODE,
                NAME_FUNCTION_INSTANCE,
                NAME_ECU_INSTANCE,
                NAME_FUNCTION,
                NAME_VEHICLE_SYSTEM,
                NAME_VEHICLE_SYSTEM_INSTANCE,
                NAME_INDUSTRY_GROUP,
                NAME_ARBITRARY_ADDRESS_CAPABLE);
  SetMessageFilter(PGN_ECU_IDENTIFICATION_INFORMATION);  // we OPT IN to RTS CTS messages
  do {                                                         // loop until we get good j1939 status
    DEBUG_PRINT("9I");
    status = Operate(&nMsgId, &lPGN, &pMsg[0], &nMsgLen, &nDestAddr, &nSrcAddr, &nPriority);
    if (--faultTimer.timeout == 0) {
      return ADDRESSCLAIM_TIMEOUT;
    }
    if (status == ADDRESSCLAIM_FAILED) {
      return status;
    }
    delay(1);
  } while (status != NORMALDATATRAFFIC);
  return status;
}

void ARD1939::Terminate(void) {
  Init(d15 / v50);  // d15/v50 is a hack to recover the original nSystemTime.
}

byte ARD1939::Operate(byte* nMsgId, long* lPGN, byte* pMsg, int* nMsgLen, byte* nDestAddr, byte* nSrcAddr, byte* nPriority) {
  byte j1939Status;
  byte compareIdsRetVal;
  _updateTimers();
  *nMsgId = receive(lPGN, &pMsg[0], nMsgLen, nDestAddr, nSrcAddr, nPriority);
  DEBUG_PRINT("Operate:nMsgId:");
  DEBUG_PRINTLN(*nMsgId);
  if (*nMsgId == J1939_MSG_APP) {
    if (*nDestAddr != j1939Stat.nSourceAddress
        && *nDestAddr != GLOBALADDRESS)
      *nMsgId = J1939_MSG_NETWORKDATA;
  }
  if (j1939Stat.canOperate == true) {
    j1939Status = NORMALDATATRAFFIC;
    switch (*lPGN) {
      case PGN_REQUEST:
        if (*nMsgId == J1939_MSG_PROTOCOL) {
          if (pMsg[0] == CANNOT_CLAIM_SA_MSB
              && pMsg[1] == CANNOT_CLAIM_SA_2ND
              && pMsg[2] == CANNOT_CLAIM_SA_LSB) {
            if (*nDestAddr == GLOBALADDRESS) {
              Transmit(priority, PGN_ADDRESS_CLAIMED, j1939Stat.nSourceAddress, GLOBALADDRESS,
                       &j1939Name[NAME_ID], 8);
            } else if (*nDestAddr == j1939Stat.nSourceAddress) {
              Transmit(priority, PGN_ADDRESS_CLAIMED, j1939Stat.nSourceAddress, *nSrcAddr,
                       &j1939Name[NAME_ID], 8);
            }
          }
        }
        break;

      case PGN_ADDRESS_CLAIMED:
        if (*nSrcAddr == j1939Stat.nSourceAddress)  // we received a message from our address. We need to tell them to try a different address by sending our own Address Claimed message
        {
          compareIdsRetVal = compareIds(&pMsg[0], &j1939Name[NAME_ID]);
          switch (compareIdsRetVal) {
            case 0:                                                                  // The bugger has the exact same ID as me.
              DEBUG_PRINTLN(F("~298:Op():PGN==PGN_ADDRESS_CLAIMED:compareIds==0"));  // we fail first here when another 0x80 comes online
              // instead of failing, we should tell them to bugger off, just like case 2
              Transmit(priority, PGN_ADDRESS_CLAIMED, j1939Stat.nSourceAddress,
                       GLOBALADDRESS, &j1939Name[NAME_ID], 8);
              break;
              // #if TRANSPORT_PROTOCOL == 1
              // f11(d12);
              // f12(d12);
              // #endif
              // Transmit(priority, PGN_ADDRESS_CLAIMED, NULLADDRESS, GLOBALADDRESS, &j1939Name[NAME_ID], 8);
              // j1939Stat.canOperate = false;
              // j1939Stat.addressClaimFailed = true;
              // j1939Status = ADDRESSCLAIM_FAILED;
              break;
            case 1:
#if TRANSPORT_PROTOCOL == 1
              f11(d12);
              f12(d12);
#endif
              j1939Stat.v07 = false;
              j1939Stat.canOperate = false;
              claimAddress(*nSrcAddr, &j1939Name[NAME_ID]);
              break;
            case 2:
              Transmit(priority, PGN_ADDRESS_CLAIMED, j1939Stat.nSourceAddress,
                       GLOBALADDRESS, &j1939Name[NAME_ID], 8);
              break;
          }
        }
        break;
      case d34:
        break;
    }

#if TRANSPORT_PROTOCOL == 1
    processTransportProtocol(*lPGN, pMsg, *nMsgLen, *nDestAddr, *nSrcAddr, *nPriority);
    if (*nMsgId == J1939_MSG_NONE) {
      if (v33.v32 == true) {
        *lPGN = v33.re_lPGN;
        *nMsgLen = v33.re_nMsgLen;
        *nDestAddr = v33.v27;
        *nSrcAddr = v33.v26;
        *nPriority = LOWEST_PRIORITY;
        //    		  for(re_i = 0; re_i < v33.re_nMsgLen; re_i++)
        //    			  pMsg[re_i] = v33.re_pFullMessage[re_i];
        memcpy(pMsg, v33.re_pFullMessage, v33.re_nMsgLen);  //TEST
        f11(d12);
        *nMsgId = J1939_MSG_APP;
      } else if (v34.v32 == true) {
        *lPGN = v34.re_lPGN;
        *nMsgLen = v34.re_nMsgLen;
        *nDestAddr = v34.v27;
        *nSrcAddr = v34.v26;
        *nPriority = LOWEST_PRIORITY;
        //    		  for(re_i = 0; re_i < v34.re_nMsgLen; re_i++)
        //  			  pMsg[re_i] = v34.re_pFullMessage[re_i];
        memcpy(pMsg, v34.re_pFullMessage, v34.re_nMsgLen);  //TEST
        f12(d12);
        *nMsgId = J1939_MSG_APP;
      }
    }
#endif
  } else if (j1939Stat.addressClaimFailed == true) {
    DEBUG_PRINTLN(F("~363:Op():stat.claimFailed"));  // we fail here on subsequent calls to Operate()
    j1939Status = ADDRESSCLAIM_FAILED;
    switch (*lPGN) {
      case PGN_REQUEST:
        if (*nMsgId == J1939_MSG_PROTOCOL) {
          if (pMsg[0] == CANNOT_CLAIM_SA_MSB
              && pMsg[1] == CANNOT_CLAIM_SA_2ND
              && pMsg[2] == CANNOT_CLAIM_SA_LSB) {
            if (*nDestAddr == GLOBALADDRESS) {
              timer2.timeout = v60[v60index++];
              timer2.running = true;
              if (v60index > 4)
                v60index = 0;
            }
          }
        }
        break;
      case PGN_ADDRESS_CLAIMED:
        break;
      case d34:
        break;
    }
    if (timer2.expired == true) {
      _resetTimeout(&timer2);
      Transmit(priority, PGN_ADDRESS_CLAIMED, NULLADDRESS, GLOBALADDRESS,
               &j1939Name[NAME_ID], 8);
    }
  } else {
    j1939Status = claimAddress(*nSrcAddr, &pMsg[0]);
  }
  return j1939Status;
}

/** @brief claim a j1939 source address
*   @note TODO byte myAddress does not appear to be used. j1939Stat.myAddr is used instead
**/
byte ARD1939::claimAddress(byte myAddr, byte* v91) {
  byte j1939Status;
  byte compareIdsRetVal;
  j1939Status = ADDRESSCLAIM_INPROGRESS;
  DEBUG_PRINT("claimAddres:mAddr:");
  DEBUG_PRINTLN(j1939Stat.myAddr);
  if (j1939Stat.addressClaimFailed == true) {
    DEBUG_PRINTLN(F("~409:claimAddr():stat.claimFailed"));
    j1939Status = ADDRESSCLAIM_FAILED;
  } else if (j1939Stat.canOperate == true) {
    j1939Status = ADDRESSCLAIM_FINISHED;
  } else if (j1939Stat.v10 == true) {
    if (timer2.expired == true) {
      _resetTimeout(&timer2);
      Transmit(priority, PGN_ADDRESS_CLAIMED, j1939Stat.myAddr, GLOBALADDRESS,
               &j1939Name[NAME_ID], 8);
      addressClaimTimer.timeout = timeout250msTics;
      addressClaimTimer.running = true;
      j1939Stat.v10 = false;
    }
  } else {
    if (j1939Stat.v07 == false) {
      if (getAnotherMyAddr() == true) {
        DEBUG_PRINTLN(F("~437:claimAddr():j1939Stat.v07:false, getAnother:true"));
        Transmit(priority, PGN_ADDRESS_CLAIMED, j1939Stat.myAddr, GLOBALADDRESS,
                 &j1939Name[NAME_ID], 8);
        addressClaimTimer.timeout = timeout250msTics;
        addressClaimTimer.running = true;
        j1939Stat.v07 = true;
      } else {
        DEBUG_PRINTLN(F("~439:claimAddr():defaultStat:v07==false:getAnother==false"));
        Transmit(priority, PGN_ADDRESS_CLAIMED, NULLADDRESS, GLOBALADDRESS, &j1939Name[NAME_ID], 8);
        j1939Stat.canOperate = false;
        j1939Stat.addressClaimFailed = true;
        j1939Status = ADDRESSCLAIM_FAILED;
      }
    } else {
      if (addressClaimTimer.expired == true) {
        _resetTimeout(&addressClaimTimer);
        j1939Stat.nSourceAddress = j1939Stat.myAddr;
        j1939Stat.canOperate = true;
        j1939Status = ADDRESSCLAIM_FINISHED;
      } else {
        if (canCheckError(_CAN0) == 1)  // there's Maybe a problem
        {
          _resetTimeout(&addressClaimTimer);
          if (++v64 == d49) {
            DEBUG_PRINTLN(F("~462:claimAddr():else:!v07:!timer1expired:canCheckError==1:v64==d49"));
            j1939Stat.canOperate = false;
            j1939Stat.addressClaimFailed = true;
            j1939Status = ADDRESSCLAIM_FAILED;
          } else {
            canInit(_CAN0);
            timer2.timeout = v60[v60index++];
            timer2.running = true;
            if (v60index > 4)
              v60index = 0;
            j1939Stat.v10 = true;
          }
        } else
          v64 = 0;
        if (myAddr == j1939Stat.myAddr)  // I'm the requestor of an address, and my address is claimed already
        {
          compareIdsRetVal = compareIds(&v91[0], &j1939Name[NAME_ID]);
          switch (compareIdsRetVal) {
            case 0:                                                                                                  // my address is claimed by someone with the same ID as me. They told me to bugger off
              DEBUG_PRINTLN(F("~485:claimAddr():else:v07==false:!timer1expired:myAddr=stat.myAddr:compareIds==0"));  // identical addresses land here when I'm the second node.
                                                                                                                     // instead of failing at this point, Ima get me another address by falling through to case 1
                                                                                                                     // #if TRANSPORT_PROTOCOL == 1
              // f11(d12);
              // f12(d12);
              // #endif
              // Transmit(priority, PGN_ADDRESS_CLAIMED, NULLADDRESS, GLOBALADDRESS, &j1939Name[NAME_ID], 8);
              // j1939Stat.canOperate = false;
              // j1939Stat.addressClaimFailed = true;
              // j1939Status = ADDRESSCLAIM_FAILED;
              // break;

            case 1:
#if TRANSPORT_PROTOCOL == 1
              f11(d12);
              f12(d12);
#endif
              if (getAnotherMyAddr() == true) {
                Transmit(priority, PGN_ADDRESS_CLAIMED, j1939Stat.myAddr, GLOBALADDRESS,
                         &j1939Name[NAME_ID], 8);
                addressClaimTimer.timeout = timeout250msTics;
                addressClaimTimer.running = true;
              } else {
                DEBUG_PRINTLN(F("~510:claimAddr():else:v07==false:!timer1expired:myAddr=stat.myAddr:compareIds==1:getAnother==false"));

                Transmit(priority, PGN_ADDRESS_CLAIMED, NULLADDRESS, GLOBALADDRESS,
                         &j1939Name[NAME_ID], 8);
                j1939Stat.canOperate = false;
                j1939Stat.addressClaimFailed = true;
                j1939Status = ADDRESSCLAIM_FAILED;
              }
              break;

            case 2:
              Transmit(priority, PGN_ADDRESS_CLAIMED, j1939Stat.myAddr, GLOBALADDRESS,
                       &j1939Name[NAME_ID], 8);
              addressClaimTimer.timeout = timeout250msTics;
              addressClaimTimer.running = true;
              break;
          }
        }
      }
    }
  }
  return j1939Status;
}

bool ARD1939::getAnotherMyAddr(void)
// myAddr: address to try?
// checkingAddrRange: false if we haven't tried an address from the range yet?
// returnVal: true if there's an address to try?
{
  bool returnVal;
  returnVal = true;
  if (j1939Stat.nSAPreferred == NULLADDRESS)  // setting SA_PREFERRED to NULLADDRESS is equivalent to saying I don't care--give me anything in the range
    j1939Stat.nextTimeTryAddrRange = true;
  if (j1939Stat.nextTimeTryAddrRange == false) {
    j1939Stat.myAddr = j1939Stat.nSAPreferred;
    j1939Stat.nextTimeTryAddrRange = true;
  } else {                                                                          // get next address from range
    if (j1939Stat.addrBottom == NULLADDRESS || j1939Stat.addrTop == NULLADDRESS) {  // there is no valid range, so fail
      returnVal = false;
    } else {
      if (j1939Stat.myAddr == NULLADDRESS || j1939Stat.checkingAddrRange == false) {  // we haven't tried anything yet, or ?
        j1939Stat.myAddr = j1939Stat.addrBottom;
        j1939Stat.checkingAddrRange = true;
      } else {
        if (j1939Stat.myAddr < j1939Stat.addrTop)
          j1939Stat.myAddr++;
        else
          returnVal = false;
      }
      if (j1939Stat.myAddr == j1939Stat.nSAPreferred) {
        if (j1939Stat.myAddr < j1939Stat.addrTop)
          j1939Stat.myAddr++;
        else
          returnVal = false;
      }
    }
  }
  return returnVal;
}

// compares received name ID (pMsg) to my ID (pNameID).
// if my name is lower (higher priority), return 2
// if their name is higher (lower priority, return 1
// if they are the same, return 0
byte ARD1939::compareIds(byte* pMsg, byte* pNameID) {
  byte i;
  for (i = 8; i > 0; i--) {
    if (pMsg[i - 1] != pNameID[i - 1]) {
      if (pMsg[i - 1] < pNameID[i - 1])
        return 1;
      else
        return 2;
    }
  }
  return 0;
}

/* receive a CAN message
   returns J1939_MSG_PROTOCOL if ??
           J1919_MSG_APP if a message??
		   J1939_MSG_NONE if no message
*/
byte ARD1939::receive(long* re_lPGN, byte* re_pMsg, int* re_nMsgLen, byte* re_nDestAddr, byte* re_nSrcAddr, byte* re_nPriority) {
  long v78;
  long re_lID;
  *re_lPGN = 0;
  *re_nMsgLen = 0;
  *re_nDestAddr = NULLADDRESS;
  *re_nSrcAddr = NULLADDRESS;
  *re_nPriority = 0;
  if (canReceive(&re_lID, &re_pMsg[0], re_nMsgLen, _CAN0) == 0)  // we have a message
  {
    v78 = re_lID & 0x1C000000;
    *re_nPriority = (byte)(v78 >> 26);
    *re_lPGN = re_lID & 0x01FFFF00;
    *re_lPGN = *re_lPGN >> 8;
    *re_nSrcAddr = (byte)(re_lID & 0x000000FF);
    *re_nDestAddr = GLOBALADDRESS;
    if (isPeerToPeer(*re_lPGN) == true) {
      *re_nDestAddr = (byte)(*re_lPGN & 0xFF);  // mask out just the destination address
      *re_lPGN = *re_lPGN & 0x01FF00;           // mask out just the data page bit and first byte of PGN
    }
    if (isTransportProtocol(re_lPGN, &re_pMsg[0]) == true)
      return J1939_MSG_PROTOCOL;
    else
      return J1939_MSG_APP;
  } else
    return J1939_MSG_NONE;
}

byte ARD1939::Transmit(byte re_nPriority, long re_lPGN, byte nSourceAddress, byte re_nDestAddress, byte* re_pData, int re_nDataLen) {
  long re_lID;
  if (re_nDataLen > J1939_MSGLEN)
    return ERR;
  re_lID = ((long)re_nPriority << 26) + (re_lPGN << 8) + (long)nSourceAddress;
  if (isPeerToPeer(re_lPGN) == true)
    re_lID = re_lID | ((long)re_nDestAddress << 8);
  if (re_nDataLen > 8)
#if TRANSPORT_PROTOCOL_XMIT == 1
    return rtsCtsTransmit(re_nPriority, re_lPGN, nSourceAddress, re_nDestAddress, re_pData, re_nDataLen);
#else
    return ERR;
#endif
  else
    return canTransmit(re_lID, re_pData, re_nDataLen, _CAN0);
}

void ARD1939::_updateTimers(void) {
  if (addressClaimTimer.running == true && addressClaimTimer.expired == false) {
    if (--addressClaimTimer.timeout == 0)
      addressClaimTimer.expired = true;
  }
  if (timer2.running == true && timer2.expired == false)
    if (--timer2.timeout == 0)
      timer2.expired = true;
#if TRANSPORT_PROTOCOL == 1
  if (tpTimer1.running == true && tpTimer1.expired == false)
    if (--tpTimer1.timeout == 0)
      tpTimer1.expired = true;
  if (tpTimer2.running == true && tpTimer2.expired == false)
    if (--tpTimer2.timeout == 0)
      tpTimer2.expired = true;
  if (minPacketsTimer.running == true && minPacketsTimer.expired == false)
    if (--minPacketsTimer.timeout == 0)
      minPacketsTimer.expired = true;
  if (tpTimer4.running == true && tpTimer4.expired == false)
    if (--tpTimer4.timeout == 0)
      tpTimer4.expired = true;
  if (tpTimer5.running == true && tpTimer5.expired == false)
    if (--tpTimer5.timeout == 0)
      tpTimer5.expired = true;
  if (tpTimer6.running == true && tpTimer6.expired == false)
    if (--tpTimer6.timeout == 0)
      tpTimer6.expired = true;
  if (tpTimer7.running == true && tpTimer7.expired == false)
    if (--tpTimer7.timeout == 0)
      tpTimer7.expired = true;
  if (tpTimer8.running == true && tpTimer8.expired == false)
    if (--tpTimer8.timeout == 0)
      tpTimer8.expired = true;
  if (tpTimer9.running == true && tpTimer9.expired == false)
    if (--tpTimer9.timeout == 0)
      tpTimer9.expired = true;
#endif
}

void ARD1939::_resetTimeout(struct sTimer* v75) {
  v75->timeout = 0;
  v75->running = false;
  v75->expired = false;
}

bool ARD1939::isTransportProtocol(long* re_lPGN, byte* re_pMsg) {
  bool re_bResult;
  byte re_i;
  byte re_nPF;
  byte re_nPS;
  re_bResult = false;
  re_nPF = (byte)((*re_lPGN & 0x00FF00) >> 8);  // PF field of ID
  re_nPS = (byte)(*re_lPGN & 0x0000FF);
  switch (re_nPF) {
    case re_PF_REQUEST:
      if (re_nPS == GLOBALADDRESS) {
        if (re_pMsg[0] == CANNOT_CLAIM_SA_MSB
            && re_pMsg[1] == CANNOT_CLAIM_SA_2ND
            && re_pMsg[2] == CANNOT_CLAIM_SA_LSB)
          re_bResult = true;
      } else {
        *re_lPGN = *re_lPGN & 0x00FF00;
        if (re_pMsg[0] == CANNOT_CLAIM_SA_MSB
            && re_pMsg[1] == CANNOT_CLAIM_SA_2ND
            && re_pMsg[2] == CANNOT_CLAIM_SA_LSB)
          re_bResult = true;
      }
      break;

    default:
      for (re_i = 0; re_i < re_NUM_TP_PGNS - 1; re_i++) {
        if (*re_lPGN == re_TP_PGNS[re_i]) {
          re_bResult = true;
          break;
        }
      }
      break;
  }
  return re_bResult;
}

bool ARD1939::isPeerToPeer(long re_lPGN) {
  if (re_lPGN > 0 && re_lPGN <= 0xEFFF)
    return true;  // peer-to-peer PGN
  if (re_lPGN > 0x10000 && re_lPGN <= 0x1EFFF)
    return true;  // peer-to-peer PGN with data page bit set
  return false;
}

byte ARD1939::GetSourceAddress(void) {
  return j1939Stat.nSourceAddress;
}

void ARD1939::SetPreferredAddress(byte v86) {
  j1939Stat.nSAPreferred = v86;
}

void ARD1939::SetAddressRange(byte nAddrBottom, byte nAddrTop) {
  j1939Stat.addrBottom = nAddrBottom;
  j1939Stat.addrTop = nAddrTop;
}

void ARD1939::SetNAME(long identityNumber, int nManufacturerCode, byte nFunctionInstance, byte nECUInstance,
                      byte nFunction, byte nVehicleSystem, byte nVehicleSystemInstance, byte nIndustryGroup, byte nArbitraryAddressCapable) {
  j1939Name[NAME_ID] = (byte)(identityNumber & 0xFF);
  j1939Name[1] = (byte)((identityNumber >> 8) & 0xFF);
  j1939Name[2] = (byte)(((nManufacturerCode << 5) & 0xFF) | (identityNumber >> 16));
  j1939Name[3] = (byte)(nManufacturerCode >> 3);
  j1939Name[4] = (byte)((nFunctionInstance << 3) | nECUInstance);
  j1939Name[5] = (byte)(nFunction);
  j1939Name[6] = (byte)(nVehicleSystem << 1);
  j1939Name[7] = (byte)((nArbitraryAddressCapable << 7) | (nIndustryGroup << 4) | (nVehicleSystemInstance));
}

byte ARD1939::SetMessageFilter(long re_lPGN) {
  byte re_nResult;
  int re_i;
  re_nResult = ERR;
  if ((re_lPGN & 0x00FF00) == PGN_REQUEST)
    re_lPGN = PGN_REQUEST;
  if (isFilterActive(re_lPGN) == true)
    re_nResult = OK;
  else {
    for (re_i = 0; re_i < MSGFILTERS; re_i++) {
      if (re_pFilters[re_i].bActive == false)  // this is an available slot
      {
        re_pFilters[re_i].bActive = true;
        re_pFilters[re_i].re_lPGN = re_lPGN;
        re_nResult = OK;
        break;
      }
    }
  }
  return re_nResult;  // if we get here, there are no available slots
}

void ARD1939::DeleteMessageFilter(long re_lPGN) {
  int re_i;
  if ((re_lPGN & 0x00FF00) == PGN_REQUEST)
    re_lPGN = PGN_REQUEST;
  for (re_i = 0; re_i < MSGFILTERS; re_i++) {
    if (re_pFilters[re_i].re_lPGN == re_lPGN) {
      re_pFilters[re_i].bActive = false;
      re_pFilters[re_i].re_lPGN = 0;
      break;
    }
  }
}

bool ARD1939::isFilterActive(long re_lPGN) {
  bool re_bResult;
  int re_i;
  re_bResult = false;
  if ((re_lPGN & 0x00FF00) == PGN_REQUEST)
    re_lPGN = PGN_REQUEST;
  for (re_i = 0; re_i < MSGFILTERS; re_i++) {
    if (re_pFilters[re_i].bActive == true
        && re_pFilters[re_i].re_lPGN == re_lPGN) {
      re_bResult = true;
      break;
    }
  }
  return re_bResult;
}

#if TRANSPORT_PROTOCOL == 1
// called from Operate()
byte ARD1939::processTransportProtocol(long re_lPGN, byte* re_pMsg, int re_nMsgLen, byte re_nDestAddr, byte re_nSrcAddr, byte re_nPriority) {
  byte v94;
  int nPointer;
  v94 = OK;
  UNUSED(re_nPriority);
  UNUSED(re_nMsgLen);

  if (v33.TRANSPORT_PROTOCOL_MODE == TRANSPORT_PROTOCOL_2 && v33.running == true) {
    if (v33.v22 == false) {
      msgBuf[0] = d04;
      msgBuf[1] = (byte)(v33.re_nMsgLen & 0xFF);
      msgBuf[2] = (byte)(v33.re_nMsgLen >> 8);
      msgBuf[3] = v33.v28;
      msgBuf[4] = 0xFF;
      msgBuf[5] = (byte)(v33.re_lPGN & 0x0000FF);
      msgBuf[6] = (byte)((v33.re_lPGN & 0x00FF00) >> 8);
      msgBuf[7] = (byte)(v33.re_lPGN >> 16);
      v94 = Transmit(d38, TPCM_PGN, v33.v26, GLOBALADDRESS, &msgBuf[0], 8);
      tpTimer1.timeout = v51;
      tpTimer1.running = true;
      v33.v22 = true;
    } else {
      if (tpTimer1.expired == true) {
        nPointer = v33.v29 * 7;
        v63[0] = ++v33.v29;
        //          for(re_i = 0; re_i < 7; re_i++)
        //            v63[re_i+1] = v33.re_pFullMessage[nPointer + re_i];
        memcpy(v63 + 1, v33.re_pFullMessage, 7);  // TEST
        v94 = Transmit(d40, re_TPDT_PGN, v33.v26, GLOBALADDRESS, &v63[0], 8);
        if (v33.v29 == v33.v28) {
          f11(d12);
        } else {
          tpTimer1.timeout = v51;
          tpTimer1.running = true;
          tpTimer1.expired = false;
        }
      }
    }
  }
  if (re_lPGN == TPCM_PGN && re_pMsg[0] == d04 && v33.TRANSPORT_PROTOCOL_MODE == TRANSPORT_PROTOCOL_0
      && v33.v32 == false) {
    v33.re_lPGN = (((long)re_pMsg[7]) << 16) + (((long)re_pMsg[6]) << 8) + (long)re_pMsg[5];
    if (isFilterActive(v33.re_lPGN) == true) {
      v33.re_nMsgLen = (int)re_pMsg[1] + ((int)(re_pMsg[2]) << 8);
      if (v33.re_nMsgLen > J1939_MSGLEN) {
        f11(d12);  // send an error response message?
      } else {
        v33.TRANSPORT_PROTOCOL_MODE = TRANSPORT_PROTOCOL_1;
        v33.running = true;
        v33.v26 = re_nSrcAddr;
        v33.v27 = re_nDestAddr;
        v33.v28 = re_pMsg[3];
        v33.v29 = 0;
        tpTimer2.timeout = v52;
        tpTimer2.running = true;
      }
    } else
      v33.re_lPGN = 0;
  }
  if (v33.TRANSPORT_PROTOCOL_MODE == TRANSPORT_PROTOCOL_1 && tpTimer2.expired == true) {
    f11(d12);
  }

  if (v33.TRANSPORT_PROTOCOL_MODE == TRANSPORT_PROTOCOL_1 && v33.running == true
      && re_lPGN == re_TPDT_PGN && re_nSrcAddr == v33.v26 && re_nDestAddr == v33.v27) {
    nPointer = ((int)re_pMsg[0] - 1) * 7;
    //    for(re_i = 1; re_i < 8; re_i++)
    //      v33.re_pFullMessage[nPointer++] = re_pMsg[re_i]; //TODO does this work for multi-packet receives?
    memcpy(v33.re_pFullMessage + nPointer, re_pMsg, 7);  //TEST
    if (++v33.v29 == v33.v28) {
      f11(d13);
      v33.v32 = true;
    }
  }
  if (v34.TRANSPORT_PROTOCOL_MODE == TRANSPORT_PROTOCOL_2 && v34.running == true
      && re_lPGN == TPCM_PGN && re_pMsg[0] == TP_CONN_ABORT) {
    f12(d12);
  }

  if (v34.TRANSPORT_PROTOCOL_MODE == TRANSPORT_PROTOCOL_2 && v34.running == true
      && re_lPGN == TPCM_PGN && re_pMsg[0] == d07) {
    f12(d12);
  }
  if (v34.TRANSPORT_PROTOCOL_MODE == TRANSPORT_PROTOCOL_2 && v34.running == true) {
    if (v34.v23 == false) {
      msgBuf[0] = TPCM_RTS_CONTROL_BYTE;
      msgBuf[1] = (byte)(v34.re_nMsgLen & 0xFF);
      msgBuf[2] = (byte)(v34.re_nMsgLen >> 8);
      msgBuf[3] = v34.v28;
      msgBuf[4] = 0xFF;
      msgBuf[5] = (byte)(v34.re_lPGN & 0x0000FF);
      msgBuf[6] = (byte)((v34.re_lPGN & 0x00FF00) >> 8);
      msgBuf[7] = (byte)(v34.re_lPGN >> 16);
      v94 = Transmit(d38, TPCM_PGN, v34.v26, v34.v27, &msgBuf[0], 8);
      tpTimer4.timeout = responseTics;
      tpTimer4.running = true;
      v34.v23 = true;
    } else {
      if (tpTimer4.expired == true) {
        msgBuf[0] = TP_CONN_ABORT;
        msgBuf[1] = TP_TIMEOUT;
        msgBuf[2] = 0xFF;
        msgBuf[3] = 0xFF;
        msgBuf[4] = 0xFF;
        msgBuf[5] = (byte)(v34.re_lPGN & 0x0000FF);
        msgBuf[6] = (byte)((v34.re_lPGN & 0x00FF00) >> 8);
        msgBuf[7] = (byte)(v34.re_lPGN >> 16);
        v94 = Transmit(d38, TPCM_PGN, v34.v26, v34.v27, &msgBuf[0], 8);
        f12(d12);
      }
      if (re_lPGN == TPCM_PGN && re_nDestAddr == v34.v26 && re_pMsg[0] == TPCM_CTS_CONTROL_BYTE) {
        _resetTimeout(&tpTimer4);
        minPacketsTimer.timeout = minPacketsTics;
        minPacketsTimer.running = true;
        v34.v24 = true;
      }
      if (v34.v24 == true && minPacketsTimer.expired == true) {
        nPointer = v34.v29 * 7;
        v63[0] = ++v34.v29;
        //          for(re_i = 0; re_i < 7; re_i++)
        //            v63[re_i+1] = v34.re_pFullMessage[nPointer + re_i];
        memcpy(v63 + 1, v34.re_pFullMessage + nPointer, 7);  //TEST
        v94 = Transmit(d40, re_TPDT_PGN, v34.v26, v34.v27, &v63[0], 8);
        if (v34.v29 == v34.v28) {
          _resetTimeout(&minPacketsTimer);
          tpTimer8.timeout = responseAfterRtsTics;
          tpTimer8.running = true;
        } else {
          minPacketsTimer.timeout = v51;
          minPacketsTimer.running = true;
          minPacketsTimer.expired = false;
        }
      }
      if (tpTimer8.expired == true)  // this is being reached after a successful CTS is transmitted, but the last three bytes are 00 00 00. I think it's a bug.
      {
        msgBuf[0] = TP_CONN_ABORT;
        msgBuf[1] = TP_TIMEOUT;
        msgBuf[2] = 0xFF;
        msgBuf[3] = 0xFF;
        msgBuf[4] = 0xFF;
        msgBuf[5] = (byte)(v34.re_lPGN & 0x0000FF);
        msgBuf[6] = (byte)((v34.re_lPGN & 0x00FF00) >> 8);
        msgBuf[7] = (byte)(v34.re_lPGN >> 16);
        v94 = Transmit(d38, TPCM_PGN, v34.v26, v34.v27, &msgBuf[0], 8);
        f12(d12);
      }
    }
  }
  if (re_lPGN == TPCM_PGN && re_nDestAddr == j1939Stat.nSourceAddress && re_pMsg[0] == TPCM_RTS_CONTROL_BYTE) {
    int v77;
    v77 = (int)re_pMsg[1] + ((int)(re_pMsg[2]) << 8);
    if (v34.TRANSPORT_PROTOCOL_MODE != TRANSPORT_PROTOCOL_0 || v77 > J1939_MSGLEN || v34.v32 == true) {
      msgBuf[0] = TP_CONN_ABORT;
      if (v34.re_nMsgLen > J1939_MSGLEN)
        msgBuf[1] = TP_LACKING_RESOURCES;
      else
        msgBuf[1] = TP_NODE_IN_ANOTHER_SESSION;
      msgBuf[2] = 0xFF;
      msgBuf[3] = 0xFF;
      msgBuf[4] = 0xFF;
      msgBuf[5] = re_pMsg[5];
      msgBuf[6] = re_pMsg[6];
      msgBuf[7] = re_pMsg[7];
      v94 = Transmit(d38, TPCM_PGN, re_nDestAddr, re_nSrcAddr, &msgBuf[0], 8);
    } else {
      v34.re_lPGN = (((long)re_pMsg[7]) << 16) + (((long)re_pMsg[6]) << 8) + (long)re_pMsg[5];
      if (isFilterActive(v34.re_lPGN) == true) {
        v34.TRANSPORT_PROTOCOL_MODE = TRANSPORT_PROTOCOL_1;
        v34.running = true;
        v34.v24 = true;
        v34.v26 = re_nSrcAddr;
        v34.v27 = re_nDestAddr;
        v34.v28 = re_pMsg[3];
        v34.v29 = 0;
        v34.re_nMsgLen = (int)re_pMsg[1] + ((int)(re_pMsg[2]) << 8);
        //        for(re_i = 0; re_i < 8; re_i++)
        //          msgBuf[re_i] = re_pMsg[re_i];
        memcpy(msgBuf, re_pMsg, 8);  //TEST
        msgBuf[0] = TPCM_CTS_CONTROL_BYTE;
        msgBuf[1] = msgBuf[3];
        msgBuf[2] = 1;
        msgBuf[3] = 0xFF;
        v94 = Transmit(d38, TPCM_PGN, re_nDestAddr, re_nSrcAddr, &msgBuf[0], 8);
        tpTimer6.timeout = betweenPacketsTics;
        tpTimer6.running = true;
      } else  // we ignore every transport message not in the filter
      {
        msgBuf[0] = TP_CONN_ABORT;
        msgBuf[1] = TP_LACKING_RESOURCES;
        msgBuf[2] = 0xFF;
        msgBuf[3] = 0xFF;
        msgBuf[4] = 0xFF;
        msgBuf[5] = re_pMsg[5];
        msgBuf[6] = re_pMsg[6];
        msgBuf[7] = re_pMsg[7];
        v94 = Transmit(d38, TPCM_PGN, re_nDestAddr, re_nSrcAddr, &msgBuf[0], 8);
      }
    }
  }
  if (v34.TRANSPORT_PROTOCOL_MODE == TRANSPORT_PROTOCOL_1 && v34.running == true) {
    if (tpTimer6.expired == true) {
      f12(d12);
      msgBuf[0] = TP_CONN_ABORT;
      msgBuf[1] = TP_TIMEOUT;
      msgBuf[2] = 0xFF;
      msgBuf[3] = 0xFF;
      msgBuf[4] = 0xFF;
      msgBuf[5] = (byte)(v34.re_lPGN & 0x0000FF);
      msgBuf[6] = (byte)((v34.re_lPGN & 0x00FF00) >> 8);
      msgBuf[7] = (byte)(v34.re_lPGN >> 16);
      v94 = Transmit(d38, TPCM_PGN, v34.v27, v34.v26, &msgBuf[0], 8);
    }
    if (re_lPGN == re_TPDT_PGN && re_nDestAddr == v34.v27 && re_nSrcAddr == v34.v26) {
      nPointer = ((int)re_pMsg[0] - 1) * 7;                    // advance pointer to the next set of 7 bytes based on the sequence number in the first byte of the message
                                                               //        for(re_i = 1; re_i < 8; re_i++)
                                                               //          v34.re_pFullMessage[nPointer++] = re_pMsg[re_i]; // copy the actual 7 bytes into the FullMessage buffer
      memcpy(v34.re_pFullMessage + nPointer + 1, re_pMsg, 7);  //TEST
      if (++v34.v29 == v34.v28) {
        msgBuf[0] = d07;
        msgBuf[1] = (byte)(v34.re_nMsgLen & 0x00FF);
        msgBuf[2] = (byte)((v34.re_nMsgLen & 0x00FF) >> 8);
        msgBuf[3] = v34.v28;
        msgBuf[4] = 0xFF;
        msgBuf[5] = (byte)(v34.re_lPGN & 0x0000FF);
        msgBuf[6] = (byte)((v34.re_lPGN & 0x00FF00) >> 8);
        msgBuf[7] = (byte)(v34.re_lPGN >> 16);
        v94 = Transmit(d38, TPCM_PGN, v34.v27, v34.v26, &msgBuf[0], 8);
        f12(d13);
        v34.v32 = true;
      }
    }
  }
  return v94;
}

#if TRANSPORT_PROTOCOL_XMIT == 1
byte ARD1939::rtsCtsTransmit(byte re_nPriority, long re_lPGN, byte nSourceAddress, byte re_nDestAddress, byte* re_pData, int re_nDataLen)  // re_DataLen was v77
{
  UNUSED(re_nPriority);

  byte j1939Status;
  int re_i;
  struct re_j1939_t* re_j1939bam;
  j1939Status = OK;
  if (re_nDestAddress != GLOBALADDRESS)
    re_j1939bam = &v34;
  else
    re_j1939bam = &v33;
  if (re_j1939bam->TRANSPORT_PROTOCOL_MODE != TRANSPORT_PROTOCOL_0 || re_nDataLen > J1939_MSGLEN)
    j1939Status = ERR;
  else {
    //    for(re_i = 0; re_i < re_nDataLen; re_i++) // TEST
    //     re_j1939bam->re_pFullMessage[re_i] = re_pData[re_i]; //copy the data to re_j1939bam
    memcpy(re_j1939bam, re_pData, re_nDataLen);
    for (re_i = re_nDataLen; re_i < (re_nDataLen + 7); re_i++) {
      if (re_i >= J1939_MSGLEN) break;
      re_j1939bam->re_pFullMessage[re_i] = 0xFF;
    }
    re_j1939bam->re_lPGN = re_lPGN;
    re_j1939bam->re_nMsgLen = re_nDataLen;
    re_j1939bam->v26 = nSourceAddress;
    re_j1939bam->v27 = re_nDestAddress;
    re_i = re_nDataLen;
    re_j1939bam->v28 = 0;
    while (re_i > 0) {
      re_i = re_i - 7;
      re_j1939bam->v28++;
    }
    re_j1939bam->v29 = 0;
    re_j1939bam->TRANSPORT_PROTOCOL_MODE = TRANSPORT_PROTOCOL_2;
    re_j1939bam->running = true;
  }
  return j1939Status;
}
#endif

void ARD1939::f11(byte v90) {
  if (v90 == d12) {
    v33.TRANSPORT_PROTOCOL_MODE = TRANSPORT_PROTOCOL_0;
    v33.running = false;
    v33.v22 = false;
    v33.re_lPGN = 0;
    v33.v26 = GLOBALADDRESS;
    v33.v27 = GLOBALADDRESS;
    v33.v28 = 0;
    v33.v29 = 0;
    v33.re_nMsgLen = 0;
    v33.v32 = false;
  }
  _resetTimeout(&tpTimer1);
  _resetTimeout(&tpTimer2);
}

void ARD1939::f12(byte v90) {
  if (v90 == d12) {
    v34.TRANSPORT_PROTOCOL_MODE = TRANSPORT_PROTOCOL_0;
    v34.running = false;
    v34.v23 = false;
    v34.v24 = false;
    v34.re_lPGN = 0;
    v34.v26 = GLOBALADDRESS;
    v34.v27 = GLOBALADDRESS;
    v34.v28 = 0;
    v34.v29 = 0;
    v34.re_nMsgLen = 0;
    v34.v32 = false;
  }
  _resetTimeout(&minPacketsTimer);
  _resetTimeout(&tpTimer4);
  _resetTimeout(&tpTimer5);
  _resetTimeout(&tpTimer6);
  _resetTimeout(&tpTimer7);
  _resetTimeout(&tpTimer8);
  _resetTimeout(&tpTimer9);
}

#endif
