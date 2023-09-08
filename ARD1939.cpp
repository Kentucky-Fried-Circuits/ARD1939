// BA has done some reverse engineering. Variable names starting with "re_" have been reverse engineered
// variables ending in "Maybe" may or may not be named well

#include <arduino.h>
#include <stdlib.h>
#include <inttypes.h>
// #include "mcp_can.h" // should be included with ARD1939.h if needed
#include "ARD1939.h"
#ifdef ARD_MCP_CAN
#include <SPI.h>
#endif
#ifdef ARD_TWAI
#include <esp_check.h>
#include <esp_log.h>
#include "driver/twai.h"
#endif

#define d49 10

#define d50 8

#define PGN_REQUEST 0x00EA00
// #define PGN_REQUEST_GLOBAL         	0x00EAFF
#define d46 0x00
#define re_PF_REQUEST 0xEA
#define d48 0x00

#define PGN_ADDRESS_CLAIM 0x00EE00
#define PGN_ADDRESS_CLAIM_BROADCAST 0x00EEFF // not used?
#define CANNOT_CLAIM_SA_LSB 0x00
#define CANNOT_CLAIM_SA_2ND 0xEE
#define CANNOT_CLAIM_SA_MSB 0x00
#define d28 6

#define d33 0x00EE00
#define d34 0x00FED8

#define d35 0x00ECFF
#define TPCM_PGN 0x00EC00
#define d37 0xEC
#define d38 7

#define re_TPDT_PGN 0x00EB00 // TP.DT PGN
#define d40 7

#define d41 32
#define TPCM_RTS_CONTROL_BYTE 16             // TP.CM_RTS control byte
#define TPCM_CTS_CONTROL_BYTE 17             // TP.CM_CTS control byte
#define re_TPCM_ENDOFMSG_ACK_CONTROL_BYTE 19 // TP.CM_EndOfMsgACK control byte
#define TP_CONN_ABORT 255                    // TP.Conn_Abort control byte

#define LOWEST_PRIORITY 255

#define NAME_ID 0

#define UNUSED(x) (void)(x) // this macro is used to supress unused-parameter warnings where I intend to leave them unused

#if DEBUG != 0
#ifdef ARD_MCP_CAN
#include <SoftwareSerial.h>
SoftwareSerial mySerial(A0, A1);
#endif
#endif

struct re_structFilter
{
  bool bActive;
  long re_lPGN;
};
re_structFilter re_pFilters[MSGFILTERS];

unsigned char j1939Name[] =
    {
        (byte)(NAME_IDENTITY_NUMBER & 0xFF),
        (byte)((NAME_IDENTITY_NUMBER >> 8) & 0xFF),
        (byte)((((long)NAME_MANUFACTURER_CODE << 5) & 0xFF) | (NAME_IDENTITY_NUMBER >> 16)),
        (byte)(NAME_MANUFACTURER_CODE >> 3),
        (byte)((NAME_FUNCTION_INSTANCE << 3) | NAME_ECU_INSTANCE),
        (byte)(NAME_FUNCTION),
        (byte)(NAME_VEHICLE_SYSTEM << 1),
        (byte)((NAME_ARBITRARY_ADDRESS_CAPABLE << 7) | (NAME_INDUSTRY_GROUP << 4) | (NAME_VEHICLE_SYSTEM_INSTANCE))};

#define re_NUM_TP_PGNS 4
long re_TP_PGNS[] =
    {
        PGN_ADDRESS_CLAIM,
        d34,
        TPCM_PGN,
        re_TPDT_PGN};

struct j1939Stat_t
{
  bool v07;
  byte canOperateMaybe; // should be a boolean, because it's only assigned booleans
  bool nextTimeTryAddrRange;
  bool v10;
  bool addressClaimFailedMaybe;
  byte nSAPreferred;
  byte addrBottom;
  byte addrTop;
  bool checkingAddrRange;
  byte myAddr;
  byte nSourceAddress;
};
struct j1939Stat_t j1939Stat;

#if TRANSPORT_PROTOCOL == 1
#define d01 0
#define d02 1
#define d03 2

struct re_j1939_t
{
  byte v20;
  bool activeMaybe;
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

byte msgBuf[] = {0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00};
byte v63[8];
#endif

struct sTimer v38;
struct sTimer v39;

#if TRANSPORT_PROTOCOL == 1
struct sTimer v40;
struct sTimer v41;

struct sTimer v42;
struct sTimer v43;
struct sTimer v44;
struct sTimer v45;
struct sTimer v46;
struct sTimer v47;
struct sTimer v48;
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

int v60[] = {8, 9, 10, 12, 15};
byte v61;

byte v64;
#ifdef ARD_MCP_CAN
extern byte canInit(MCP_CAN *);
extern byte canCheckError(MCP_CAN *);
extern byte canTransmit(long, unsigned char *, int, MCP_CAN *);
extern byte canReceive(long *, unsigned char *, int *, MCP_CAN *);
#endif
#ifdef ARD_TWAI
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static SemaphoreHandle_t transmit_sem;
static const char *TAG = "ARD1939";
#endif

#ifdef ARD_MCP_CAN
    ARD1939::ARD1939(byte _CS)
{
  _CAN0 = new MCP_CAN(_CS); // Set CS to pin 9/10
  /*	SPICS = _CS;  // the MCP_CAN constructor already does this. why do it again?
      pinMode(SPICS, OUTPUT);
      MCP2515_UNSELECT();
    */
}
#endif
#ifdef ARD_TWAI
ARD1939::ARD1939(twai_general_config_t *g_config, twai_timing_config_t *t_config)
{
  _g_config = g_config;
  _t_config = t_config;
  transmit_sem = xSemaphoreCreateBinary();
}
#endif

    byte ARD1939::GetAddressClaimed()
{
  return nAddressClaimed;
}

byte ARD1939::Init(int nSystemTime)
{
  int re_i;
  resetTimeoutMaybe(&v38);
  resetTimeoutMaybe(&v39);
  timeout250msTics = timeout250ms / nSystemTime;
  v50 = d15 / nSystemTime;
  v61 = 0;
  v60[0] = (int)8 / nSystemTime;
  v60[1] = (int)9 / nSystemTime;
  v60[2] = (int)10 / nSystemTime;
  v60[3] = (int)12 / nSystemTime;
  v60[4] = (int)15 / nSystemTime;
  j1939Stat.v07 = false;
  j1939Stat.canOperateMaybe = false;
  j1939Stat.nextTimeTryAddrRange = false;
  j1939Stat.v10 = false;
  j1939Stat.addressClaimFailedMaybe = false;
  j1939Stat.nSAPreferred = NULLADDRESS;
  j1939Stat.addrBottom = NULLADDRESS;
  j1939Stat.addrTop = NULLADDRESS;
  j1939Stat.checkingAddrRange = false;
  j1939Stat.myAddr = NULLADDRESS;
  j1939Stat.nSourceAddress = NULLADDRESS;
  for (re_i = 0; re_i < MSGFILTERS; re_i++)
  {
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
#ifdef ARD_MCP_CAN
  return canInit(_CAN0);
#endif
#ifdef ARD_TWAI
  // Initialize the CAN controller
  ESP_LOGD(TAG, "Init(): nSystemTime:%d", nSystemTime);
  ESP_ERROR_CHECK(twai_driver_install(_g_config, _t_config, &f_config));
  ESP_RETURN_ON_ERROR(twai_start(), TAG, "Init():failed to start");
  return ESP_OK;
#endif
}

void ARD1939::Terminate(void)
{
#ifdef ARD_MCP_CAN
  Init(d15 / v50); // d15/v50 is a hack to recover the original nSystemTime.
#endif
#ifdef ARD_TWAI
  twai_stop();
  twai_driver_uninstall();
#endif
}

/**
* @brief the main j1939 function. Calls the receive function and handles J1939 protocol as required
* All marameters are populated by Operate() for return to the caller.
* @param nMsgId is the J1939 protocol
* @param lPGN the pgn
* @param pMsg pointer to message data array
* @param nMsgLen number of bytes in pMsg
* @param nDestAddr target recpient of message
* @param nSrcAddr sender of message
* @param nPriority j1939 message priority
*
* @return byte, one of ADDRESSCLAIM_INIT, ADDRESSCLAIM_INPROGRESS, ADDRESSCLAIM_FINISHED, NORMALDATATRAFFIC, ADDRESSCLAIM_FAILED
FIXME for TWAI
**/
byte ARD1939::Operate(byte *nMsgId, long *lPGN, byte *pMsg, int *nMsgLen, byte *nDestAddr, byte *nSrcAddr, byte *nPriority)
{
  byte j1939Status;
  byte compareIdsRetVal;
  f05();
  *nMsgId = receive(lPGN, &pMsg[0], nMsgLen, nDestAddr, nSrcAddr, nPriority);
  if (*nMsgId == J1939_MSG_APP)
  {
    if (*nDestAddr != j1939Stat.nSourceAddress && *nDestAddr != GLOBALADDRESS)
      *nMsgId = J1939_MSG_NETWORKDATA;
  }
  // DEBUG_ESPLOG2(TAG, "canOp:%d", j1939Stat.canOperateMaybe);
  if (j1939Stat.canOperateMaybe == true)
  {
    j1939Status = NORMALDATATRAFFIC;
    switch (*lPGN)
    {
    case PGN_REQUEST:
      if (*nMsgId == J1939_MSG_PROTOCOL)
      {
        if (pMsg[0] == CANNOT_CLAIM_SA_MSB && pMsg[1] == CANNOT_CLAIM_SA_2ND && pMsg[2] == CANNOT_CLAIM_SA_LSB)
        {
          if (*nDestAddr == GLOBALADDRESS)
          {
            Transmit(d28, PGN_ADDRESS_CLAIM, j1939Stat.nSourceAddress, GLOBALADDRESS,
                     &j1939Name[NAME_ID], 8);
          }
          else if (*nDestAddr == j1939Stat.nSourceAddress)
          {
            Transmit(d28, PGN_ADDRESS_CLAIM, j1939Stat.nSourceAddress, *nSrcAddr,
                     &j1939Name[NAME_ID], 8);
          }
        }
      }
      break;

    case PGN_ADDRESS_CLAIM:
      nAddressClaimed = *nSrcAddr;               // save so upper levels of OSI stack know someone new is on the bus
      if (*nSrcAddr == j1939Stat.nSourceAddress) // we received a message from our address. We need to tell them to try a different address by sending our own Address Claimed message
      {
        compareIdsRetVal = compareIds(&pMsg[0], &j1939Name[NAME_ID]);
        switch (compareIdsRetVal)
        {
        case 0:                                                                 // The bugger has the exact same ID as me.
          DEBUG_PRINTLN(F("~298:Op():PGN==PGN_ADDRESS_CLAIMED:compareIds==0")); // we fail first here when another 0x80 comes online
          // instead of failing, we should tell them to bugger off, just like case 2
          Transmit(d28, PGN_ADDRESS_CLAIM, j1939Stat.nSourceAddress,
                   GLOBALADDRESS, &j1939Name[NAME_ID], 8);
          break;
          // #if TRANSPORT_PROTOCOL == 1
          // f11(d12);
          // f12(d12);
          // #endif
          // Transmit(d28, PGN_ADDRESS_CLAIM, NULLADDRESS, GLOBALADDRESS, &j1939Name[NAME_ID], 8);
          // j1939Stat.canOperateMaybe = false;
          // j1939Stat.addressClaimFailedMaybe = true;
          // j1939Status = ADDRESSCLAIM_FAILED;
          break;
        case 1:
#if TRANSPORT_PROTOCOL == 1
          f11(d12);
          f12(d12);
#endif
          j1939Stat.v07 = false;
          j1939Stat.canOperateMaybe = false;
          claimAddressMaybe(*nSrcAddr, &j1939Name[NAME_ID]);
          break;
        case 2:
          Transmit(d28, PGN_ADDRESS_CLAIM, j1939Stat.nSourceAddress,
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
    if (*nMsgId == J1939_MSG_NONE)
    {
      if (v33.v32 == true)
      {
        *lPGN = v33.re_lPGN;
        *nMsgLen = v33.re_nMsgLen;
        *nDestAddr = v33.v27;
        *nSrcAddr = v33.v26;
        *nPriority = LOWEST_PRIORITY;
        //    		  for(re_i = 0; re_i < v33.re_nMsgLen; re_i++)
        //    			  pMsg[re_i] = v33.re_pFullMessage[re_i];
        memcpy(pMsg, v33.re_pFullMessage, v33.re_nMsgLen); // TEST
        f11(d12);
        *nMsgId = J1939_MSG_APP;
      }
      else if (v34.v32 == true)
      {
        *lPGN = v34.re_lPGN;
        *nMsgLen = v34.re_nMsgLen;
        *nDestAddr = v34.v27;
        *nSrcAddr = v34.v26;
        *nPriority = LOWEST_PRIORITY;
        //    		  for(re_i = 0; re_i < v34.re_nMsgLen; re_i++)
        //  			  pMsg[re_i] = v34.re_pFullMessage[re_i];
        memcpy(pMsg, v34.re_pFullMessage, v34.re_nMsgLen); // TEST
        f12(d12);
        *nMsgId = J1939_MSG_APP;
      }
    }
#endif
  }
  else if (j1939Stat.addressClaimFailedMaybe == true)
  {
    DEBUG_PRINTLN(F("~363:Op():stat.claimFailed")); // we fail here on subsequent calls to Operate()
    DEBUG_ESPLOG1(TAG, "~363:Op()");
    j1939Status = ADDRESSCLAIM_FAILED;
    switch (*lPGN)
    {
    case PGN_REQUEST:
      if (*nMsgId == J1939_MSG_PROTOCOL)
      {
        if (pMsg[0] == CANNOT_CLAIM_SA_MSB && pMsg[1] == CANNOT_CLAIM_SA_2ND && pMsg[2] == CANNOT_CLAIM_SA_LSB)
        {
          if (*nDestAddr == GLOBALADDRESS)
          {
            v39.timeoutMaybe = v60[v61++];
            v39.activeMaybe = true;
            if (v61 > 4)
              v61 = 0;
          }
        }
      }
      break;
    case PGN_ADDRESS_CLAIM:
      break;
    case d34:
      break;
    }
    if (v39.expiredMaybe == true)
    {
      resetTimeoutMaybe(&v39);
      Transmit(d28, PGN_ADDRESS_CLAIM, NULLADDRESS, GLOBALADDRESS,
               &j1939Name[NAME_ID], 8);
    }
  }
  else
  {
    DEBUG_PRINTLN("claiming address");
    j1939Status = claimAddressMaybe(*nSrcAddr, &pMsg[0]);
  }
  return j1939Status;
}

byte ARD1939::claimAddressMaybe(byte myAddr, byte *v91)
{
  byte j1939Status;
  byte compareIdsRetVal;
  j1939Status = ADDRESSCLAIM_INPROGRESS;
  if (j1939Stat.addressClaimFailedMaybe == true)
  {
    DEBUG_PRINTLN(F("~409:claimAddr():stat.claimFailed"));
    j1939Status = ADDRESSCLAIM_FAILED;
  }
  else if (j1939Stat.canOperateMaybe == true)
  {
    j1939Status = ADDRESSCLAIM_FINISHED;
  }
  else if (j1939Stat.v10 == true)
  {
    if (v39.expiredMaybe == true)
    {
      resetTimeoutMaybe(&v39);
      Transmit(d28, PGN_ADDRESS_CLAIM, j1939Stat.myAddr, GLOBALADDRESS,
               &j1939Name[NAME_ID], 8);
      v38.timeoutMaybe = timeout250msTics;
      v38.activeMaybe = true;
      j1939Stat.v10 = false;
    }
  }
  else
  {
    if (j1939Stat.v07 == false)
    {
      if (getAnotherMyAddr() == true)
      {
        Transmit(d28, PGN_ADDRESS_CLAIM, j1939Stat.myAddr, GLOBALADDRESS,
                 &j1939Name[NAME_ID], 8);
        v38.timeoutMaybe = timeout250msTics;
        v38.activeMaybe = true;
        j1939Stat.v07 = true;
      }
      else
      {
        DEBUG_PRINTLN(F("~439:claimAddr():defaultStat:v07==false:getAnother==false"));
        Transmit(d28, PGN_ADDRESS_CLAIM, NULLADDRESS, GLOBALADDRESS, &j1939Name[NAME_ID], 8);
        j1939Stat.canOperateMaybe = false;
        j1939Stat.addressClaimFailedMaybe = true;
        j1939Status = ADDRESSCLAIM_FAILED;
      }
    }
    else
    {
      if (v38.expiredMaybe == true)
      {
        resetTimeoutMaybe(&v38);
        j1939Stat.nSourceAddress = j1939Stat.myAddr;
        j1939Stat.canOperateMaybe = true;
        j1939Status = ADDRESSCLAIM_FINISHED;
      }
      else
      {
#ifdef ARD_MCP_CAN
        if (canCheckError(_CAN0) == 1) // there's Maybe a problem
        {
          resetTimeoutMaybe(&v38);
          if (++v64 == d49)
          {
            DEBUG_PRINTLN(F("~6=462:claimAddr():else:!v07:!v38expired:canCheckError==1:v64==d49"));
            j1939Stat.canOperateMaybe = false;
            j1939Stat.addressClaimFailedMaybe = true;
            j1939Status = ADDRESSCLAIM_FAILED;
          }
          else
          {
            canInit(_CAN0);
            v39.timeoutMaybe = v60[v61++];
            v39.activeMaybe = true;
            if (v61 > 4)
              v61 = 0;
            j1939Stat.v10 = true;
          }
        }
        else
#endif
          v64 = 0;
        // TODO check for TWAI errors
        if (myAddr == j1939Stat.myAddr) // I'm the requestor of an address, and my address is claimed already
        {
          compareIdsRetVal = compareIds(&v91[0], &j1939Name[NAME_ID]);
          switch (compareIdsRetVal)
          {
          case 0:                                                                                              // my address is claimed by someone with the same ID as me. They told me to bugger off
            DEBUG_PRINTLN(F("~485:claimAddr():else:v07==false:!v38expired:myAddr=stat.myAddr:compareIds==0")); // identical addresses land here when I'm the second node.
            // instead of failing at this point, Ima get me another address by falling through to case 1
            // #if TRANSPORT_PROTOCOL == 1
            // f11(d12);
            // f12(d12);
            // #endif
            // Transmit(d28, PGN_ADDRESS_CLAIM, NULLADDRESS, GLOBALADDRESS, &j1939Name[NAME_ID], 8);
            // j1939Stat.canOperateMaybe = false;
            // j1939Stat.addressClaimFailedMaybe = true;
            // j1939Status = ADDRESSCLAIM_FAILED;
            // break;

          case 1:
#if TRANSPORT_PROTOCOL == 1
            f11(d12);
            f12(d12);
#endif
            if (getAnotherMyAddr() == true)
            {
              Transmit(d28, PGN_ADDRESS_CLAIM, j1939Stat.myAddr, GLOBALADDRESS,
                       &j1939Name[NAME_ID], 8);
              v38.timeoutMaybe = timeout250msTics;
              v38.activeMaybe = true;
            }
            else
            {
              DEBUG_PRINTLN(F("~510:claimAddr():else:v07==false:!v38expired:myAddr=stat.myAddr:compareIds==1:getAnother==false"));

              Transmit(d28, PGN_ADDRESS_CLAIM, NULLADDRESS, GLOBALADDRESS,
                       &j1939Name[NAME_ID], 8);
              j1939Stat.canOperateMaybe = false;
              j1939Stat.addressClaimFailedMaybe = true;
              j1939Status = ADDRESSCLAIM_FAILED;
            }
            break;

          case 2:
            Transmit(d28, PGN_ADDRESS_CLAIM, j1939Stat.myAddr, GLOBALADDRESS,
                     &j1939Name[NAME_ID], 8);
            v38.timeoutMaybe = timeout250msTics;
            v38.activeMaybe = true;
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
  if (j1939Stat.nSAPreferred == NULLADDRESS)
    j1939Stat.nextTimeTryAddrRange = true;
  if (j1939Stat.nextTimeTryAddrRange == false)
  {
    j1939Stat.myAddr = j1939Stat.nSAPreferred;
    j1939Stat.nextTimeTryAddrRange = true;
  }
  else
  { // get next address from range
    if (j1939Stat.addrBottom == NULLADDRESS || j1939Stat.addrTop == NULLADDRESS)
    { // there is no valid range, so fail
      returnVal = false;
    }
    else
    {
      if (j1939Stat.myAddr == NULLADDRESS || j1939Stat.checkingAddrRange == false)
      { // we haven't tried anything yet, or ?
        j1939Stat.myAddr = j1939Stat.addrBottom;
        j1939Stat.checkingAddrRange = true;
      }
      else
      {
        if (j1939Stat.myAddr < j1939Stat.addrTop)
          j1939Stat.myAddr++;
        else
          returnVal = false;
      }
      if (j1939Stat.myAddr == j1939Stat.nSAPreferred)
      {
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
byte ARD1939::compareIds(byte *pMsg, byte *pNameID)
{
  byte i;
  for (i = 8; i > 0; i--)
  {
    if (pMsg[i - 1] != pNameID[i - 1])
    {
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
byte ARD1939::receive(long *re_lPGN, byte *re_pMsg, int *re_nMsgLen, byte *re_nDestAddr, byte *re_nSrcAddr, byte *re_nPriority)
{
  long v78;
  long lID;
  *re_lPGN = 0;
  *re_nMsgLen = 0;
  *re_nDestAddr = NULLADDRESS;
  *re_nSrcAddr = NULLADDRESS;
  *re_nPriority = 0;
#ifdef ARD_MCP_CAN
  if (canReceive(&lID, &re_pMsg[0], re_nMsgLen, _CAN0) == 0) // we have a message
  {
#endif
#ifdef ARD_TWAI
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, 0) == ESP_OK) // we are effectively polling. TODO optimize
    {
      lID = rx_msg.identifier;
      *re_nMsgLen = rx_msg.data_length_code;
      for (int i = 0; i < *re_nMsgLen; i++)
      {
        re_pMsg[i] = rx_msg.data[i];
      }
#endif
      v78 = lID & 0x1C000000;
      *re_nPriority = (byte)(v78 >> 26);
      *re_lPGN = lID & 0x01FFFF00;
      *re_lPGN = *re_lPGN >> 8;
      *re_nSrcAddr = (byte)(lID & 0x000000FF);
      *re_nDestAddr = GLOBALADDRESS;
      if (isPeerToPeer(*re_lPGN) == true)
      {
        *re_nDestAddr = (byte)(*re_lPGN & 0xFF); // mask out just the destination address
        *re_lPGN = *re_lPGN & 0x01FF00;          // mask out just the data page bit and first byte of PGN
      }
      if (isTransportProtocol(re_lPGN, &re_pMsg[0]) == true)
        return J1939_MSG_PROTOCOL;
      else
        return J1939_MSG_APP;
    }
    else
      return J1939_MSG_NONE;
  }

#ifdef ARD_TWAI
  static void twai_transmit_task(void *arg)
  {
    // pull message from arg
    const twai_message_t *tx_msg = static_cast<twai_message_t *>(arg);
    ESP_LOGD(TAG, "twai_transmit_task() id:0x%x extd:%d len:%d msg[]:0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x",
             tx_msg->extd, tx_msg->identifier, tx_msg->data_length_code, tx_msg->data[0], tx_msg->data[1], tx_msg->data[2], tx_msg->data[3], tx_msg->data[4], tx_msg->data[5], tx_msg->data[6], tx_msg->data[7]);
    twai_transmit(tx_msg, portMAX_DELAY);
    ESP_LOGD(TAG, "twai_transmit_task() complete");
    xSemaphoreGive(transmit_sem); // Rudimentary synch. to prevent spawning lots of tasks. TODO change to a queue.
    vTaskDelete(NULL);
  }
#endif

  /**
   * @brief send a CAN message using canTransmit or rtsCtsTransmit, as appropriate
   * @return the return 0 on success, 1 (ERR) on failure (or ESP_ERR_TIMEOUT)
   */
  byte ARD1939::Transmit(byte re_nPriority, long re_lPGN, byte nSourceAddress, uint8_t re_nDestAddress, const uint8_t *pData, int nDataLen)
  {
    long lID;
    if (nDataLen > J1939_MSGLEN)
      return ERR;
    lID = ((long)re_nPriority << 26) + (re_lPGN << 8) + (long)nSourceAddress;
    if (isPeerToPeer(re_lPGN) == true)
      lID = lID | ((long)re_nDestAddress << 8);
    if (nDataLen > 8)
#if TRANSPORT_PROTOCOL_XMIT == 1
      return rtsCtsTransmit(re_nPriority, re_lPGN, nSourceAddress, re_nDestAddress, pData, nDataLen);
#else
    return ERR;
#endif
    else
    #ifdef ARD_MCP_CAN
      return canTransmit(lID, (uint8_t *)pData, nDataLen, _CAN0);
    #endif
    #ifdef ARD_TWAI
    // spawn a transmit task and continue
    ESP_RETURN_ON_ERROR(xSemaphoreTake(transmit_sem, pdMS_TO_TICKS(1000)), "TAG", "Trasmit aborted: xSemaphoreTake failed"); // TODO #define this wait time
    twai_message_t tx_msg;
    tx_msg.extd = 1;
    tx_msg.rtr = 0;
    tx_msg.identifier = lID;
    tx_msg.data_length_code = nDataLen;
    memcpy(tx_msg.data, pData, nDataLen);
    ESP_LOGD(TAG, "Transmit():transmitting");
    esp_err_t ret = twai_transmit(&tx_msg, portMAX_DELAY);
     // esp_err_t ret = xTaskCreate(twai_transmit_task, "TWAI_tx", 4096, &tx_msg, 7, NULL); TODO change to a queue and a task
    ESP_LOGD(TAG, "Transmit() result:%s(%d)", esp_err_to_name(ret), ret);
    return ret;
#endif
  }

  void ARD1939::f05(void)
  {
    if (v38.activeMaybe == true && v38.expiredMaybe == false)
    {
      if (--v38.timeoutMaybe == 0)
        v38.expiredMaybe = true;
    }
    if (v39.activeMaybe == true && v39.expiredMaybe == false)
      if (--v39.timeoutMaybe == 0)
        v39.expiredMaybe = true;
#if TRANSPORT_PROTOCOL == 1
    if (v40.activeMaybe == true && v40.expiredMaybe == false)
      if (--v40.timeoutMaybe == 0)
        v40.expiredMaybe = true;
    if (v41.activeMaybe == true && v41.expiredMaybe == false)
      if (--v41.timeoutMaybe == 0)
        v41.expiredMaybe = true;
    if (v42.activeMaybe == true && v42.expiredMaybe == false)
      if (--v42.timeoutMaybe == 0)
        v42.expiredMaybe = true;
    if (v43.activeMaybe == true && v43.expiredMaybe == false)
      if (--v43.timeoutMaybe == 0)
        v43.expiredMaybe = true;
    if (v44.activeMaybe == true && v44.expiredMaybe == false)
      if (--v44.timeoutMaybe == 0)
        v44.expiredMaybe = true;
    if (v45.activeMaybe == true && v45.expiredMaybe == false)
      if (--v45.timeoutMaybe == 0)
        v45.expiredMaybe = true;
    if (v46.activeMaybe == true && v46.expiredMaybe == false)
      if (--v46.timeoutMaybe == 0)
        v46.expiredMaybe = true;
    if (v47.activeMaybe == true && v47.expiredMaybe == false)
      if (--v47.timeoutMaybe == 0)
        v47.expiredMaybe = true;
    if (v48.activeMaybe == true && v48.expiredMaybe == false)
      if (--v48.timeoutMaybe == 0)
        v48.expiredMaybe = true;
#endif
  }

  void ARD1939::resetTimeoutMaybe(struct sTimer * v75)
  {
    v75->timeoutMaybe = 0;
    v75->activeMaybe = false;
    v75->expiredMaybe = false;
  }

  bool ARD1939::isTransportProtocol(long *re_lPGN, byte *re_pMsg)
  {
    bool re_bResult;
    byte re_i;
    byte re_nPF;
    byte re_nPS;
    re_bResult = false;
    re_nPF = (byte)((*re_lPGN & 0x00FF00) >> 8); // PF field of ID
    re_nPS = (byte)(*re_lPGN & 0x0000FF);
    switch (re_nPF)
    {
    case re_PF_REQUEST:
      if (re_nPS == GLOBALADDRESS)
      {
        if (re_pMsg[0] == CANNOT_CLAIM_SA_MSB && re_pMsg[1] == CANNOT_CLAIM_SA_2ND && re_pMsg[2] == CANNOT_CLAIM_SA_LSB)
          re_bResult = true;
      }
      else
      {
        *re_lPGN = *re_lPGN & 0x00FF00;
        if (re_pMsg[0] == CANNOT_CLAIM_SA_MSB && re_pMsg[1] == CANNOT_CLAIM_SA_2ND && re_pMsg[2] == CANNOT_CLAIM_SA_LSB)
          re_bResult = true;
      }
      break;

    default:
      for (re_i = 0; re_i < re_NUM_TP_PGNS - 1; re_i++)
      {
        if (*re_lPGN == re_TP_PGNS[re_i])
        {
          re_bResult = true;
          break;
        }
      }
      break;
    }
    return re_bResult;
  }

  bool ARD1939::isPeerToPeer(long re_lPGN)
  {
    if (re_lPGN > 0 && re_lPGN <= 0xEFFF)
      return true; // peer-to-peer PGN
    if (re_lPGN > 0x10000 && re_lPGN <= 0x1EFFF)
      return true; // peer-to-peer PGN with data page bit set
    return false;
  }

  byte ARD1939::getSourceAddress(void)
  {
    return j1939Stat.nSourceAddress;
  }

  void ARD1939::SetPreferredAddress(byte v86)
  {
    j1939Stat.nSAPreferred = v86;
  }

  void ARD1939::SetAddressRange(byte nAddrBottom, byte nAddrTop)
  {
    j1939Stat.addrBottom = nAddrBottom;
    j1939Stat.addrTop = nAddrTop;
  }

  /*
     send - send a J1939 request
     Returns true if no errors detected in sending. The calling function must ensure the bus is in normal data mode.
  */
  byte ARD1939::begin(int sysTime, boolean toggleTermination)
  {
    byte status = Init(sysTime);
    DEBUG_PRINT(F("j9~786"));
    DEBUG_PRINTLN(status);
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
    // Note: All Tranport PGNs must be explicitly filtered *in*
    // SetMessageFilter(PGN_ECU_IDENTIFICATION_INFORMATION);  // we OPT IN to RTS CTS messages
    return status;
  }

  void ARD1939::SetNAME(long identityNumber, int nManufacturerCode, byte nFunctionInstance, byte nECUInstance,
                        byte nFunction, byte nVehicleSystem, byte nVehicleSystemInstance, byte nIndustryGroup, byte nArbitraryAddressCapable)
  {
    j1939Name[NAME_ID] = (byte)(identityNumber & 0xFF);
    j1939Name[1] = (byte)((identityNumber >> 8) & 0xFF);
    j1939Name[2] = (byte)(((nManufacturerCode << 5) & 0xFF) | (identityNumber >> 16));
    j1939Name[3] = (byte)(nManufacturerCode >> 3);
    j1939Name[4] = (byte)((nFunctionInstance << 3) | nECUInstance);
    j1939Name[5] = (byte)(nFunction);
    j1939Name[6] = (byte)(nVehicleSystem << 1);
    j1939Name[7] = (byte)((nArbitraryAddressCapable << 7) | (nIndustryGroup << 4) | (nVehicleSystemInstance));
  }

  byte ARD1939::SetMessageFilter(long re_lPGN)
  {
    byte re_nResult;
    int re_i;
    re_nResult = ERR;
    if ((re_lPGN & 0x00FF00) == PGN_REQUEST)
      re_lPGN = PGN_REQUEST;
    if (isFilterActive(re_lPGN) == true)
      re_nResult = OK;
    else
    {
      for (re_i = 0; re_i < MSGFILTERS; re_i++)
      {
        if (re_pFilters[re_i].bActive == false) // this is an available slot
        {
          re_pFilters[re_i].bActive = true;
          re_pFilters[re_i].re_lPGN = re_lPGN;
          re_nResult = OK;
          break;
        }
      }
    }
    return re_nResult; // if we get here, there are no available slots
  }

  void ARD1939::DeleteMessageFilter(long re_lPGN)
  {
    int re_i;
    if ((re_lPGN & 0x00FF00) == PGN_REQUEST)
      re_lPGN = PGN_REQUEST;
    for (re_i = 0; re_i < MSGFILTERS; re_i++)
    {
      if (re_pFilters[re_i].re_lPGN == re_lPGN)
      {
        re_pFilters[re_i].bActive = false;
        re_pFilters[re_i].re_lPGN = 0;
        break;
      }
    }
  }

  bool ARD1939::isFilterActive(long re_lPGN)
  {
    bool re_bResult;
    int re_i;
    re_bResult = false;
    if ((re_lPGN & 0x00FF00) == PGN_REQUEST)
      re_lPGN = PGN_REQUEST;
    for (re_i = 0; re_i < MSGFILTERS; re_i++)
    {
      if (re_pFilters[re_i].bActive == true && re_pFilters[re_i].re_lPGN == re_lPGN)
      {
        re_bResult = true;
        break;
      }
    }
    return re_bResult;
  }

#if TRANSPORT_PROTOCOL == 1
  // called from Operate()
  byte ARD1939::processTransportProtocol(long re_lPGN, byte *re_pMsg, int re_nMsgLen, byte re_nDestAddr, byte re_nSrcAddr, byte re_nPriority)
  {
    byte v94;
    int nPointer;
    v94 = OK;
    UNUSED(re_nPriority);
    UNUSED(re_nMsgLen);

    if (v33.v20 == d03 && v33.activeMaybe == true)
    {
      if (v33.v22 == false)
      {
        msgBuf[0] = d04;
        msgBuf[1] = (byte)(v33.re_nMsgLen & 0xFF);
        msgBuf[2] = (byte)(v33.re_nMsgLen >> 8);
        msgBuf[3] = v33.v28;
        msgBuf[4] = 0xFF;
        msgBuf[5] = (byte)(v33.re_lPGN & 0x0000FF);
        msgBuf[6] = (byte)((v33.re_lPGN & 0x00FF00) >> 8);
        msgBuf[7] = (byte)(v33.re_lPGN >> 16);
        v94 = Transmit(d38, TPCM_PGN, v33.v26, GLOBALADDRESS, &msgBuf[0], 8);
        v40.timeoutMaybe = v51;
        v40.activeMaybe = true;
        v33.v22 = true;
      }
      else
      {
        if (v40.expiredMaybe == true)
        {
          nPointer = v33.v29 * 7;
          v63[0] = ++v33.v29;
          //          for(re_i = 0; re_i < 7; re_i++)
          //            v63[re_i+1] = v33.re_pFullMessage[nPointer + re_i];
          memcpy(v63 + 1, v33.re_pFullMessage, 7); // TEST
          v94 = Transmit(d40, re_TPDT_PGN, v33.v26, GLOBALADDRESS, &v63[0], 8);
          if (v33.v29 == v33.v28)
          {
            f11(d12);
          }
          else
          {
            v40.timeoutMaybe = v51;
            v40.activeMaybe = true;
            v40.expiredMaybe = false;
          }
        }
      }
    }
    if (re_lPGN == TPCM_PGN && re_pMsg[0] == d04 && v33.v20 == d01 && v33.v32 == false)
    {
      v33.re_lPGN = (((long)re_pMsg[7]) << 16) + (((long)re_pMsg[6]) << 8) + (long)re_pMsg[5];
      if (isFilterActive(v33.re_lPGN) == true)
      {
        v33.re_nMsgLen = (int)re_pMsg[1] + ((int)(re_pMsg[2]) << 8);
        if (v33.re_nMsgLen > J1939_MSGLEN)
        {
          f11(d12); // send an error response message?
        }
        else
        {
          v33.v20 = d02;
          v33.activeMaybe = true;
          v33.v26 = re_nSrcAddr;
          v33.v27 = re_nDestAddr;
          v33.v28 = re_pMsg[3];
          v33.v29 = 0;
          v41.timeoutMaybe = v52;
          v41.activeMaybe = true;
        }
      }
      else
        v33.re_lPGN = 0;
    }
    if (v33.v20 == d02 && v41.expiredMaybe == true)
    {
      f11(d12);
    }

    if (v33.v20 == d02 && v33.activeMaybe == true && re_lPGN == re_TPDT_PGN && re_nSrcAddr == v33.v26 && re_nDestAddr == v33.v27)
    {
      nPointer = ((int)re_pMsg[0] - 1) * 7;
      //    for(re_i = 1; re_i < 8; re_i++)
      //      v33.re_pFullMessage[nPointer++] = re_pMsg[re_i]; //TODO does this work for multi-packet receives?
      memcpy(v33.re_pFullMessage + nPointer, re_pMsg, 7); // TEST
      if (++v33.v29 == v33.v28)
      {
        f11(d13);
        v33.v32 = true;
      }
    }
    if (v34.v20 == d03 && v34.activeMaybe == true && re_lPGN == TPCM_PGN && re_pMsg[0] == TP_CONN_ABORT)
    {
      f12(d12);
    }

    if (v34.v20 == d03 && v34.activeMaybe == true && re_lPGN == TPCM_PGN && re_pMsg[0] == d07)
    {
      f12(d12);
    }
    if (v34.v20 == d03 && v34.activeMaybe == true)
    {
      if (v34.v23 == false)
      {
        msgBuf[0] = TPCM_RTS_CONTROL_BYTE;
        msgBuf[1] = (byte)(v34.re_nMsgLen & 0xFF);
        msgBuf[2] = (byte)(v34.re_nMsgLen >> 8);
        msgBuf[3] = v34.v28;
        msgBuf[4] = 0xFF;
        msgBuf[5] = (byte)(v34.re_lPGN & 0x0000FF);
        msgBuf[6] = (byte)((v34.re_lPGN & 0x00FF00) >> 8);
        msgBuf[7] = (byte)(v34.re_lPGN >> 16);
        v94 = Transmit(d38, TPCM_PGN, v34.v26, v34.v27, &msgBuf[0], 8);
        v43.timeoutMaybe = responseTics;
        v43.activeMaybe = true;
        v34.v23 = true;
      }
      else
      {
        if (v43.expiredMaybe == true)
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
        if (re_lPGN == TPCM_PGN && re_nDestAddr == v34.v26 && re_pMsg[0] == TPCM_CTS_CONTROL_BYTE)
        {
          resetTimeoutMaybe(&v43);
          v42.timeoutMaybe = minPacketsTics;
          v42.activeMaybe = true;
          v34.v24 = true;
        }
        if (v34.v24 == true && v42.expiredMaybe == true)
        {
          nPointer = v34.v29 * 7;
          v63[0] = ++v34.v29;
          //          for(re_i = 0; re_i < 7; re_i++)
          //            v63[re_i+1] = v34.re_pFullMessage[nPointer + re_i];
          memcpy(v63 + 1, v34.re_pFullMessage + nPointer, 7); // TEST
          v94 = Transmit(d40, re_TPDT_PGN, v34.v26, v34.v27, &v63[0], 8);
          if (v34.v29 == v34.v28)
          {
            resetTimeoutMaybe(&v42);
            v47.timeoutMaybe = responseAfterRtsTics;
            v47.activeMaybe = true;
          }
          else
          {
            v42.timeoutMaybe = v51;
            v42.activeMaybe = true;
            v42.expiredMaybe = false;
          }
        }
        if (v47.expiredMaybe == true) // this is being reached after a successful CTS is transmitted, but the last three bytes are 00 00 00. I think it's a bug.
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
    if (re_lPGN == TPCM_PGN && re_nDestAddr == j1939Stat.nSourceAddress && re_pMsg[0] == TPCM_RTS_CONTROL_BYTE)
    {
      int v77;
      v77 = (int)re_pMsg[1] + ((int)(re_pMsg[2]) << 8);
      if (v34.v20 != d01 || v77 > J1939_MSGLEN || v34.v32 == true)
      {
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
      }
      else
      {
        v34.re_lPGN = (((long)re_pMsg[7]) << 16) + (((long)re_pMsg[6]) << 8) + (long)re_pMsg[5];
        if (isFilterActive(v34.re_lPGN) == true)
        {
          v34.v20 = d02;
          v34.activeMaybe = true;
          v34.v24 = true;
          v34.v26 = re_nSrcAddr;
          v34.v27 = re_nDestAddr;
          v34.v28 = re_pMsg[3];
          v34.v29 = 0;
          v34.re_nMsgLen = (int)re_pMsg[1] + ((int)(re_pMsg[2]) << 8);
          //        for(re_i = 0; re_i < 8; re_i++)
          //          msgBuf[re_i] = re_pMsg[re_i];
          memcpy(msgBuf, re_pMsg, 8); // TEST
          msgBuf[0] = TPCM_CTS_CONTROL_BYTE;
          msgBuf[1] = msgBuf[3];
          msgBuf[2] = 1;
          msgBuf[3] = 0xFF;
          v94 = Transmit(d38, TPCM_PGN, re_nDestAddr, re_nSrcAddr, &msgBuf[0], 8);
          v45.timeoutMaybe = betweenPacketsTics;
          v45.activeMaybe = true;
        }
        else // we ignore every transport message not in the filter
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
    if (v34.v20 == d02 && v34.activeMaybe == true)
    {
      if (v45.expiredMaybe == true)
      {
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
      if (re_lPGN == re_TPDT_PGN && re_nDestAddr == v34.v27 && re_nSrcAddr == v34.v26)
      {
        nPointer = ((int)re_pMsg[0] - 1) * 7;                   // advance pointer to the next set of 7 bytes based on the sequence number in the first byte of the message
                                                                //        for(re_i = 1; re_i < 8; re_i++)
                                                                //          v34.re_pFullMessage[nPointer++] = re_pMsg[re_i]; // copy the actual 7 bytes into the FullMessage buffer
        memcpy(v34.re_pFullMessage + nPointer + 1, re_pMsg, 7); // TEST
        if (++v34.v29 == v34.v28)
        {
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
  byte ARD1939::rtsCtsTransmit(byte re_nPriority, long re_lPGN, byte nSourceAddress, byte re_nDestAddress, byte *pData, int nDataLen) // re_DataLen was v77
  {
    UNUSED(re_nPriority);

    byte j1939Status;
    int re_i;
    struct re_j1939_t *re_j1939bam;
    j1939Status = OK;
    if (re_nDestAddress != GLOBALADDRESS)
      re_j1939bam = &v34;
    else
      re_j1939bam = &v33;
    if (re_j1939bam->v20 != d01 || nDataLen > J1939_MSGLEN)
      j1939Status = ERR;
    else
    {
      //    for(re_i = 0; re_i < nDataLen; re_i++) // TEST
      //     re_j1939bam->re_pFullMessage[re_i] = pData[re_i]; //copy the data to re_j1939bam
      memcpy(re_j1939bam, pData, nDataLen);
      for (re_i = nDataLen; re_i < (nDataLen + 7); re_i++)
      {
        if (re_i >= J1939_MSGLEN)
          break;
        re_j1939bam->re_pFullMessage[re_i] = 0xFF;
      }
      re_j1939bam->re_lPGN = re_lPGN;
      re_j1939bam->re_nMsgLen = nDataLen;
      re_j1939bam->v26 = nSourceAddress;
      re_j1939bam->v27 = re_nDestAddress;
      re_i = nDataLen;
      re_j1939bam->v28 = 0;
      while (re_i > 0)
      {
        re_i = re_i - 7;
        re_j1939bam->v28++;
      }
      re_j1939bam->v29 = 0;
      re_j1939bam->v20 = d03;
      re_j1939bam->activeMaybe = true;
    }
    return j1939Status;
  }
#endif

  void ARD1939::f11(byte v90)
  {
    if (v90 == d12)
    {
      v33.v20 = d01;
      v33.activeMaybe = false;
      v33.v22 = false;
      v33.re_lPGN = 0;
      v33.v26 = GLOBALADDRESS;
      v33.v27 = GLOBALADDRESS;
      v33.v28 = 0;
      v33.v29 = 0;
      v33.re_nMsgLen = 0;
      v33.v32 = false;
    }
    resetTimeoutMaybe(&v40);
    resetTimeoutMaybe(&v41);
  }

  void ARD1939::f12(byte v90)
  {
    if (v90 == d12)
    {
      v34.v20 = d01;
      v34.activeMaybe = false;
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
    resetTimeoutMaybe(&v42);
    resetTimeoutMaybe(&v43);
    resetTimeoutMaybe(&v44);
    resetTimeoutMaybe(&v45);
    resetTimeoutMaybe(&v46);
    resetTimeoutMaybe(&v47);
    resetTimeoutMaybe(&v48);
  }

#endif
