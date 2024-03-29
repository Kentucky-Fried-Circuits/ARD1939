// ------------------------------------------------------------------------
// J1939 CAN Connection
// ------------------------------------------------------------------------
#include <inttypes.h>
#include "mcp_can.h"
#include "ARD1939.h"

// ------------------------------------------------------------------------
// CAN message ring buffer setup
// ------------------------------------------------------------------------
#define CANMSGBUFFERSIZE 10
struct CANMsg
{
  long lID;
  unsigned char pData[8];
  int nDataLen;
};
CANMsg CANMsgBuffer[CANMSGBUFFERSIZE];
int nWritePointer;
int nReadPointer;

// ------------------------------------------------------------------------
// Initialize the CAN controller
// return 0 for success, 1 for failure
// ------------------------------------------------------------------------
byte canInit(MCP_CAN *CAN0)
{
  // Default settings
  nReadPointer = 0;
  nWritePointer = 0;
  
  // Initialize the CAN controller
  if(CAN0->begin(CAN_250KBPS) == 0)
    return 0;
  else return 1;

}// end canInitialize

// ------------------------------------------------------------------------
// Check CAN controller for error
// ------------------------------------------------------------------------
byte canCheckError(MCP_CAN *CAN0)
{
  if(CAN0->checkError() == 0)
    return 0;
  else return 1;
  
}// end canCheckError

// ------------------------------------------------------------------------
// Transmit CAN message
// ------------------------------------------------------------------------
byte canTransmit(long lID, unsigned char* pData, int nDataLen, MCP_CAN *CAN0)
{
  
  if(CAN0->sendMsgBuf(lID, CAN_EXTID, nDataLen, pData) == 0)
    return 0;
  else
    return 1;
  
}// end canTransmit

// ------------------------------------------------------------------------
// Receive CAN message
// ------------------------------------------------------------------------
byte canReceive(long* lID, unsigned char* pData, int* nDataLen, MCP_CAN *CAN0)
{
  // Declarations
  byte nCnt;

  // In case there is a message, put it into the buffer
  while(CAN0->checkReceive() == CAN_MSGAVAIL)
  {
    // Read the message buffer
    CAN0->readMsgBuf(&nCnt, &CANMsgBuffer[nWritePointer].pData[0]);
    CANMsgBuffer[nWritePointer].nDataLen = (int)nCnt;
    CANMsgBuffer[nWritePointer].lID = CAN0->getCanId();    
    
    if(++nWritePointer == CANMSGBUFFERSIZE)
      nWritePointer = 0;
    
  }

  // Check ring buffer for a message
  if(nReadPointer != nWritePointer)
  {
    // Read the next message buffer entry
    *nDataLen = CANMsgBuffer[nReadPointer].nDataLen;
    *lID = CANMsgBuffer[nReadPointer].lID;

//    for(int nIdx = 0; nIdx < *nDataLen; nIdx++)
//      pData[nIdx] = CANMsgBuffer[nReadPointer].pData[nIdx];
	memcpy(pData, CANMsgBuffer[nReadPointer].pData, *nDataLen); // TEST

    if(++nReadPointer == CANMSGBUFFERSIZE)
      nReadPointer = 0;
    
    return 0;
  }
  else return 1;

}// end canReceive 
