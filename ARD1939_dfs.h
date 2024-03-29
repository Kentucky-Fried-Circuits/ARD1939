/* ARD1939VERSION AND DEBUG need to be set for your sketch. */
// Program Version
// -----------------------------------------------
// 0 - ARD1939-Uno
// 1 - ARD1939-Uno/TP
// 2 - ARD1939-Mega
// 3 - Custom ARD1939-Nano with Saft_6t
#define ARD1939VERSION                          0

// CAN contoller. Choose one, maybe define before you include this file.
#define ARD_TWAI // ESP-IDF twai driver
//#define ARD_MCP_CAN
#if (!defined(ARD_MCP_CAN) && !defined(ARD_TWAI))
#error Either ARD_MCP_CAN or ARD_TWAI must be defined to specify your CAN controller
#endif

// Debugger Settings
#ifdef DEBUG
#undef DEBUG
#endif
#define DEBUG                                  2  // 1 to enable debugging output to serial monitor

#define SA_PREFERRED                      	8 
#define ADDRESSRANGEBOTTOM                	8
#define ADDRESSRANGETOP                   	127  // 80 is HyPR 6000; we'll avoid it. See connectors spreadsheet for Solar Stik assignments.

#define GLOBALADDRESS                    	255
#define NULLADDRESS                      	254

// NAME Fields Default Use the Solar Stik Identity Number if one is ever assigned
#define NAME_IDENTITY_NUMBER              	0xFFFFFF
#define NAME_MANUFACTURER_CODE            	0xFFF //  269 is Saft, 3 is reserved for R&D
#define NAME_FUNCTION_INSTANCE            	0
#define NAME_ECU_INSTANCE                 	0x01
#define NAME_FUNCTION                     	0x04 // 0x04 battery pack monitor
#define NAME_RESERVED                     	0
#define NAME_VEHICLE_SYSTEM               	0x7F
#define NAME_VEHICLE_SYSTEM_INSTANCE      	0
#define NAME_INDUSTRY_GROUP               	0x01 // 0x01 on highway
#define NAME_ARBITRARY_ADDRESS_CAPABLE    	0x01 // single bit. 1 - yes, 0 - cannot alter w/o intervention of external process

// System Settings
#define SYSTEM_TIME                             1    // Milliseconds per tic (or vice versa)

