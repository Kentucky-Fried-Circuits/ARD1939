/* ARD1939VERSION AND DEBUG need to be set for your sketch. */
// Program Version
// -----------------------------------------------
// 0 - ARD1939-Uno
// 1 - ARD1939-Uno/TP
// 2 - ARD1939-Mega
// 3 - Custom ARD1939-Nano with Saft_6t
#define ARD1939VERSION                          3

// Debugger Settings
#define DEBUG                                  0  // 1 to enable debugging output to serial monitor

#define SA_PREFERRED                      	128
#define ADDRESSRANGEBOTTOM                	129
#define ADDRESSRANGETOP                   	242  // 243 is the preferred address of the Saft 6TE battery, P/N 209188

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
#define NAME_ARBITRARY_ADDRESS_CAPABLE    	0x01 // 1 - yes, 2 - no

// System Settings
#define SYSTEM_TIME                             1    // Milliseconds per tic (or vice versa)

