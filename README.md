This ARD1939 library is based on CopperHill Tech's ARD1939 library. 
DO NOT USE this version of the library for dual CAN bus boards without first back-porting the bug fixes in the AMMPS MEGA Board version of this library.

- 2023-02-20 moved to KFC Github and added init function for arbitrary addressing, which isn't yet working.
- 2021-11-24 fixed bug where two nodes with identical Name IDs would fail arbitrary address negotiation
- 2021-11-23 additional reverse engineering
- 2020-07-22 reverse-engineered variable names.
- ARD1939_dfs.h used to customize for application
- must pass CAN CS pin when instantiating

-BA

Usage
------
Application sets defaults in ARD1939_defs.h. Can define ARD1939VERSION to select pre-defined features and buffer sizes.  Otherwise the version 0 settings will be used, but they can be overwritten by defining any of
  TRANSPORT_PROTOCOL
  J1939_MSGLEN                        
  MSGFILTERS     
  TRANSPORT_PROTOCOL_XMIT

Application must pass the CS pin for the CAN driver chip when instantiating ARD1939. Uses mcp_can for a CAN driver. 

Application setup should call Init() to connect the CAN DRIVER, then j1939Init() to get an address on the j1939 network.
