This ARD1939 library is based on CopperHill Tech's ARD1939 library. 
DO NOT USE this version of the library for dual CAN bus boards without first back-porting the bug fixes in the AMMPS MEGA Board version of this library.

- 2021-11-24 fixed bug where two nodes with identical Name IDs would fail arbitrary address negotiation
- 2021-11-23 additional reverse engineering
- 2020-07-22 reverse-engineered variable names.
- ARD1939_dfs.h used to customize for application
- must pass CAN CS pin when instantiating

-BA