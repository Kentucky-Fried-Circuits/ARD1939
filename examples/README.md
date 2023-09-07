= Example supporting ESP-IDF
All this currently does is establish itself on the J1939 bus and receive messages, printing a lot of debugging info (at ESP log level DEBUG)

== Hardware
- Requires 05-1000193 REV B

== Known Issues
- Change receive to a message queue
- Change Transmit to a message queue
- Remove LED Test