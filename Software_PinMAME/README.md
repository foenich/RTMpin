# Software related to PinMAME
A few files from the PinMAME project were adapted to communicate with the IO-boards (mainly s11.c).

Here is the link to the branch RTMpin (forked from PinMAME-Version of 5th of April 2021) and added neccessary changes for RTMPin  
https://github.com/foenich/pinmame/tree/RTMpin

Here is the link to the PinMAME project:  
[https://github.com/vpinball/pinmame](https://github.com/vpinball/pinmame)  

To make the DMD (Dot Matrix Display) running the excellent and well documentated library rpi-rgb-led-matrix was used:  
[https://github.com/hzeller/rpi-rgb-led-matrix](https://github.com/hzeller/rpi-rgb-led-matrix)  
To compile the RTMpin branch of PinMAME you also have to compile the rpi-rgb-led-matix library and make it visible to the system (e.g. copy librgbmatrix.so.1 and/or librgbmatrix.a to /usr/local/lib and run ldconfig)
