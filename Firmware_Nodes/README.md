# Firmware for IO boards
Firmware is written and compiled with STM32CubeIDE.

For now every node has it's own firmware: 3 nodes -> 3 firmware folders. As different machines can be run on the same playfield (and therefore the same nodes) an extra subdirectory was created for every machine. (e.g. RollerGames for the RG playfield with the RG game-code. Additionally the JudgeDredd_on_RollerGames for the RG playfield with the JD game-code and mapping)

Another folder contains the bootloader firmware witch is the same for all nodes.

Furthermore there are Python (V3) scripts for updating the nodes from the Raspberry Pi.
