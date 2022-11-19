import serial
import time

# don't use zlib for CRC as in STM hardware it is calculated like CRC32-MPEG2 with LSB first
def create_table():
    a = []
    for i in range(256):
        k = i << 24;
        for _ in range(8):
            k = (k << 1) ^ 0x4c11db7 if k & 0x80000000 else k << 1
        a.append(k & 0xffffffff)
    return a

def crc32(bytestream, startval):
    crc = startval
    for byte in bytestream:
        lookup_index = ((crc >> 24) ^ byte) & 0xff
        crc = ((crc & 0xffffff) << 8) ^ crc_table[lookup_index]
    return crc


ser = serial.Serial('/dev/ttyUSB0', 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)
print (ser.portstr)       #check which port was really used

nodeNrAndLength = 0x0f #node address (1. nibble) and length (2. nibble); length 0xf becomes 16 in softwareupdate

try:

    with open('./rtmpin_node_0_401.bin', 'rb') as f:
        crc32of_f = 0xFFFFFFFF
        crc_table = create_table()
        frame16 = f.read(16)
        while frame16:
            missing_bytes = 16 - len(frame16) # 16 bytes per frame
            if missing_bytes != 0:
                frame16 += (bytes(missing_bytes)) #fill frame with '0's
            ser.write(bytes([nodeNrAndLength]))   #node address and length length 0xf becomes 16 in softwareupdate
            ser.write(frame16) 
            #rearange frame for crc calculation (LSB first):
            frame16_for_crc = [frame16[3],frame16[2],frame16[1],frame16[0],frame16[7],frame16[6],frame16[5],frame16[4],frame16[11],frame16[10],frame16[9],frame16[8],frame16[15],frame16[14],frame16[13],frame16[12]]
            crc32of_f = crc32(frame16_for_crc,crc32of_f)     
            frame16 = f.read(16)
            time.sleep(0.004) #wait for flash being programmed (2 ms required, say 4 ms as time.sleep is not precise)
        ser.write(bytes([nodeNrAndLength])) #start new frame for crc
        print(hex(crc32of_f))
        ser.write(crc32of_f.to_bytes(4, byteorder="little"))      
        ser.write(12) #fill frame with '0's
except IOError:
     print('Error While Opening the file!')  

ser.close()
