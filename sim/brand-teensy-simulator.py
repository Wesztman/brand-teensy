import select
import serial

ser_receive = serial.Serial("/dev/pts/3", 9600, timeout=1)
ser_send = serial.Serial("/dev/pts/2", 9600, timeout=1)

while True:
    readable, _, _ = select.select([ser_send, ser_receive], [], [])

    if ser_receive in readable:
        print("Received: " + ser_receive.readline().strip().decode("ascii"))
        print("Sending response")
        ser_send.write(b"Response from teensy")