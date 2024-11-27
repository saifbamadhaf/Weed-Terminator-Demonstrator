import serial

class ReadConveyor():

    def __init__(self):
        self.serialPort = serial.Serial(port = "/dev/ttyACM2", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

        while(True):
            serialString = ""                           # Used to hold data coming over UART

            # Wait until there is data waiting in the serial buffer
            if(self.serialPort.in_waiting > 0):

                # Read data out of the buffer until a carraige return / new line is found
                serialString = self.serialPort.readline()

                data = serialString.decode('Ascii')

                if (data.startswith("RS485 Master started")):
                    self.serialPort.write(b"RUN\n")
                else:
                    try:
                        distance = float(data)
                        print("Distance travelled: %f" % distance)
                    except:
                        print(data)

                # msg = String()
                # msg.data = 'Hello World: %d' % self.i
                # self.publisher_.publish(msg)
                # self.get_logger().info('Publishing: "%s"' % msg.data)
                # self.i += 1


def main(args=None):
    read_conveyor = ReadConveyor()

if __name__ == '__main__':
    main()