import rclpy
import serial

from rclpy.node import Node

from detection_interfaces.msg import Conveyor

BELT_LENGTH = 3.545

class ReadConveyor(Node):

    def __init__(self):
        super().__init__('read_conveyor')
        self.publisher_ = self.create_publisher(Conveyor, 'conveyor', 10)
        timer_period = 0.1  # seconds
        
        self.serialPort = serial.Serial(port = "/dev/ttyACM2", baudrate=115200,
                           bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)

        # distance(0x.xxx)|turns(x)
        separator = "|"

        # Tell the device connected over the serial port that we recevied the data!
        # The b at the beginning is used to indicate bytes!
        run_sent = False
        while(rclpy.ok()):
            serialString = ""                           # Used to hold data coming over UART

            # Wait until there is data waiting in the serial buffer
            if(self.serialPort.in_waiting > 0):

                # Read data out of the buffer until a carraige return / new line is found
                serialString = self.serialPort.readline()

                data = serialString.decode('Ascii')

                if (data.startswith("RS485 Master started")):
                    self.serialPort.write(b"RUN\n")
                    run_sent = True
                else:
                    if (run_sent):
                        try:
                            parts = data.split(separator)
                            distance = float(parts[0])
                            turns = int(parts[1])
                            msg = Conveyor()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.turns = turns
                            msg.distance_travelled = distance
                            self.publisher_.publish(msg)
                        except Exception as e:
                            self.get_logger().info('Unable to read data: %s' % data)


def main(args=None):
    rclpy.init(args=args)

    read_conveyor = ReadConveyor()

    rclpy.spin(read_conveyor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    read_conveyor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()