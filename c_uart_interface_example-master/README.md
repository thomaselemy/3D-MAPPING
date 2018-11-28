# c_uart_interface_example-master
This is a simple MAVLink to UART interface example for *nix systems that can allow communication between Pixhawk and an offboard computer.

This example will receive one MAVLink message and send one MAVLink message.

Building
$ cd c_uart_interface_example/
$ make
Hardware Setup
Connect the USB programming cable to your Pixhawk.

If you want to be able to interact with this example in Pixhawk's NuttX shell, you'll need a Telemetry Radio or an FTDI developer's cable. See the Exploration section below for more detail.

Also Note: Using a UART (serial) connection should be preferred over using the USB port for flying systems. The reason being that the driver for the USB port is much more complicated, so the UART is a much more trusted port for flight-critical functions. To learn how this works though the USB port will be fine and instructive.

Execution
You have to pick a port name, try searching for it with


$ ls /dev/ttyACM* 
$ ls /dev/ttyUSB*
