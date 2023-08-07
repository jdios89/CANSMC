# CANSMC

Driver to control Integrated stepper motor controllers with Torque Control over CAN bus. 

# Protocol 

The driver works under the protocol CiA 402 for CAN bus controlled motors. 

# Board 

The driver was tested on a [Teensy 3.6](https://www.pjrc.com/store/teensy36.html)

It can control over 128 different devices under CAN bus. 

Serice Data Object were also implemented to achieve control loops of up to 1 KHz. 

Process Data Objects were used to configure the Transmit and Receive Service Data Objects. 

More info in [CAN protocol](https://www.can-cia.org/can-knowledge/)
