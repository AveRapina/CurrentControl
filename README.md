## Synopsis

Voltage to current control (V-I) using an AVR microcontroller.  
The need to define an contant load current independent of it's impedance and supply voltage is an major demand in inductive loads such as magnetic brakes.  
The system uses an PID controller to keep track of the reference current and the feedback series resistor to automaticaly adjust the PWM value for the switching MOSFET.  
This can be seen as an step down but instead of voltage control, the current is the main objective.

## Features 

User can program the range of the current (min -max) to be controled regarding the reference input (0-10V).  



## Videos/Images

![](./Images/SchThumbnail.jpg)
Proteus schematics.
![](./Images/DiagramThumbnail.jpg)
Reference voltage vs. load current.


## Contributors

Main Developer :HSO  
Email: hugo(dot)soares(at)fe(dot)up(dot)pt
