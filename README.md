# MEG421 Practicals
Code for the PIC32 MCU for the Mechatronics 421 practical projects

The practical consisted of 3 different aspects; each addressed seperately.

The descriptions below were adapted from the instruction sheets (written by Dr. Theunis Botha):

## Practical 1
A sensor value should be recorded by a PIC32MM, the output should be displayed by means
of the R/G/B LED on the PIC32MM Curiosity board. The input can be scaled between 0-
100% and the output displayed as:

| % Input | LED Operation |
|:-------:|:-------------:|
|    0    |      Off      |
|    20   |      Blue     |
|    40   |   Blue-Green  |
|    60   |     Green     |
|    80   |   Green-Red   |
|   100   |      Red      |

For the purpose of this practical, the potentiometer on the PIC32MM Curiosity Dev Board was used as a sensor.

## Practical 2

The aim of the practical is to gain practical experience of the circuits discussed during this module.
1.  Using a PIC32MM curiosity board develop a program which can uses PWM to control
    the speed of a DC-motor. Using a CPP module control either pin RC3 or RD1 to control
    the brightness of the Red or Green LED on the PIC32MM curiosity board. Parts will be
    supplied
2.  After the program works build a circuit using a N-Channel (STP36NF06L) to control a
    DC-motor’s speed. Parts will be supplied
3.  Using a P-Channel (IRF5305PBF) , N-Channel (STP36NF06L) and 1kΩ resistor to
    control a DC-motor’s speed. The P-Channel must supply current to the motor. Parts will
    be supplied
    
## Practical 3

The aim of this practical is to create a feedback controller of which the set point can be adjusted.
The controller is to control the distance of a linear actuator between 25mm and 225mm. The
feedback is obtained from a potentiometer on the actuator that provides a voltage based on the
distance of the actuator.
The set point should be adjustable by using the potentiometer on the PIC32MM Curiosity board
connected to AN14. The conversion from volts [V] to the set point value in millimetre [mm] can
be assumed linear equation with the following 2 points.

|     | Voltage | Set Point Distance [mm] |
|:---:|:-------:|:-----------------------:|
| Min |    0    |            25           |
| Max |   3.3   |           225           |

The feedback sensor has the following calibration from volts to distance in millimetres:

$Distance[mm] = 74(Volts) − 28.09$

In order to control the motorspeed two signals are required
1.  Direction Pin: The digital direction pin tells the linear actuator to moves out or in. A
    value of 1 (HIGH) denotes the motor moves out and a value of 0 (LOW) indicates the
    motor moves in.
2.  Moto Speed Pin: This analogue pin is used to control the speed of the motor. A higher
    voltage to this spin makes the motor move faster. A value of 0V stops motor motion.

The analogue speed control voltage should be obtained from the 5bit DAC output (See main
documentation on DAC). The voltage can be controlled, in 100mV increments, from 0V to 3.3V.
The voltage will be amplified by 3 using a LM358, such that 3.3V=9.9V to obtain higher motor speeds. The direction pin can be from any digital output pin. The tuning of the controller is not as
important as the motor speed is relatively slow and should be relatively easy to obtain a good
gain values.

In summary create a program which samples AN14 as set point, any other analogue in for the
distance sensor. Determine the error and output the actuator speed and direction using the DAC
output and digital pin respectively.

A PID controller was implemented to control the motion of the actuator. Althought the full controller was included in the code, the integral and derivate gains (K_I and K_D) were set to zero, resulting in only a proportional term.
