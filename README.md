# Elevator 2.0

Upgrade from previous Elevator logic simulation project: https://github.com/gapizaver/elevator-logic-stm32f4-arduino

Now running STM32F769IDISCOVERY with FreeRTOS

VIdeo demonstration: https://youtu.be/X4a3xXFS2QQ

## FreeRTOS

main.c consists of 4 tasks:

- **LCDInputTask**

Responsible for detecting touch input on LCD. Gets the touch input coordinates and check if any of the buttons have been pressed -> sets flags.

- **LCDDrawTask**

Responsible for drawing on the LCD. Loops through all buttons and draws them. 

- **UARTTask**

Responsible for communication with Arduino through UART protocol. Position of the elevator is send to Arduino and buttons pressed are received from Arduino.

- **elevatorTask**

Moores machine consisting of 4 states:

1. elevator has requests for current direction on the way
2. elevator has no requests for current direction on the way, but has request for the opposite direction on the way
3. change the direction of the elevator
4. opening and closing of the doors

