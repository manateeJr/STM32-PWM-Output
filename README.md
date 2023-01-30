# STM32-PWM-Output
Output compare functionality of general purpose timers on the STM32 board (ARM Cortex M4 MCU)

This is a school project for Fontys, Semester 3, Embedded Software. The problem in detail, as well as my take on it in the form of a report can be found in this folder. 
Additionally, there is a video link down below.

I'm particularly excited about this project, because I creatively used TIM2 to handle the PWM output on the pulsing LED, but also as a basic timer in the interrupt handler, meaning I also use it to accurately measure time. I achieved that by using convenient values for the prescaler and auto-reload value registers. 

Video demonstration: https://www.youtube.com/watch?v=97JiJO4MTds&feature=youtu.be
