# STM32F4xx_Servo_PWM<br>
Simple PWM 50Hz Servo with STM32F411RE6 CMIS without HAL<br><br>

Creating a simple pwm signal with a period of 50Hz and a fixed duty cycle.<br> 
Using a Nucleo-64 Board with a STM32F411RE6 together with a SGS90 servo for testing.<br>
Using the external oscillator of 8 MHz and internal PLL<br><br>

Position - Pulse length<br>
0°  - 1ms <br>
90° - 1.5ms<br>
180° - 2ms (not possible with the SGS90 only with a hardware fix) <br>

Main file is located here:
STM32F4xx_Servo_PWM/Core/Src/main.c
STM32F4xx_Servo_PWM/Core/Inc/main.h


 
