# STM32F4xx_Servo_PWM
Simple PWM 50Hz Servo with STM32F411RE6 CMIS without HAL

Creating a simple pwm signal with a period of 50Hz and a fixed duty cycle. 
Using a Nucleo-64 Board with a STM32F411RE6 together with a SGS90 servo for testing.
Using the external oscillator of 8 MHz and internal PLL

Position - Pulse length
0°  - 1ms 
90° - 1.5ms
180° - 2ms (not possible with the SGS90 only with a hardware fix
 
