// Sammy Nayhouse (san6yhk)

#ifndef DEFINES_H_
#define DEFINES_H_
#include "ADCInit.h"

#define TRUE                                1
#define FALSE                               0

#define SCHEDULE_TIMER                      TIMER_A1
#define SCHEDULE_TIMER_PERIOD               100                 // 100 us period <-> 10 kHz
#define SCHEDULE_TIMER_CCR                  0
#define SCHEDULE_TIMER_IRQ                  SCHEDULE_TIMER_IRQ
#define SCHEDULE_TIMER_INTERRUPT            10
#define ISER_REGISTER                       0
#define SCHEDULE_TIMER_INTERRUPT_PRIORITY   2

#define GLOBAL_TIMER                        TIMER_A3
#define GLOBAL_TIMER_PERIOD                 12000               // (12000*83.33333 ns) <-> 1 kHz
#define GLOBAL_TIMER_CCR                    0
#define GLOBAL_TIMER_IRQ                    TA3_0_IRQHandler
#define GLOBAL_TIMER_INTERRUPT              14
#define GLOBAL_TIMER_INTERRUPT_PRIORITY     0

#define Ts_DISTANCE_SENSORS                 5                   // distance sensors sampling period
#define Ts_MOTOR_DECODERS                   1                   // rotary motor decoder FSM period
#define Ts_BUMP_SWITCHES                    1                   // bump switch read period

/*
#define PWM_DELTA                           1
#define T_PWM_RED_LED                       (2*PWM_DELTA)   // 1ms <-> 1 kHz
#define T_PWM_GREEN_LED                     (4*PWM_DELTA)   // 2ms <-> 500 Hz
#define T_PWM_BLUE_LED                      (8*PWM_DELTA)   // 4ms <-> 250 Hz
*/

#define NUMBER_OF_TASKS                     3
#define NUMBER_OF_DISTANCE_SENSORS          3
#define NUMBER_OF_BUMP_SWITCHES             6
#define NUMBER_OF_ROTARY_DECODERS           2
#define RIGHT_SENSOR_ID                     0   // Note that these are used to index the sensor reading and
#define CENTER_SENSOR_ID                    1   // distance arrays declared in main.c
#define LEFT_SENSOR_ID                      2
#define ADC_FLAG_SHIFT                      DISTANCE_SENSOR_LEFT_MEM // Largest ADC memory index - see PortPins.h

// Used to filter the sensor input buffer. Note that SENSOR_INPUT_BUFFER_LENGTH = 2^SENSOR_INPUT_AVERAGE_SHIFT
#define SENSOR_INPUT_AVERAGE_SHIFT          3
#define SENSOR_INPUT_BUFFER_LENGTH          8

// Length of output buffer string in main.c
#define OUTPUT_BUFFER_LENGTH                80

#endif /* DEFINES_H_ */

