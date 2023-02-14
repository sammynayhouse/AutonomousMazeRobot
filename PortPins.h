// Sammy Nayhouse (san6yhk)

#ifndef PORTPINS_H_
#define PORTPINS_H_

#include "msp.h"

#define NVIC_INTERRUPT_ENABLE(REG,BIT)      NVIC->ISER[REG] |= (0x00000001 << BIT)
#define NVIC_INTERRUPT_DISABLE(REG,BIT)     NVIC->ICER[REG] = (0x00000001 << BIT)
#define INTERRUPT_PRIORITY(PR)              (PR << 5)
#define PRIORITY_REGISTER(INT,PR)           NVIC->IP[INT] = ((NVIC->IP[INT]&0x00)|PR)
#define ADC14_CONVERSION_START_ADDR(REG)    (uint32_t)((REG<<ADC14_CTL1_CSTARTADD_OFS) & ADC14_CTL1_CSTARTADD_MASK)

// LED1
#define LED1_PORT                   P1
#define LED1_PIN                    0
#define LED1_BIT                    (0x01 << LED1_PIN)
#define SET_LED1_AS_AN_OUTPUT       LED1_PORT->DIR |= LED1_BIT
#define TURN_ON_LED1                LED1_PORT->OUT |= LED1_BIT
#define TURN_OFF_LED1               LED1_PORT->OUT &= ~LED1_BIT
#define TOGGLE_LED1                 LED1_PORT->OUT ^= LED1_BIT

// LED2 Red
#define LED2_RED_PORT               P2
#define LED2_RED_PIN                0
#define LED2_RED_BIT                (0x01 << LED2_RED_PIN)
#define SET_LED2_RED_AS_AN_OUTPUT   LED2_RED_PORT->DIR |= LED2_RED_BIT
#define TURN_ON_LED2_RED            LED2_RED_PORT->OUT |= LED2_RED_BIT
#define TURN_OFF_LED2_RED           LED2_RED_PORT->OUT &= ~LED2_RED_BIT
#define TOGGLE_LED2_RED             LED2_RED_PORT->OUT ^= LED2_RED_BIT

// LED2 Green
#define LED2_GREEN_PORT             P2
#define LED2_GREEN_PIN              1
#define LED2_GREEN_BIT              (0x01 << LED2_GREEN_PIN)
#define SET_LED2_GREEN_AS_AN_OUTPUT LED2_GREEN_PORT->DIR |= LED2_GREEN_BIT
#define TURN_ON_LED2_GREEN          LED2_GREEN_PORT->OUT |= LED2_GREEN_BIT
#define TURN_OFF_LED2_GREEN         LED2_GREEN_PORT->OUT &= ~LED2_GREEN_BIT
#define TOGGLE_LED2_GREEN           LED2_GREEN_PORT->OUT ^= LED2_GREEN_BIT

// LED2 Blue
#define LED2_BLUE_PORT              P2
#define LED2_BLUE_PIN               2
#define LED2_BLUE_BIT               (0x01 << LED2_BLUE_PIN)
#define SET_LED2_BLUE_AS_AN_OUTPUT  LED2_BLUE_PORT->DIR |= LED2_BLUE_BIT
#define TURN_ON_LED2_BLUE           LED2_BLUE_PORT->OUT |= LED2_BLUE_BIT
#define TURN_OFF_LED2_BLUE          LED2_BLUE_PORT->OUT &= ~LED2_BLUE_BIT
#define TOGGLE_LED2_BLUE            LED2_BLUE_PORT->OUT ^= LED2_BLUE_BIT

// Line Sensors
#define LINE_SENSOR_CTRL_EVEN_PORT          P5
#define LINE_SENSOR_CTRL_EVEN_PIN           3
#define LINE_SENSOR_CTRL_EVEN_BIT           (0x01 << LINE_SENSOR_CTRL_EVEN_PIN) // 0x08
#define LINE_SENSOR_CTRL_EVEN_DIR           LINE_SENSOR_CTRL_EVEN_PORT->DIR
#define LINE_SENSOR_CTRL_EVEN_OUT           LINE_SENSOR_CTRL_EVEN_PORT->OUT
#define SET_LINE_SENSOR_CTRL_EVEN_AS_OUTPUT LINE_SENSOR_CTRL_EVEN_DIR |= LINE_SENSOR_CTRL_EVEN_BIT
#define TURN_ON_LINE_SENSOR_CTRL_EVEN       LINE_SENSOR_CTRL_EVEN_OUT |= LINE_SENSOR_CTRL_EVEN_BIT
#define TURN_OFF_LINE_SENSOR_CTRL_EVEN      LINE_SENSOR_CTRL_EVEN_OUT &= ~LINE_SENSOR_CTRL_EVEN_BIT

#define LINE_SENSOR_CTRL_ODD_PORT           P9
#define LINE_SENSOR_CTRL_ODD_PIN            2
#define LINE_SENSOR_CTRL_ODD_BIT            (0x01 << LINE_SENSOR_CTRL_ODD_PIN) // 0x04
#define LINE_SENSOR_CTRL_ODD_DIR            LINE_SENSOR_CTRL_ODD_PORT->DIR
#define LINE_SENSOR_CTRL_ODD_OUT            LINE_SENSOR_CTRL_ODD_PORT->OUT
#define SET_LINE_SENSOR_CTRL_ODD_AS_OUTPUT  LINE_SENSOR_CTRL_ODD_DIR |= LINE_SENSOR_CTRL_ODD_BIT
#define TURN_ON_LINE_SENSOR_CTRL_ODD        LINE_SENSOR_CTRL_ODD_OUT |= LINE_SENSOR_CTRL_ODD_BIT
#define TURN_OFF_LINE_SENSOR_CTRL_ODD       LINE_SENSOR_CTRL_ODD_OUT &= ~LINE_SENSOR_CTRL_ODD_BIT

#define REFLECTANCE_SENSOR_PORT             P7
#define REFLECTANCE_SENSOR_DIR              REFLECTANCE_SENSOR_PORT->DIR
#define REFLECTANCE_SENSOR_OUT              REFLECTANCE_SENSOR_PORT->OUT
#define REFLECTANCE_SENSOR_IN               REFLECTANCE_SENSOR_PORT->IN
#define SET_REFLECTANCE_SENSOR_AS_INPUT     REFLECTANCE_SENSOR_DIR = 0x00
#define SET_REFLECTANCE_SENSOR_AS_OUTPUT    REFLECTANCE_SENSOR_DIR = 0xFF

// Definitions for bump switches
#define BUMP_PORT   P4
#define BUMP5_BIT   0x80
#define BUMP4_BIT   0x40
#define BUMP3_BIT   0x20
#define BUMP2_BIT   0x08
#define BUMP1_BIT   0x04
#define BUMP0_BIT   0x01
#define BUMP_BITS   (BUMP5_BIT|BUMP4_BIT|BUMP3_BIT|BUMP2_BIT|BUMP1_BIT|BUMP0_BIT)
#define SET_BUMP_SWITCHES_AS_INPUTS     BUMP_PORT->DIR &= ~BUMP_BITS
#define ENABLE_PULL_RESISTORS           BUMP_PORT->REN |= BUMP_BITS
#define PULL_UP_RESISTORS               BUMP_PORT->OUT |= BUMP_BITS
#define FALLING_EDGE_INTERRUPTS         BUMP_PORT->IES |= BUMP_BITS
#define RISING_EDGE_INTERRUPTS          BUMP_PORT->IES &= ~BUMP_BITS
#define ENABLE_BUMP_SWITCHES_INTERRUPTS BUMP_PORT->IE |= BUMP_BITS

// Definitions for motors
#define DIRR_PORT                       P5      // Right Motor Direction
#define DIRR_PIN                        5
#define DIRR_BIT                        (0x01 << DIRR_PIN)  // 0x20
#define RIGHT_MOTOR_DIRECTION_FORWARD   DIRR_PORT->OUT &= ~DIRR_BIT
#define RIGHT_MOTOR_DIRECTION_REVERSE   DIRR_PORT->OUT |= DIRR_BIT

#define SLPR_PORT                       P3      // Right Motor Sleep
#define SLPR_PIN                        6
#define SLPR_BIT                        (0x01 << SLPR_PIN)  // 0x40
#define RIGHT_MOTOR_SLEEP_ENABLE        SLPR_PORT->OUT &= ~SLPR_BIT
#define RIGHT_MOTOR_SLEEP_DISABLE       SLPR_PORT->OUT |= SLPR_BIT

#define PWMR_PORT                       P2      // Right Motor PWM
#define PWMR_PIN                        6
#define PWMR_BIT                        (0x01 << PWMR_PIN)  // 0x40
#define RIGHT_MOTOR_PWM_ENABLE          PWMR_PORT->OUT |= PWMR_BIT
#define RIGHT_MOTOR_PWM_DISABLE         PWMR_PORT->OUT &= ~PWMR_BIT

#define DIRL_PORT                       P5      // Left Motor Direction
#define DIRL_PIN                        4
#define DIRL_BIT                        (0x01 << DIRL_PIN)  // 0x10
#define LEFT_MOTOR_DIRECTION_FORWARD    DIRL_PORT->OUT &= ~DIRL_BIT
#define LEFT_MOTOR_DIRECTION_REVERSE    DIRL_PORT->OUT |= DIRL_BIT

#define SLPL_PORT                       P3      // Left Motor Sleep
#define SLPL_PIN                        7
#define SLPL_BIT                        (0x01 << SLPL_PIN)  // 0x80
#define LEFT_MOTOR_SLEEP_ENABLE         SLPL_PORT->OUT &= ~SLPL_BIT
#define LEFT_MOTOR_SLEEP_DISABLE        SLPL_PORT->OUT |= SLPL_BIT

#define PWML_PORT                       P2      // Left Motor PWM
#define PWML_PIN                        7
#define PWML_BIT                        (0x01 << PWML_PIN)  // 0x80
#define LEFT_MOTOR_PWM_ENABLE           PWML_PORT->OUT |= PWML_BIT
#define LEFT_MOTOR_PWM_DISABLE          PWML_PORT->OUT &= ~PWML_BIT

// Definitions associated with IR distance sensors.
#define ANALOG_CHANNEL6_PORT P4
#define ANALOG_CHANNEL6_BIT 0x80
#define ANALOG_CHANNEL7_PORT P4
#define ANALOG_CHANNEL7_BIT 0x40
#define ANALOG_CHANNEL12_PORT P4
#define ANALOG_CHANNEL12_BIT 0x02
#define ANALOG_CHANNEL14_PORT P6
#define ANALOG_CHANNEL14_BIT 0x02
#define ANALOG_CHANNEL16_PORT P9
#define ANALOG_CHANNEL16_BIT 0x02
#define ANALOG_CHANNEL17_PORT P9
#define ANALOG_CHANNEL17_BIT 0x01

// Definitions associated with tachometers
#define ENCODER_LEFT_A_PORT    P10
#define ENCODER_LEFT_A_PIN     5
#define ENCODER_LEFT_A_BIT     (0x01 << ENCODER_LEFT_A_PIN)  // 0x20

#define ENCODER_LEFT_B_PORT    P5
#define ENCODER_LEFT_B_PIN     2
#define ENCODER_LEFT_B_BIT     (0x01 << ENCODER_LEFT_B_PIN)  // 0x04

#define ENCODER_RIGHT_A_PORT   P10
#define ENCODER_RIGHT_A_PIN    4
#define ENCODER_RIGHT_A_BIT    (0x01 << ENCODER_RIGHT_A_PIN)  // 0x10

#define ENCODER_RIGHT_B_PORT   P5
#define ENCODER_RIGHT_B_PIN    0
#define ENCODER_RIGHT_B_BIT    (0x01 << ENCODER_RIGHT_B_PIN)  // 0x01

// Definitions associated with IR Sensors
#define DISTANCE_SENSOR_RIGHT_PORT      P9
#define DISTANCE_SENSOR_RIGHT_PIN       0
#define DISTANCE_SENSOR_RIGHT_BIT       (0x01 << DISTANCE_SENSOR_RIGHT_PIN)
#define DISTANCE_SENSOR_RIGHT_CHANNEL   ADC14_MCTLN_INCH_17
#define DISTANCE_SENSOR_RIGHT_MEM       2

#define DISTANCE_SENSOR_CENTER_PORT     P6
#define DISTANCE_SENSOR_CENTER_PIN      1
#define DISTANCE_SENSOR_CENTER_BIT      (0x01 << DISTANCE_SENSOR_CENTER_PIN)
#define DISTANCE_SENSOR_CENTER_CHANNEL  ADC14_MCTLN_INCH_14
#define DISTANCE_SENSOR_CENTER_MEM      3

#define DISTANCE_SENSOR_LEFT_PORT       P9
#define DISTANCE_SENSOR_LEFT_PIN        1
#define DISTANCE_SENSOR_LEFT_BIT        (0x01 << DISTANCE_SENSOR_LEFT_PIN)
#define DISTANCE_SENSOR_LEFT_CHANNEL    ADC14_MCTLN_INCH_16
#define DISTANCE_SENSOR_LEFT_MEM        4

// Note that encoder outputs are open-drain (see https://www.pololu.com/product/3542),
// so external pull-up resistors are needed, which means we shouldn't need to use
// internal pull-up/pull-down resistors.
#define SET_ELA_AS_INPUT                        ENCODER_LEFT_A_PORT->DIR &= ~ENCODER_LEFT_A_BIT
#define SET_ERA_AS_INPUT                        ENCODER_RIGHT_A_PORT->DIR &= ~ENCODER_RIGHT_A_BIT
#define RISING_EDGE_INTERRUPTS_ELA              ENCODER_LEFT_A_PORT->IES &= ~ENCODER_LEFT_A_BIT
#define RISING_EDGE_INTERRUPTS_ERA              ENCODER_RIGHT_A_PORT->IES &= ~ENCODER_RIGHT_A_BIT
#define ENABLE_ELA_INTERRUPTS                   ENCODER_LEFT_A_PORT->IE |= ENCODER_LEFT_A_BIT
#define ENABLE_ERA_INTERRUPTS                   ENCODER_RIGHT_A_PORT->IE |= ENCODER_RIGHT_A_BIT
#define CLEAR_ENCODER_LEFT_A_INTERRUPT_FLAG     ENCODER_LEFT_A_PORT->IFG &= ~ENCODER_LEFT_A_BIT
#define CLEAR_ENCODER_RIGHT_A_INTERRUPT_FLAG    ENCODER_RIGHT_A_PORT->IFG &= ~ENCODER_RIGHT_A_BIT

#define SET_ELB_AS_INPUT                        ENCODER_LEFT_B_PORT->DIR &= ~ENCODER_LEFT_B_BIT
#define SET_ERB_AS_INPUT                        ENCODER_RIGHT_B_PORT->DIR &= ~ENCODER_RIGHT_B_BIT
#define RISING_EDGE_INTERRUPTS_ELB              ENCODER_LEFT_B_PORT->IES &= ~ENCODER_LEFT_B_BIT
#define RISING_EDGE_INTERRUPTS_ERB              ENCODER_RIGHT_B_PORT->IES &= ~ENCODER_RIGHT_B_BIT
#define ENABLE_ELB_INTERRUPTS                   ENCODER_LEFT_B_PORT->IE |= ENCODER_LEFT_B_BIT
#define ENABLE_ERB_INTERRUPTS                   ENCODER_RIGHT_B_PORT->IE |= ENCODER_RIGHT_B_BIT
#define CLEAR_ENCODER_LEFT_B_INTERRUPT_FLAG     ENCODER_LEFT_B_PORT->IFG &= ~ENCODER_LEFT_B_BIT
#define CLEAR_ENCODER_RIGHT_B_INTERRUPT_FLAG    ENCODER_RIGHT_B_PORT->IFG &= ~ENCODER_RIGHT_B_BIT

// Port definitions for robot arm servos
#define SERVO_ARM_HEIGHT_PORT               P2
#define SERVO_ARM_HEIGHT_PORT_MAP_REGISTER  P2MAP->PMAP_REGISTER4
#define SERVO_ARM_HEIGHT_PIN                4
#define SERVO_ARM_HEIGHT_BIT                (0x01 << SERVO_ARM_HEIGHT_PIN)  // 0x10

#define SERVO_ARM_TILT_PORT                 P3
#define SERVO_ARM_TILT_PORT_MAP_REGISTER    P3MAP->PMAP_REGISTER5
#define SERVO_ARM_TILT_PIN                  5
#define SERVO_ARM_TILT_BIT                  (0x01 << SERVO_ARM_TILT_PIN)  // 0x20

#define SERVO_GRIPPER_PORT                  P5
#define SERVO_GRIPPER_PIN                   7
#define SERVO_GRIPPER_BIT                   (0x01 << SERVO_GRIPPER_PIN)  //0x80

#define SERVO_ARM_HEIGHT_FEEDBACK_PORT      P8
#define SERVO_ARM_HEIGHT_FEEDBACK_BIT       0x10
#define SERVO_ARM_TILT_FEEDBACK_PORT        P8
#define SERVO_ARM_TILT_FEEDBACK_BIT         0x08
#define SERVO_GRIPPER_FEEDBACK_PORT         P8
#define SERVO_GRIPPER_FEEDBACK_BIT          0x04

// Port pin definitions for LCD module
// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
#define LCD_PORT                    P9
#define LCD_ENABLE                  0x10    // UCA3STE       (SCE, pin 3) connected to P9.4
#define LCD_RESET                   0x08    // Reset         (RST, pin 4) connected to P9.3
#define LCD_DC                      0x40    // Data/Command  (D/C, pin 5) connected to P9.6
#define LCD_MOSI                    0x80    // UCA3SIMO      (DN,  pin 6) connected to P9.7
#define LCD_SCLK                    0x20    // UCA3CLK       (SCLK, pin 7) connected to P9.5
#define LCD_SET_ENABLE_AS_OUTPUT    LCD_PORT->DIR |= LCD_ENABLE
#define LCD_SET_RESET_AS_OUTPUT     LCD_PORT->DIR |= LCD_RESET
#define LCD_SET_DC_AS_OUTPUT        LCD_PORT->DIR |= LCD_DC
#define LCD_SET_MOSI_AS_OUTPUT      LCD_PORT->DIR |= LCD_MOSI
#define LCD_SET_SCLK_AS_OUTPUT      LCD_PORT->DIR |= LCD_SCLK

#define LCD_ENABLE_EQUAL_1          LCD_PORT->OUT |= LCD_ENABLE
#define LCD_ENABLE_EQUAL_0          LCD_PORT->OUT &= ~LCD_ENABLE
#define LCD_RESET_EQUALS_1          LCD_PORT->OUT |= LCD_RESET
#define LCD_RESET_EQUALS_0          LCD_PORT->OUT &= ~LCD_RESET
#define SET_DC_FOR_COMMAND          LCD_PORT->OUT &= ~LCD_DC
#define SET_DC_FOR_DATA             LCD_PORT->OUT |= LCD_DC
#define LCD_MOSI_EQUAL_1            LCD_PORT->OUT |= LCD_MOSI
#define LCD_MOSI_EQUAL_0            LCD_PORT->OUT &= ~LCD_MOSI
#define LCD_TOGGLE_SCLK             LCD_PORT->OUT ^= LCD_SCLK
#define LCD_SCLK_EQUALS_1           LCD_PORT->OUT |= LCD_SCLK
#define LCD_SCLK_EQUALS_0           LCD_PORT->OUT &= ~LCD_SCLK

// Port pin definitions for CC2650 Module Boosterpack

// DEFAULT equals TRUE chooses option 4; otherwise, chooses options 1,2,3
#define TRUE 1
#define FALSE 0
#define DEFAULT TRUE

#if (DEFAULT == TRUE)
// Option 4
#define CC2650_MRDY_PORT       P6
#define CC2650_MRDY_PIN        0
#define CC2650_RESET_PORT      P6
#define CC2650_RESET_PIN       7
#define CC2650_SRDY_PORT       P2
#define CC2650_SRDY_PIN        5
#else
// Options 1,2,3
#define CC2650_MRDY_PORT       P1
#define CC2650_MRDY_PIN        7
#define CC2650_RESET_PORT      P6
#define CC2650_RESET_PIN       7
#define CC2650_SRDY_PORT       P5
#define CC2650_SRDY_PIN        2
#endif
#define CC2650_MRDY_BIT        (0x01 << CC2650_MRDY_PIN)
#define CC2650_RESET_BIT       (0x01 << CC2650_RESET_PIN)
#define CC2650_ENABLE_MRDY     CC2650_MRDY_PORT->OUT &= ~CC2650_MRDY_BIT     // Active-low signal
#define CC2650_DISABLE_MRDY    CC2650_MRDY_PORT->OUT |= CC2650_MRDY_BIT
#define CC2650_ENABLE_RESET    CC2650_RESET_PORT->OUT &= ~CC2650_RESET_BIT        // Active-low signal
#define CC2650_DISABLE_RESET   CC2650_RESET_PORT->OUT |= CC2650_RESET_BIT
#define CC2650_SRDY_BIT        (0x01 << CC2650_SRDY_PIN)
#define CC2650_READ_SRDY       ((CC2650_SRDY_PORT->IN & CC2650_SRDY_BIT) >> CC2650_SRDY_PIN)


// Bit-banding for P5.3 and P7.0-P7.7
#define REFLECTANCE_IR  (*((volatile uint8_t *) (0x4209884c)))
#define REFLECTANCE_S1_IN  (*((volatile uint8_t *) (0x42098c00)))
#define REFLECTANCE_S2_IN  (*((volatile uint8_t *) (0x42098c04)))
#define REFLECTANCE_S3_IN  (*((volatile uint8_t *) (0x42098c08)))
#define REFLECTANCE_S4_IN  (*((volatile uint8_t *) (0x42098c0c)))
#define REFLECTANCE_S5_IN  (*((volatile uint8_t *) (0x42098c10)))
#define REFLECTANCE_S6_IN  (*((volatile uint8_t *) (0x42098c14)))
#define REFLECTANCE_S7_IN  (*((volatile uint8_t *) (0x42098c18)))
#define REFLECTANCE_S8_IN  (*((volatile uint8_t *) (0x42098c1c)))

#define REFLECTANCE_S1_OUT  (*((volatile uint8_t *) (0x42098c40)))
#define REFLECTANCE_S2_OUT  (*((volatile uint8_t *) (0x42098c44)))
#define REFLECTANCE_S3_OUT  (*((volatile uint8_t *) (0x42098c48)))
#define REFLECTANCE_S4_OUT  (*((volatile uint8_t *) (0x42098c4c)))
#define REFLECTANCE_S5_OUT  (*((volatile uint8_t *) (0x42098c50)))
#define REFLECTANCE_S6_OUT  (*((volatile uint8_t *) (0x42098c54)))
#define REFLECTANCE_S7_OUT  (*((volatile uint8_t *) (0x42098c58)))
#define REFLECTANCE_S8_OUT  (*((volatile uint8_t *) (0x42098c5c)))

// Function Prototypes
void PortRegisterMapping(void);

#endif /* PORTPINS_H_ */
