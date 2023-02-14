// Sammy Nayhouse (san6yhk)

#include <stdio.h>
#include "msp.h"
#include "Defines.h"
#include "PortPins.h"
#include "Clock.h"
#include "CortexM.h"
#include "UART0.h"
#include "FSM.h"
#include "Tasks.h"
#include "ADCInit.h"
#include "Motor.h"

// Function prototypes
void Task1(FSMType* DistanceSensor);
void Task2(FSMType* BumpSwitch);
void Task3(FSMType* RotaryPhaseDecoder);
void InitializeGlobalVariables(void);
void InitializePortPins(void);
void SystemTimerInitialization(uint16_t period);
void InitializeADC(void);
void GlobalTimerInitialization(uint16_t period);

// Definitions
#define DISTANCE_LIMIT          300
#define OUTPUT_DISPLAY_LIMIT    10000

// Variables
uint64_t GlobalTimer = 0;
uint16_t RightSpeed;
uint16_t LeftSpeed;
uint8_t BumpSwitchesAreActive = FALSE;

// Global variables
DistanceSensorType DistanceSensors[NUMBER_OF_DISTANCE_SENSORS];     //0 <-> Right, 1 <-> Center, 2 <->Left
BumpSwitchType BumpSwitches[NUMBER_OF_BUMP_SWITCHES];
RotaryPhaseDecoderType EncoderFSMs[NUMBER_OF_ROTARY_DECODERS];      // 0 <-> Left, 1 <-> Right

TaskType Tasks[NUMBER_OF_TASKS]=
{
 // Task, Task Counter, Task Execution Period, FSM Object
 {&Task1, 0, Ts_DISTANCE_SENSORS, (FSMType *) &DistanceSensors[0]},
 {&Task2, 0, Ts_BUMP_SWITCHES,    (FSMType*)  &BumpSwitches[0]},
 {&Task3, 0, Ts_MOTOR_DECODERS,   (FSMType*)  &EncoderFSMs[0]}
};

// Main function
void main(void)
{
    // Variables
    uint32_t i;
    uint32_t left;
    uint32_t middle;
    uint32_t right;
    uint32_t Distance = 0;
    uint32_t SensorADCAverage = 0;

    /*
    uint32_t OutputCounter = 0;
    uint32_t SensorID = 0;
    uint32_t SensorInput;
    char OutputBuffer [OUTPUT_BUFFER_LENGTH];
    */

    DisableInterrupts();
    Clock_Init48MHz();
    SystemTimerInitialization(SCHEDULE_TIMER_PERIOD);
    GlobalTimerInitialization(GLOBAL_TIMER_PERIOD);
    InitializePortPins();
    InitializeADC();
    UART0_Init();
    InitializeGlobalVariables();
    Motor_Init();
    EnableInterrupts();

    /*
    // Clear the output buffer used to print to the terminal window.
    for(i = 0; i < OUTPUT_BUFFER_LENGTH; i++) OutputBuffer[i] = 0;
    */

    // Determine if either any bump switch has been activated, or the
    // robot is within the distance limit of an object. If either is true,
    // robot should stop until both conditions are not true.
    while(TRUE)
    {
        if (BumpSwitchesAreActive)
        {
            Motor_Stop();
        }
        else
        {
            for (i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++)
            {
                SensorADCAverage = DistanceSensors[i].SensorAverage;
                Distance = ConvertDistanceSensorReading(SensorADCAverage);
                DistanceSensors[i].Distance = Distance;
            }
            right = DistanceSensors[0].Distance;
            middle = DistanceSensors[1].Distance;
            left = DistanceSensors[2].Distance;
            if (middle < DISTANCE_LIMIT)
            {
                if (right < DISTANCE_LIMIT || left < DISTANCE_LIMIT)
                {
                    if (right > left)
                    {
                        Motor_Right(7500, 7500);
                        Clock_Delay1us(75);
                    }
                    else
                    {
                        Motor_Left(7500, 7500);
                        Clock_Delay1us(75);
                    }
                }
            }
            else if (abs(left - right) < DISTANCE_LIMIT)
            {
                Motor_Forward(7500, 7500);
            }
            else if (right < left)
            {
                Motor_Forward(3000, 7500);
            }
            else if (left < right)
            {
                Motor_Forward(7500, 3000);
            }
        }

    }
}

void Task1 (FSMType *DistanceSensor)
{
    DistanceSensorType *LocalSensor = (DistanceSensorType *) DistanceSensor;
    uint32_t CurrentSensorInput = 0;
    uint32_t i;

    // This flag corresponds to the largest ADC memory index used.
    uint32_t ADCFlag = (uint32_t) (ADC14_IFGR0_IFG0 << ADC_FLAG_SHIFT);

    //wait for busy to be zero
    while(ADC14->CTL0 & ADC14_CTL0_BUSY){};

    //wait for conversions to complete
    while((ADC14->IFGR0 & ADCFlag) == 0){};

    for(i = 0; i < NUMBER_OF_DISTANCE_SENSORS; i++) {

        // Read current sensor input value.
        CurrentSensorInput= ADC14->MEM[((LocalSensor+i)->ADCMemoryIndex)];

        // Update cumulative sum.
        (LocalSensor+i)->CumulativeSum = ((LocalSensor+i)->CumulativeSum)+CurrentSensorInput -  ((LocalSensor+i)->SensorInputBuffer[(LocalSensor+i)->BufferIndex]);

        //Calculate the average.
        (LocalSensor+i)->SensorAverage = ((LocalSensor+i)->CumulativeSum) >> SENSOR_INPUT_AVERAGE_SHIFT;

        // Update input buffer with current sensor input.
        (LocalSensor+i)->SensorInputBuffer[((LocalSensor+i)->BufferIndex)] = CurrentSensorInput;

        // Update buffer index (modulo operation without using modulo operator).
        (LocalSensor+i)->BufferIndex++;
        if((LocalSensor+i)->BufferIndex >= SENSOR_INPUT_BUFFER_LENGTH) (LocalSensor+i)->BufferIndex = 0;
    }
    // start next conversion
    ADC14->CTL0 |= ADC14_CTL0_SC;
}

void Task2(FSMType* BumpSwitch)
{
    // Cast FSMType pointer to correct type for task.
    BumpSwitchType* LocalSwitch = (BumpSwitchType*) BumpSwitch;

    // Determine if any bump switches are active
    uint32_t i;
    for (i = 0; i < NUMBER_OF_BUMP_SWITCHES; i++) {
        if (ReadSwitchStatus(&(LocalSwitch + i)->Switch) == Active) {
            BumpSwitchesAreActive = TRUE;
            return;
        }
    }
    BumpSwitchesAreActive = FALSE;
}

void Task3(FSMType* RotaryPhaseDecoder)
{
    // Cast FSMType pointer to correct type for task.
    RotaryPhaseDecoderType* LocalDecoder = (RotaryPhaseDecoderType*) RotaryPhaseDecoder;

    // Variables
    uint32_t i;
    uint64_t t;
    uint16_t CurrentSpeed;
    uint64_t EndingTime;

    // Read current inputs.
    // Next, produce the output based on the current state.
    // Finally, based on the inputs and the current state, determine the next state.
    for (i = 0; i < NUMBER_OF_ROTARY_DECODERS; i++)
    {
        // Maximum
        if ((LocalDecoder+i)->EncoderStateCount == 1439)
        {
            EndingTime = GlobalTimer;
            t = EndingTime - (LocalDecoder+i)->BeginTime;
            CurrentSpeed = (uint16_t) (60000 / t);
            if (CurrentSpeed < (LocalDecoder+i)->SetPoint)
                (LocalDecoder+i)->DutyCycle += 0;
            else if (CurrentSpeed > (LocalDecoder+i)->SetPoint)
                (LocalDecoder+i)->DutyCycle -= 0;
            if (i == 0)
                LeftSpeed = CurrentSpeed;
            else
                RightSpeed = CurrentSpeed;
            (LocalDecoder+i)->EncoderStateCount = 0;
            (LocalDecoder+i)->BeginTime = EndingTime;
        }

        // Minimum
        else if ((LocalDecoder+i)->EncoderStateCount == -1439)
        {
            EndingTime = GlobalTimer;
            t = EndingTime - (LocalDecoder+i)->BeginTime;
            CurrentSpeed = (uint16_t) (60000 / t);
            if (CurrentSpeed < (LocalDecoder+i)->SetPoint)
                (LocalDecoder+i)->DutyCycle += 0;
            else if (CurrentSpeed > (LocalDecoder+i)->SetPoint)
                (LocalDecoder+i)->DutyCycle -= 0;
            if (i == 0)
                LeftSpeed = CurrentSpeed;
            else
                RightSpeed = CurrentSpeed;
            (LocalDecoder+i)->EncoderStateCount = 0;
            (LocalDecoder+i)->BeginTime = EndingTime;
        }

        (LocalDecoder + i)->SwitchA = ReadSwitchStatus(&((LocalDecoder + i)->EncoderA));
        (LocalDecoder + i)->SwitchB = ReadSwitchStatus(&((LocalDecoder + i)->EncoderB));
        (LocalDecoder + i)->CurrentState = NextStateFunctionRotaryPhaseDecoder((LocalDecoder + i));
    }
}

void InitializeGlobalVariables (void)
{
    // Initialize the IR sensor FSMs.
    InitializeDistanceSensor(&DistanceSensors[0], DISTANCE_SENSOR_RIGHT_CHANNEL,  DISTANCE_SENSOR_RIGHT_MEM,0);
    InitializeDistanceSensor(&DistanceSensors[1], DISTANCE_SENSOR_CENTER_CHANNEL, DISTANCE_SENSOR_CENTER_MEM,1);
    InitializeDistanceSensor(&DistanceSensors[2], DISTANCE_SENSOR_LEFT_CHANNEL,   DISTANCE_SENSOR_LEFT_MEM,2);

    // Initialize the bump switches.
    SET_BUMP_SWITCHES_AS_INPUTS;
    ENABLE_PULL_RESISTORS;
    PULL_UP_RESISTORS;
    InitializeBump(&BumpSwitches[0], &BumpSwitches[0].Switch, (uint8_t*) &(BUMP_PORT->IN), (uint8_t) BUMP0_BIT, Active, Inactive, 0);
    InitializeBump(&BumpSwitches[1], &BumpSwitches[1].Switch, (uint8_t*) &(BUMP_PORT->IN), (uint8_t) BUMP1_BIT, Active, Inactive, 1);
    InitializeBump(&BumpSwitches[2], &BumpSwitches[2].Switch, (uint8_t*) &(BUMP_PORT->IN), (uint8_t) BUMP2_BIT, Active, Inactive, 2);
    InitializeBump(&BumpSwitches[3], &BumpSwitches[3].Switch, (uint8_t*) &(BUMP_PORT->IN), (uint8_t) BUMP3_BIT, Active, Inactive, 3);
    InitializeBump(&BumpSwitches[4], &BumpSwitches[4].Switch, (uint8_t*) &(BUMP_PORT->IN), (uint8_t) BUMP4_BIT, Active, Inactive, 4);
    InitializeBump(&BumpSwitches[5], &BumpSwitches[5].Switch, (uint8_t*) &(BUMP_PORT->IN), (uint8_t) BUMP5_BIT, Active, Inactive, 5);

    // Initialize switches.
    SET_ELA_AS_INPUT;
    SET_ERA_AS_INPUT;
    SET_ELB_AS_INPUT;
    SET_ERB_AS_INPUT;
    InitializeSwitch(&(EncoderFSMs[0].EncoderA), (uint8_t*) &(ENCODER_LEFT_A_PORT->IN),  (uint8_t) ENCODER_LEFT_A_BIT,  Inactive, Active);
    InitializeSwitch(&(EncoderFSMs[0].EncoderB), (uint8_t*) &(ENCODER_LEFT_B_PORT->IN),  (uint8_t) ENCODER_LEFT_B_BIT,  Inactive, Active);
    InitializeSwitch(&(EncoderFSMs[1].EncoderA), (uint8_t*) &(ENCODER_RIGHT_A_PORT->IN), (uint8_t) ENCODER_RIGHT_A_BIT, Inactive, Active);
    InitializeSwitch(&(EncoderFSMs[1].EncoderB), (uint8_t*) &(ENCODER_RIGHT_B_PORT->IN), (uint8_t) ENCODER_RIGHT_B_BIT, Inactive, Active);

    // Initialize Finite State Machine (FSM) state variables.
    InitializeRotaryPhaseDecoder(&(EncoderFSMs[0]), LEFT_SENSOR_ID);
    InitializeRotaryPhaseDecoder(&(EncoderFSMs[1]), RIGHT_SENSOR_ID);
    EncoderFSMs[0].SetPoint = 75; // units = RPM
    EncoderFSMs[1].SetPoint = 75; // units = RPM
}

void SystemTimerInitialization(uint16_t period)
{
    SCHEDULE_TIMER->CTL = (TIMER_A_CTL_MC__STOP | TIMER_A_CTL_CLR);

    SCHEDULE_TIMER->CCTL[SCHEDULE_TIMER_CCR] = TIMER_A_CCTLN_CCIE;
    SCHEDULE_TIMER->CCR[SCHEDULE_TIMER_CCR] = (period-1);
    SCHEDULE_TIMER->EX0 = 0x005;                // configure for input clock divider /6

    NVIC_INTERRUPT_ENABLE(ISER_REGISTER,SCHEDULE_TIMER_INTERRUPT);
    PRIORITY_REGISTER(SCHEDULE_TIMER_INTERRUPT,INTERRUPT_PRIORITY(SCHEDULE_TIMER_INTERRUPT_PRIORITY));

    SCHEDULE_TIMER->CTL = (TIMER_A_CTL_SSEL__SMCLK | TIMER_A_CTL_ID__2 | TIMER_A_CTL_MC__UP);
}

void GlobalTimerInitialization(uint16_t period)
{
    GLOBAL_TIMER->CTL = (TIMER_A_CTL_MC__STOP | TIMER_A_CTL_CLR);

    GLOBAL_TIMER->CCTL[GLOBAL_TIMER_CCR] = TIMER_A_CCTLN_CCIE;
    GLOBAL_TIMER->CCR[GLOBAL_TIMER_CCR] = (period-1);
    GLOBAL_TIMER->EX0 = 0x0000;

    NVIC_INTERRUPT_ENABLE(ISER_REGISTER, GLOBAL_TIMER_INTERRUPT);
    PRIORITY_REGISTER(GLOBAL_TIMER_INTERRUPT, INTERRUPT_PRIORITY(GLOBAL_TIMER_INTERRUPT_PRIORITY));

    GLOBAL_TIMER->CTL = (TIMER_A_CTL_SSEL__SMCLK |TIMER_A_CTL_ID__1 | TIMER_A_CTL_MC__UP);
}

void SCHEDULE_TIMER_IRQ(void)
{
    SCHEDULE_TIMER->CCTL[SCHEDULE_TIMER_CCR] &= ~TIMER_A_CCTLN_CCIFG;
    TaskSchedulerISR();
}

void GLOBAL_TIMER_IRQ(void)
{
    GLOBAL_TIMER->CCTL[GLOBAL_TIMER_CCR] &= ~TIMER_A_CCTLN_CCIFG;
    GlobalTimer++;
}

void InitializePortPins(void)
{
    DISTANCE_SENSOR_RIGHT_PORT->SEL0 |= DISTANCE_SENSOR_RIGHT_BIT;
    DISTANCE_SENSOR_RIGHT_PORT->SEL1 |= DISTANCE_SENSOR_RIGHT_BIT;
    DISTANCE_SENSOR_CENTER_PORT->SEL0 |= DISTANCE_SENSOR_CENTER_BIT;
    DISTANCE_SENSOR_CENTER_PORT->SEL1 |= DISTANCE_SENSOR_CENTER_BIT;
    DISTANCE_SENSOR_LEFT_PORT->SEL0 |= DISTANCE_SENSOR_LEFT_BIT;
    DISTANCE_SENSOR_LEFT_PORT->SEL1 |= DISTANCE_SENSOR_LEFT_BIT;
}

