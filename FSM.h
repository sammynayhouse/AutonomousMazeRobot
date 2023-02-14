// Sammy Nayhouse (san6yhk)

#ifndef FSM_H_
#define FSM_H_

#include "msp.h"
#include "Defines.h"

/*
 * The FSMType structure is the base for all derived FSM types, and would be used
 * (and possibly augmented) for information that is common to all FSM types.
 *
 * Regardless, for all derived FSM types, the first field must be of type FSMType
 * so that derived types can be cast as a pointer to FSMType so that the objects can
 * be passed to the tasks scheduled by the task scheduler (see Tasks.h).
 *
 */
typedef struct {
    uint32_t Value;
} FSMType;

// Distance sensor type derived from FSMType. Note that this object has no state
// variables, since it is an analog input.
typedef struct {
    FSMType FSM;
    uint32_t AnalogChannel;     // Analog channel associated with sensor
    uint32_t ADCMemoryIndex;    // ADC conversion memory index for (analog) sensor input
    uint32_t Distance;          // Distance based on average sensor input
    uint32_t CumulativeSum;     // Cumulative sum of sensor input buffer
    uint32_t SensorAverage;     // Average of sensor input buffer
    uint32_t BufferIndex;       // Sensor input buffer index that points to most recent input value
    uint32_t SensorInputBuffer[SENSOR_INPUT_BUFFER_LENGTH];
} DistanceSensorType;

// Function Prototypes
void InitializeFSM(FSMType *FSM, uint32_t Value);

void InitializeDistanceSensor(DistanceSensorType *DistanceSensor,uint32_t AnalogChannel,uint32_t ADCMemoryIndex,
                              uint32_t SensorID);
uint32_t ConvertDistanceSensorReading(uint32_t SensorInput);

// Switch type definition
typedef enum {Inactive, Active} SwitchStatus;

typedef struct {
    uint8_t* SwitchPort;            // Input port associated with switch
    uint8_t SwitchBit;              // Bit mask for port pin associated with switch
    SwitchStatus PortPinEquals0;    // Switch state associated with logic 0
    SwitchStatus PortPinEquals1;    // Switch state associated with logic 1
} SwitchDefine;

typedef struct {
    FSMType FSM;
    SwitchDefine Switch;
} BumpSwitchType;

void InitializeSwitch(SwitchDefine* Switch, uint8_t* SwitchPort, uint8_t SwitchBit, SwitchStatus PortPinEquals0, SwitchStatus PortPinquals1);

void InitializeBump(BumpSwitchType* Bump, SwitchDefine* Switch, uint8_t* SwitchPort, uint8_t SwitchBit, SwitchStatus PortPinEquals0, SwitchStatus PortPinEquals1, uint32_t BumpID);

// This function returns the instantaneous value of the selected switch
SwitchStatus ReadSwitchStatus(SwitchDefine* Switch);

// Rotary Phase Decoder FSM
typedef enum {AInactiveBInactive, AInactiveBActive, AActiveBInactive, AActiveBActive, ResetState} RotaryPhaseDecoderState;

typedef struct {
    FSMType FSM;
    RotaryPhaseDecoderState CurrentState;   // Current state of the FSM
    uint16_t DutyCycle;                     // Output: PWM duty cycle
    int16_t EncoderStateCount;              // Indicates valid FSM sequences.
    uint64_t BeginTime;                     // Begin time for FSM cycle
    uint16_t Period;                        // FSM cycle period
    uint16_t SetPoint;                      // Desired speed (units = RPM)
    SwitchStatus Reset;                     // Used to reset the FSM; currently unused.
    SwitchStatus SwitchA;                   // Encoder inputs A and B
    SwitchStatus SwitchB;
    SwitchDefine EncoderA;
    SwitchDefine EncoderB;
} RotaryPhaseDecoderType;

void InitializeRotaryPhaseDecoder(RotaryPhaseDecoderType* Decoder, uint32_t MotorID);
RotaryPhaseDecoderState NextStateFunctionRotaryPhaseDecoder(RotaryPhaseDecoderType* Decoder);
void OutputFunctionRotaryPhaseDecoder(RotaryPhaseDecoderType* Decoder);
uint16_t AdaptiveControl(uint16_t Difference);

#endif /* FSM_H_ */

