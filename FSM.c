// Sammy Nayhouse (san6yhk)

#include "FSM.h"
#include "Motor.h"
#include "PortPins.h"

// Definitions
#define DEFAULT_DUTY_CYCLE  7000

// Variables
extern uint64_t GlobalTimer;

void InitializeFSM(FSMType *FSM, uint32_t Value)
{
    FSM->Value = Value;
}

void InitializeDistanceSensor(DistanceSensorType *DistanceSensor,uint32_t AnalogChannel,uint32_t ADCMemoryIndex,
                              uint32_t SensorID)
{
    InitializeFSM((FSMType *) DistanceSensor, SensorID);

    DistanceSensor -> AnalogChannel = AnalogChannel;
    DistanceSensor -> ADCMemoryIndex = ADCMemoryIndex;
    DistanceSensor -> Distance = 0;
    DistanceSensor -> BufferIndex = 0;
    DistanceSensor -> CumulativeSum = 0;
    DistanceSensor -> SensorAverage = 0;
    uint32_t i = 0;
    for (i = 0; i < SENSOR_INPUT_BUFFER_LENGTH; i++){
        DistanceSensor -> SensorInputBuffer[i] = 0;
    }
}

uint32_t ConvertDistanceSensorReading(uint32_t SensorInput)
{
    uint64_t val1 = 23; // 22.5136535449329
    uint64_t val2 = 552449; // 552448.557808782
    uint64_t val3 = 75061711; // 75061710.6966632

    uint64_t ReturnValue = ((1000000000000)/((SensorInput)*(SensorInput)*val1 + (SensorInput)*val2 + val3));

    if(ReturnValue > 800) ReturnValue = 800;

    return ReturnValue;
}

void InitializeSwitch(SwitchDefine* Switch, uint8_t* SwitchPort, uint8_t SwitchBit, SwitchStatus PortPinEquals0, SwitchStatus PortPinEquals1)
{
    Switch->SwitchPort = SwitchPort;
    Switch->SwitchBit = SwitchBit;
    Switch->PortPinEquals0 = PortPinEquals0;
    Switch->PortPinEquals1 = PortPinEquals1;
}

void InitializeBump(BumpSwitchType* Bump, SwitchDefine* Switch, uint8_t* SwitchPort, uint8_t SwitchBit, SwitchStatus PortPinEquals0, SwitchStatus PortPinEquals1, uint32_t BumpID)
{
    InitializeFSM((FSMType *) Bump, BumpID);
    InitializeSwitch(Switch, SwitchPort, SwitchBit, PortPinEquals0, PortPinEquals1);
}

SwitchStatus ReadSwitchStatus(SwitchDefine* Switch)
{
    uint8_t i = *(Switch->SwitchPort) & Switch->SwitchBit;
    if (i != 0)
        return Switch->PortPinEquals1;
    else
        return Switch->PortPinEquals0;
}

void InitializeRotaryPhaseDecoder(RotaryPhaseDecoderType* Decoder, uint32_t MotorID)
{
    InitializeFSM((FSMType*) Decoder, MotorID);
    Decoder->CurrentState = ResetState;
    Decoder->Reset = Inactive;
    Decoder->DutyCycle = DEFAULT_DUTY_CYCLE;
    Decoder->BeginTime = GlobalTimer;
    Decoder->EncoderStateCount = 0;
}

RotaryPhaseDecoderState NextStateFunctionRotaryPhaseDecoder(RotaryPhaseDecoderType* Decoder)
{
    RotaryPhaseDecoderState NextState;

    // State logic
    switch (Decoder->CurrentState)
    {
    case AInactiveBInactive:
        if (Decoder->Reset == Active)
        {
            NextState = ResetState;
            Decoder->EncoderStateCount = 0;
        }
        else if (Decoder->SwitchA == Active)
        {
            NextState = AActiveBInactive;
            Decoder->EncoderStateCount--;
        }
        else if (Decoder->SwitchB == Active)
        {
            NextState = AInactiveBActive;
            Decoder->EncoderStateCount++;
        }
        break;
    case AInactiveBActive:
        if (Decoder->Reset == Active)
        {
            NextState = ResetState;
            Decoder->EncoderStateCount = 0;
        }
        else if (Decoder->SwitchA == Active)
        {
            NextState = AActiveBActive;
            Decoder->EncoderStateCount++;
        }
        else if (Decoder->SwitchB == Inactive)
        {
            NextState = AInactiveBInactive;
            Decoder->EncoderStateCount--;
        }
        break;
    case AActiveBInactive:
        if (Decoder->Reset == Active)
        {
            NextState = ResetState;
            Decoder->EncoderStateCount = 0;
        }
        else if (Decoder->SwitchA == Inactive)
        {
            NextState = AInactiveBInactive;
            Decoder->EncoderStateCount++;
        }
        else if (Decoder->SwitchB == Active)
        {
            NextState = AActiveBActive;
            Decoder->EncoderStateCount--;
        }
        break;
    case AActiveBActive:
        if (Decoder->Reset == Active)
        {
            NextState = ResetState;
            Decoder->EncoderStateCount = 0;
        }
        else if (Decoder->SwitchA == Inactive)
        {
            NextState = AInactiveBActive;
            Decoder->EncoderStateCount++;
        }
        else if (Decoder->SwitchB == Inactive)
        {
            NextState = AActiveBInactive;
            Decoder->EncoderStateCount--;
        }
        break;
    case ResetState:
        if (Decoder->SwitchA == Inactive && Decoder->SwitchB == Inactive)
        {
            NextState = AInactiveBInactive;
        }
        else if (Decoder->SwitchA == Inactive && Decoder->SwitchB == Active)
        {
            NextState = AInactiveBActive;
        }
        else if (Decoder->SwitchA == Active && Decoder->SwitchB == Inactive)
        {
            NextState = AActiveBInactive;
        }
        else if (Decoder->SwitchA == Active && Decoder->SwitchB == Active)
        {
            NextState = AActiveBActive;
        }
        break;
    }
    return NextState;
}

void OutputFunctionRotaryPhaseDecoder(RotaryPhaseDecoderType* Decoder)
{
    // First update the FSM state counter.
    switch (Decoder->CurrentState)
    {
    case AInactiveBInactive:

        break;
    case AInactiveBActive:

        break;
    case AActiveBInactive:

        break;
    case AActiveBActive:

        break;
    case ResetState:

        break;
    }
}

uint16_t AdaptiveControl(uint16_t Difference)
{
    uint16_t ReturnValue = 0;

    // Adaptive control algorithm to reduce convergence time
    if (Difference > 100)
    {
        ReturnValue = 1000;
    }
    else if (Difference > 50)
    {
        ReturnValue = 500;
    }
    else if (Difference > 25)
    {
        ReturnValue = 100;
    }
    else
    {
        ReturnValue = 50;
    }
    return ReturnValue;

    // return 0;
}

