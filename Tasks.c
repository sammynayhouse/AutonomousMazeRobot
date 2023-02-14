// Sammy Nayhouse (san6yhk)

#include "Defines.h"
#include "Tasks.h"

extern TaskType Tasks[NUMBER_OF_TASKS];

void TaskSchedulerISR(void)
{
    uint32_t i;
    for(i = 0; i < NUMBER_OF_TASKS; i++){
        Tasks[i].TaskCycleCounter++;
        if(Tasks[i].TaskCycleCounter >= Tasks[i].TaskExecutionPeriod){
            Tasks[i].TaskCycleCounter = 0;
            (*Tasks[i].Task)((FSMType *) Tasks[i].FSM);
        }
    }
}
