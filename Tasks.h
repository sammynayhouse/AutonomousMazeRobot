// Sammy Nayhouse (san6yhk)

#ifndef TASKS_H_
#define TASKS_H_

#include "msp.h"
#include "Defines.h"
#include "FSM.h"

typedef struct {
    void (*Task)(FSMType *);        // Pointer to task function for FSM
    uint32_t TaskCycleCounter;      // Task is executed when (TaskCycleCounter == TaskExecutionPeriod)
    uint32_t TaskExecutionPeriod;   // Must equal an integer multiple of the scheduler execution period
    FSMType *FSM;                   // FSM with state information
}TaskType;


//Function prototypes
void TaskSchedulerISR(void);

#endif /* TASKS_H_ */
