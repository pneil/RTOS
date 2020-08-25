// RTOS
// Name: Neelkanth Patel


//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDS on these pins:
// Blue:   PF2 (on-board)
// Red:    PE1
// Orange: PE2
// Yellow: PE3
// Green:  PE4
// PBs on these pins
// PB0:    PA2
// PB1:    PA3
// PB2:    PA4
// PB3:    PA5
// PB4:    PA6
// PB5:    PA7

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include <ctype.h>
#include <stdio.h>
#include <math.h>

#define MAX_CHARS 80
#define RESET 0x05FA0005

int     count = 0;
char    uartStr[81];
int     offset[20];
int     fieldCount=0;

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 4*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) // off-board orange LED

// BitBanding Port A
#define pb0     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4)))   // off-board PBs
#define pb1     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4)))   // off-board PBs
#define pb2     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4)))   // off-board PBs
#define pb3     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))   // off-board PBs
#define pb4     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))   // off-board PBs
#define pb5     (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4)))   // off-board PBs


//============================================================================

//FUNCTION TO WAIT MILI SECOND

void    waitMilisecond(uint32_t us);

//============================================================================

//STRING FUNCTION DECLARATION:

unsigned int a2h(char *x);
int     a2i(char* str);
void    revSt(char str[], int len);
char    *i2a(uint16_t num, char* str);
int     str_Cmp(char *str1,char *str2);
int     str_Len(char *str);
void    strCopy(char *str1,const char *str2);
void    hh(uint32_t x);
char*   upLow(char *str1,char *str2);

//FUNCTION FOR ISCOMMAND:

bool isCommand(char *commandName,int min_arg);

//UART FUNCTION:

int     getValue(int argNumber);
char*   getString(int argNumber);
void    putcUart0(char c);
void    putsUart0(char *str);
char    getcUart0();

//=============================================================================

//MEMORY PROTECTION UNIT:

void    setUpMPU();
void    usP();

//Extern .s file register
extern uint32_t getPSP();

//=============================================================================

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------
//Hardware Function

void OffBoardLED();
void OffBoardButtons();

//RTOS Global

uint8_t CurrentTask = 0;        // Index for task no.

//Defined SVC call number for SVC switch case:

#define SVC_yield       101
#define SVC_sleep       102
#define SVC_wait        103
#define SVC_post        104
#define SVC_destroy     105
#define SVC_restart     106
#define SVC_ipcs        107
#define SVC_ps          108
#define SVC_piON        109
#define SVC_piOFF       110
#define SVC_premON      111
#define SVC_premOFF     112
#define SVC_schedON     113
#define SVC_schedOFF    114
#define SVC_pidof       115
#define SVC_kill        116
#define SVC_reset       117
#define SVC_res         118

// EXTERN REGISTER:

extern uint32_t getPSP();
extern uint32_t getMSP();
extern uint32_t getR0();
extern uint32_t getR1();
extern uint32_t getR2();
extern uint32_t getR3();
extern uint32_t getR12();
extern uint32_t getPC();
extern uint32_t getxPSR();
extern uint32_t getLR();
extern uint32_t getPSP();


//uint8_t svcNumber;

uint32_t *pc_task;

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE];  // store task index here
    char     semaName[15];                  // store semaphore name
    uint32_t lastUser;

} semaphores[MAX_SEMAPHORES];

uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource, *semaPT;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 10            // maximum number of valid tasks
uint8_t taskCurrent = 0;        // index of last dispatched task
uint8_t taskCount = 0;          // total number of valid tasks


uint64_t totalTime = 0;                // total time

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly
struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // location of original stack pointer
    void *sp;                      // location of stack pointer for thread
    int8_t priority;               // 0=highest to 15=lowest
    int8_t currentPriority;        // used for priority inheritance
    uint32_t ticks;                // ticks until sleep complete
    char name[16];                 // name of task used in ps command
    void *semaphore;               // pointer to the semaphore that is blocking the thread
    uint8_t skip;                  // Counting value for priority scheduler
    int8_t *lastUser;              // last user of the semaphore
    uint32_t startTime;            // task start time;
    uint64_t endTime;              // task end time;
    uint64_t taskTime;             // differance between end - start time
    uint64_t accTime;              // Accumulated Time

} tcb[MAX_TASKS];

struct _fic
{
    bool prInh;
    bool schedPri;
    bool preemption;
    bool rtos;
}rtos;

uint32_t heap[MAX_TASKS][256] __attribute__((aligned (1024)));

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
       {
            tcb[i].state = STATE_INVALID;
            tcb[i].pid = 0;
       }
    rtos.schedPri   = true;
    rtos.prInh      = true;
    rtos.preemption = true;
    NVIC_ST_CTRL_R     = 0;                                                          // Clear Control bit
    NVIC_ST_CURRENT_R  = 0;
    NVIC_ST_RELOAD_R   = 0x9C3F;                                                     // Set for 1Khz value=40000-1
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
        {
            task = 0;
        }

        // Round Robbin Scheduler
        if(rtos.schedPri == false)
        {
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }

        // Priority Scheduler
        /*use skip to count the scheduling delay*/
        else if(rtos.schedPri == true)
        {
            if(tcb[task].skip!=0)
            {
                tcb[task].skip--;
                ok = false;
            }
            else if(tcb[task].skip == 0)
            {  // once task is found give skip it Current priority
               tcb[task].skip = tcb[task].currentPriority;
               ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
            }
        }
    }
    return task;
}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;

    // REQUIRED: store the thread name DONE
    //add task if room in task list
    // allocate stack space for a thread and assign to sp below
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid ==  fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].sp = &heap[i][255];                                   //set the starting address of the SP function with array
            tcb[i].spInit = &heap[i][255];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            tcb[i].skip = tcb[i].currentPriority;
            //store the name of 'i' task
            strCopy(tcb[i].name,name);
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{
    __asm(" SVC #106");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void destroyThread(_fn fn)
{
    __asm(" SVC #105");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t i = 0;

    for(i=0; i<MAX_TASKS; i++)
    {
        if(tcb[i].pid == fn)                                   // look for the task in the tcb for the matching pid
        {
            tcb[i].priority = priority;                        // make the priority = priority
            tcb[i].currentPriority = priority;                 // make the current priority = priority
            break;
        }
    }
}

struct semaphore* createSemaphore(uint8_t count, char name[])
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
        strCopy(pSemaphore->semaName,name);
    }
    return pSemaphore;
}

void _sPSP(void *fun)
{
        __asm(" MOV R1,#2");
        __asm(" MSR CONTROL,R1");
        __asm(" MSR PSP,R0");
        __asm(" ISB "      );
}

// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    _fn fn;

    CurrentTask= rtosScheduler();           //Get the task num

    _sPSP(tcb[CurrentTask].spInit);
    fn = (_fn)tcb[CurrentTask].pid;
    tcb[CurrentTask].state = STATE_READY;
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;   // Enable Systick in startRtos
    WTIMER5_TAV_R = 0;
    tcb[CurrentTask].startTime = 0;         // set starttime of CurrentTask = 0;

    // shift to Unpriviledge
    __asm(" MOV R1,#3");
    __asm(" MSR CONTROL,R1");

    (*fn)();

}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
    __asm(" SVC #101");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded badck to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #102");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm(" SVC #103");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC #104");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    static uint8_t count = 0;

    count++;                                                        // Increase every-time Systick is called:
    if(count == 200)
    {
        uint8_t i=0;
        WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                           //  Turn off Wide Timer 5 while calculating
        totalTime = 0;                                              // Make totaltime = 0  for every 200 ms cycle for time calculation

        for( i=0;i<MAX_TASKS;i++)
        {
            tcb[i].taskTime = (tcb[i].endTime - tcb[i].startTime); // record task time in one run
            tcb[i].accTime += tcb[i].taskTime;                     // Accumulate time for every 200ms run of cycle
            totalTime += tcb[i].accTime;                           // add total accumulated task time into total cpu time
            tcb[i].taskTime = 0;                                   // make the task time for next 200ms run 0
        }
        WTIMER5_TAV_R = 0;                                         // make it 0 for new beginning
        WTIMER5_CTL_R |= TIMER_CTL_TAEN;                           // Enable it
        count = 0;                                                 // set to zero before exiting the loop :
    }

    uint8_t runThread;
    for(runThread = 0 ; runThread < taskCount; runThread++)         // for all task
    {
         if(tcb[runThread].state == STATE_DELAYED)                  //if task state is dealy decrement the timeout
         {
             if(tcb[runThread].ticks > 0)                           // if ticks greater than 0 then do -- for running thread
                 tcb[runThread].ticks--;
             else
               tcb[runThread].state = STATE_READY;                  //else mark it ready
         }
     }
    // if preemption is on, call pendsvIsr for task switch
    if(rtos.preemption == true)
    {
            NVIC_INT_CTRL_R |= 0x10000000;                          // PendSv
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently

void pendSvIsr()
{
    /*Following assembly routing will do decrement and push
      operation for r4-r11 register on thread stack*/

    __asm(" MRS R3,PSP");
    __asm(" SUB R3,R3,#4");
    __asm(" STR R11,[R3]");

    __asm(" SUB R3,R3,#4");
    __asm(" STR R10,[R3]");

    __asm(" SUB R3,R3,#4");
    __asm(" STR R9,[R3]");

    __asm(" SUB R3,R3,#4");
    __asm(" STR R8,[R3]");

    __asm(" SUB R3,R3,#4");
    __asm(" STR R7,[R3]");

    __asm(" SUB R3,R3,#4");
    __asm(" STR R6,[R3]");

    __asm(" SUB R3,R3,#4");
    __asm(" STR R5,[R3]");

    __asm(" SUB R3,R3,#4");
    __asm(" STR R4,[R3]");
    __asm(" MSR PSP,R3");

    tcb[CurrentTask].sp = (uint32_t *)getPSP();                 //  get the latest PSP and store it in task.sp
    if(tcb[CurrentTask].state!=0)
    {
        tcb[CurrentTask].endTime += WTIMER5_TAV_R;                  //  Add the timer value into the endTime everytime Task comes
    }
    CurrentTask= rtosScheduler();                               //  get new task
    _sPSP(tcb[CurrentTask].sp);                                 //  set PSP to task sp
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                           //  Turn off Wide Timer 5 for new task starting
    WTIMER5_TAV_R = 0;                                          //  clear its value
    tcb[CurrentTask].startTime = 0;                 //  set new task time to 0
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;                            //  turn back on

    if( tcb[CurrentTask].state == STATE_READY)
    {
        /*Following assembly routing will do pop and increment
          operation for r4-r11 register on thread stack*/

        __asm(" MRS R3,PSP");
        __asm(" LDR R4,[R3]");
        __asm(" ADD R3,R3,#4");

        __asm(" LDR R5,[R3]");
        __asm(" ADD R3,R3,#4");

        __asm(" LDR R6,[R3]");
        __asm(" ADD R3,R3,#4");

        __asm(" LDR R7,[R3]");
        __asm(" ADD R3,R3,#4");

        __asm(" LDR R8,[R3]");
        __asm(" ADD R3,R3,#4");

        __asm(" LDR R9,[R3]");
        __asm(" ADD R3,R3,#4");

        __asm(" LDR R10,[R3]");
        __asm(" ADD R3,R3,#4");

        __asm(" LDR R11,[R3]");
        __asm(" ADD R3,R3,#4");
        __asm(" MSR PSP,R3");
    }

    if( tcb[CurrentTask].state == STATE_UNRUN)
    {
        tcb[CurrentTask].state = STATE_READY;
        _sPSP(tcb[CurrentTask].sp);

        /*Setting up a stack for unrun task to make
          it look like it had run before*/

        __asm(" MRS R0,PSP");
        __asm(" SUB R0,R0,#4");
        __asm(" MOV R1,#0x61000000");                       // Set xPSR value

        __asm(" STR R1,[R0]");
        __asm(" SUB R0,R0,#4");
        __asm(" MSR PSP,R0");
        pc_task = tcb[CurrentTask].pid;                     // Get the next task PID num
        __asm(" MOV R1,R0");
        __asm(" MRS R0,PSP");
        __asm(" STR R1,[R0]");                              // set the pc =  tcb[CurrentTask].pid

        __asm(" SUB R0,R0,#4");
        __asm(" MOV R1,#15");
        __asm(" STR R1,[R0]");                              // set lr = 15 for debug

        __asm(" SUB R0,R0,#4");
        __asm(" MOV R1,#14");
        __asm(" STR R1,[R0]");                              // r12 = 14 for debug

        __asm(" SUB R0,R0,#4");
        __asm(" MOV R1,#13");
        __asm(" STR R1,[R0]");                              // r3 = 13 for debug

        __asm(" SUB R0,R0,#4");
        __asm(" MOV R1,#12");
        __asm(" STR R1,[R0]");                              // r2 = 12 for debug

        __asm(" SUB R0,R0,#4");
        __asm(" MOV R1,#11");
        __asm(" STR R1,[R0]");                              // r1 = 11 for debug

        __asm(" SUB R0,R0,#4");
        __asm(" MOV R1,#10");
        __asm(" STR R1,[R0]");                              // r0 = 10 for debug

        __asm(" MSR PSP,R0");                               // Give back PSP new value to PSP

        // set LR to 0xFFFFFFFD on EXCEPTION_RETURN
        __asm(" MOV LR,#0xFF000000");
        __asm(" ORR LR,LR,#0x00FF0000");
        __asm(" ORR LR,LR,#0x0000FF00");
        __asm(" ORR LR,LR,#0x000000FD");
    }
}

uint8_t getSvcNumber()
{
    // get the value of SVC number he said Backup the PC and record the R0;

    __asm(" MRS r0,PSP");
    __asm(" ADD r0,#0x18");
    __asm(" LDR  r0,[r0]");
    __asm(" SUB r0,#2");
    __asm(" LDR r0,[r0]");
    __asm(" BX LR");

    return 0;
}

uint8_t getArgument()
{
    // get the value of argument from the function

    __asm(" MRS r0,PSP");
    __asm(" ADD r0,r0,#0x24");
    __asm(" LDR r0,[r0]");
    __asm(" BX LR");

    return 0;
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    uint8_t svcNumber   =     getSvcNumber();
    uint32_t funArg     =     getArgument();

    switch(svcNumber)
    {
        case SVC_yield:
        {
            //set PendSV bit:
            NVIC_INT_CTRL_R |= 0x10000000;                              // PendSV
            break;
        }

        case SVC_sleep:
        {
            tcb[CurrentTask].ticks = funArg;                            // get the timeout value
            tcb[CurrentTask].state = STATE_DELAYED;                     // Mark Current task as delay
            NVIC_INT_CTRL_R |= 0x10000000;                              // PendSV
            break;
        }

        case SVC_wait:
        {
            semaPT = (struct semaphore*)funArg;                         // get a point to the semaphore from R0
            if( semaPT->count > 0)                                      // If semaphore count is not zero -> count --
            {
                semaPT->count --;
                semaPT->lastUser = CurrentTask;
                tcb[semaPT->lastUser].semaphore = (void *)funArg;
            }
            else
            {
                semaPT->processQueue[semaPT->queueSize] = CurrentTask;  //(uint32_t)tcb[CurrentTask].pid;  // Add Current task to queue
                semaPT->queueSize++;                                    // once added , increase the size of the queue
                tcb[CurrentTask].state = STATE_BLOCKED;                 // Mark Current task as a blocked
                tcb[CurrentTask].semaphore = (void *)funArg;                    // Record Blocked semaphore for current task


            /*Following function will give the blocked process priority(high)
             * to the process which is using the same semaphore which has low priority
             *  so that it can be completed in less time than without priority inheretence*/
                if( rtos.prInh == true )
                {
                    if(tcb[semaPT->lastUser].name != tcb[0].name)
                    {
                        if(tcb[semaPT->lastUser].semaphore == tcb[CurrentTask].semaphore)
                        {
                            if(tcb[semaPT->lastUser].currentPriority < tcb[CurrentTask].currentPriority)
                            {
                                tcb[CurrentTask].currentPriority = tcb[semaPT->lastUser].currentPriority;
                            }
                        }
                    }
                }
            }
            NVIC_INT_CTRL_R |= 0x10000000;                              // PendSV
            break;
        }

        case SVC_post:
        {
            semaPT = (struct semaphore*)funArg;                         //get a point to the semaphore from R0
            // if someone is queue is not emplty
            semaPT->count++;
            semaphores[CurrentTask].count= semaPT->count;

            // following part will give back the thread its priority back after wait
                tcb[CurrentTask].currentPriority = tcb[CurrentTask].priority;// for both pi on/off case give back current task it priority back

            if( semaPT->queueSize > 0)                                  // If queueSize is more than 0
            {
                uint8_t j;

                        tcb[semaPT->processQueue[0]].state = STATE_READY;// mark it ready
                        semaPT->count--;                                // dec the count
                        semaphores[CurrentTask].count= semaPT->count;

                            // following fun will shift the queue to left as one of the task is ready to run
                        for(j = 0; j < semaPT->queueSize-1; j++)
                        {
                            // task shift their position in queue
                            semaPT->processQueue[j] = semaPT->processQueue[j+1];
                        }
                        semaPT->processQueue[0] = 0;                    // release task then from the queue
                        semaPT->queueSize --;                           // Decrement the process queue size
            }
            NVIC_INT_CTRL_R |= 0x10000000;                              // PendSV
            break;
        }

        case SVC_destroy:
        {
            uint8_t i=0, taskd=0, j= 0;
            // find the task matching the pid value
            for (i=0;i<MAX_TASKS;i++)
            {
                if(tcb[i].pid == (void *) funArg)
                {
                    taskd = i;
                    break;
                }
            }
            /* following lines will look for the match
             * function task in the queue list and un queue it
             * so it will not run again
             *Once the task is un queue, move the other task
             * in the queue*/
            if(tcb[taskd].state == STATE_BLOCKED)
            {
                      //find the task semaphoire and give it to semaPT
                      semaPT = tcb[taskd].semaphore;
                      for( j = 0;j<semaPT->queueSize;j++)
                      {
                          if( semaPT->processQueue[j] == taskd)
                          {
                              semaPT->processQueue[j]=0;

                              /*Following loop will shift the queue from
                               * the task which is been kill to end*/
                              for(;j<semaPT->queueSize;j++)
                              {
                                  semaPT->processQueue[j] = semaPT->processQueue[j+1];
                              }
                              semaPT->queueSize--;
                          }
                      }
            }
            // mark the task as invalid and decrement the task count:
            tcb[taskd].state = STATE_INVALID;
            tcb[taskd].accTime = 0;
            tcb[taskd].endTime = 0;
            taskCount--;
            break;
        }

        case SVC_restart:
        {
            uint8_t i=0;
            for (i=0;i<MAX_TASKS;i++)
            {
                if(tcb[i].pid == (void *)funArg)
                {
                    if(tcb[i].state == STATE_INVALID)
                    {
                        tcb[i].state = STATE_UNRUN;                         // Mark it UnRun
                        tcb[i].sp = tcb[i].spInit;                          // Give the starting address back to run it again
                        taskCount++;
                        break;
                    }
                }
            }
            break;
        }
        case SVC_ipcs   :
        {
            putsUart0("\t   Name        |     Count    \t\t|    Waiting        \r\n");
            putsUart0("----------------|-----------------|-------------------\r\n");

            uint8_t i=0;
            char str[10];

            for (i = 0;i< MAX_SEMAPHORES - 1 ; i++)
            {
                putsUart0(semaphores[i].semaName);
                putsUart0("     \t\t\t");
                if(semaphores[i].count == 0)
                {
                    putsUart0("0");
                    putsUart0("     \t\t\t");
                }
                else
                {
                    i2a(semaphores[i].count,str);
                    putsUart0(str);
                    putsUart0("     \t\t\t");
                }
                if(semaphores[i].processQueue[0] == 0)
                {
                    putsUart0("NONE");
                }
                else
                {
                    putsUart0(tcb[semaphores[i].processQueue[0]].name);
                }
                putsUart0("     \t\t\t");
                putsUart0("\r\n");
            }
            break;
        }

        case SVC_ps:
        {
            putsUart0("\r\n");
            putsUart0("\t   PID        |     NAME       \t\t|\t    STATE       \t|        CPU%         \r\n");
            putsUart0("----------------|-----------------|-------------------|--------------------\r\n");

            uint8_t i=0;
            uint8_t j=0;
            for(i=0;i<MAX_TASKS;i++)
            {
                char str[20];
                // Print the name of the task
                hh((uint32_t)tcb[i].pid);
                putsUart0("     \t");
                putsUart0(tcb[i].name);
                putsUart0("     \t\t");
                j = tcb[i].state;

                switch(j)
                {
                    case 0:
                        putsUart0("INVALID");
                        break;
                    case 1:
                        putsUart0("  UNRUN");
                        break;
                    case 2:
                        putsUart0("  READY");
                        break;
                    case 3:
                        putsUart0("DELAYED");
                        break;
                    case 4:
                        putsUart0("BLOCKED");
                        break;
                }
                putsUart0("     \t\t");
//                i2a((tcb[i].accTime/totalTime)*10000),str);
                uint16_t x = (tcb[i].accTime*10000/totalTime);
                tcb[i].accTime = 0;                           // make accTime = 0  for next run:
                i2a(x,str);
                switch(str_Len(str))
                {
                    case 0:
                    {
                        putsUart0("00.00");
                        break;
                    }

                    case 1:
                    {
                        putsUart0("00.0");
                        putcUart0(str[0]);
                        break;
                    }

                    case 2:
                    {
                        putsUart0("00.");
                        putcUart0(str[0]);
                        putcUart0(str[1]);
                        break;
                    }

                    case 3:
                    {
                        putsUart0("0");
                        putcUart0(str[0]);
                        putsUart0(".");
                        putcUart0(str[1]);
                        putcUart0(str[2]);
                        break;
                    }

                    case 4:
                    {
                        putcUart0(str[0]);
                        putcUart0(str[1]);
                        putsUart0(".");
                        putcUart0(str[2]);
                        putcUart0(str[3]);
                        break;
                    }
                }
                putsUart0("%");
                putsUart0("\r\n");
            }
            break;
        }

        case SVC_piON:
        {
            rtos.prInh = true;
            break;
        }

        case SVC_piOFF:
        {
            rtos.prInh = false;
            break;
        }

        case SVC_premON:
        {
            rtos.preemption = true;
            break;
        }

        case SVC_premOFF:
        {
            rtos.preemption = false;
            break;
        }

        case SVC_schedON:
        {
            rtos.schedPri = true;
            break;
        }

        case SVC_schedOFF:
        {
            rtos.schedPri = false;
            break;
        }

        case SVC_pidof:
        {
            uint8_t i=0;
            char str[10];

            for(i=0;i<MAX_TASKS;i++)
            {
               if(str_Cmp(tcb[i].name,(void *)funArg)==0)
               {
                   funArg = (uint32_t)tcb[i].pid;
                   i2a(funArg,str);
                   putsUart0(str);
                   putsUart0("\r\n");
                   hh((uint32_t)tcb[i].pid);
                   putsUart0("\r\n");
                   break;
               }
            }
            break;
        }

        case SVC_kill:
        {
            uint8_t i=0, taskd=0, j= 0;
                        // find the task matching the pid value
                        for (i=0;i<MAX_TASKS;i++)
                        {
                            if(tcb[i].pid == (void *) funArg)
                            {
                                taskd = i;
                                break;
                            }
                        }

                        /* following lines will look for the match
                         * function task in the queue list and un queue it
                         * so it will not run again
                         * Once the task is un queue, move the other task
                         * in the queue*/

                        // same as SVC_destroy
                        if(tcb[taskd].state == STATE_BLOCKED)
                        {
                                  semaPT = tcb[taskd].semaphore;
                                  for( j = 0;j<semaPT->queueSize;j++)
                                  {
                                      if( semaPT->processQueue[j] == taskd)
                                      {
                                          semaPT->processQueue[j]=0;            // release task from queue

                                          /*Following loop will shift the queue from
                                           * the task which is been kill to end*/
                                          for(;j<semaPT->queueSize;j++)
                                          {
                                              semaPT->processQueue[j] = semaPT->processQueue[j+1];
                                          }
                                          semaPT->queueSize--;
                                      }
                                  }
                        }
                        // mark the task as invalid and decrement the task count:
                        tcb[taskd].state = STATE_INVALID;
                        tcb[taskd].accTime = 0;
                        tcb[taskd].endTime = 0;
                        taskCount--;
                        break;
        }

        case SVC_reset:
        {
            NVIC_APINT_R= RESET;
            break;
        }

        case SVC_res:
        {
            // same as Restart:
            uint8_t i=0;
            for (i=0;i<MAX_TASKS;i++)
            {
                char str[10];
//                char *x = tcb[i].name;
                upLow(tcb[i].name,str);
                if(str_Cmp(str,(void *)funArg)==0)
                {
                    if(tcb[i].state == STATE_INVALID)
                    {
                        tcb[i].state = STATE_UNRUN;                         // Mark it UnRun
                        tcb[i].sp = tcb[i].spInit;                          // Give the starting address back to run it again
                        taskCount++;
                    }
                    break;
                }
            }
            break;
        }
    }
}

// REQUIRED: code this function
void mpuFaultIsr()
{
    putsUart0("MPU Fault Occur in  ");
    putsUart0(tcb[CurrentTask].name);

    putsUart0("\r\n");
    putsUart0("\r\n\r");
    putsUart0("---------------------------------");
    putsUart0("\r\n\r");
        putsUart0(" MSP: ");
    hh(getMSP());
    putsUart0("\r\n");
        putsUart0(" PSP: ");
    hh(getPSP());
    putsUart0("\r\n");
        putsUart0("Fault address: ");
    hh(NVIC_MM_ADDR_R);
    putsUart0("\r\n");
    putsUart0("---------------------------------");
    putsUart0("\r\n");
    putsUart0("***********PROCESS STACK DUMP***********");
    putsUart0("\r\n");
    putsUart0("---------------------------------");
    putsUart0("\r\n");
        putsUart0(" xPSR: ");
    hh(getxPSR());
    putsUart0("\r\n");
        putsUart0(" PC:  ");
    hh(getPC());
    putsUart0("\r\n");
        putsUart0(" LR:  ");
    hh(getLR());
    putsUart0("\r\n");
        putsUart0(" R12:  ");
    hh(getR12());
    putsUart0("\r\n");
        putsUart0(" R3: ");
    hh(getR3());
    putsUart0("\r\n");
        putsUart0(" R2:  ");
    hh(getR2());
    putsUart0("\r\n");
        putsUart0(" R1 :  ");
    hh(getR1());
    putsUart0("\r\n");
        putsUart0(" R0:  ");
    hh(getR0());

    NVIC_SYS_HND_CTRL_R &= ~NVIC_SYS_HND_CTRL_MEM;
    NVIC_INT_CTRL_R =  NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: code this function
void hardFaultIsr()
{
    putsUart0("Hard Fault Occur in  ");
    putsUart0(tcb[CurrentTask].name);
    while(1){}
}

// REQUIRED: code this function
void busFaultIsr()
{
    putsUart0("Hard Fault Occur in  ");
    putsUart0(tcb[CurrentTask].name);
    while(1){}
}

// REQUIRED: code this function
void usageFaultIsr()
{
    putsUart0("Hard Fault Occur in  ");
    putsUart0(tcb[CurrentTask].name);
    while(1){}
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           5 pushbuttons, and uart
void initHw()
{
    //Configure HW to work with 16 MHz XTAL, PLL enabled, sysdivider of 5, creating system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    //Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    SYSCTL_GPIOHBCTL_R |= 0;

    //Enable GPIO port F ,A,B,C and E peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOC;
    SYSCTL_RCGCADC_R |=  1;                             // turn on ADC module 0 clocking
    SYSCTL_RCGCHIB_R |=  1;                             // turn-on Hibernation clock
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;            // turn-on UART0, leave other UARTs in same status
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;        // Enable Wide-Timer 5

    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;// select UART0 to drive pins PA0 and PA1: default, added for clarity

    //Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
    UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    //Configure Blue LED:
    GPIO_PORTF_DIR_R    |= 4;                        // make bit an output
    GPIO_PORTF_DR2R_R   |= 4;                        // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R    |= 4;

    OffBoardLED();
    OffBoardButtons();


    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                                               // Turn-off Counter before configuring
    WTIMER5_CFG_R = 4;                                                              // Configure as 32-bit counter
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TACDIR |TIMER_TAMR_TAMR_CAP ;    // Configure forCount Up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;
    WTIMER5_TAV_R = 0;
}

void OffBoardLED()
{
    GPIO_PORTE_DIR_R    |= 0x1E;                      // make bit an output
    GPIO_PORTE_DR2R_R   |= 0x1E;                      // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTE_DEN_R    |= 0x1E;
}

void OffBoardButtons()
{
    GPIO_PORTA_DEN_R |= 0xFC;           // Enable Digital for PORTA pins
    GPIO_PORTA_PUR_R |= 0xFC;           // Enable Internal Pull-up for push buttons
}

void setUpMPU()
{
        //  Configure Memory Protection Unit
        //  for Flash
        //  AP 011//TEX 000//S 0//C 1//B 0//SIZE 10001
        NVIC_MPU_NUMBER_R = 2;
        NVIC_MPU_BASE_R = 0x00000000;
        NVIC_MPU_ATTR_R = 0x03020023;
        NVIC_MPU_CTRL_R |=  NVIC_MPU_CTRL_ENABLE | NVIC_MPU_CTRL_PRIVDEFEN;

        //  for SRAM:
        //  AP 011//TEX 000//S 0//C 1//B 1//SIZE 01110
        NVIC_MPU_NUMBER_R = 1;
        NVIC_MPU_BASE_R = 0x20000000;
        NVIC_MPU_ATTR_R = 0x0303001D;
        NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE | NVIC_MPU_CTRL_PRIVDEFEN;

        //  AP 011//TEX 000//S 1//C 0//B 1//SIZE 1111
        NVIC_MPU_NUMBER_R = 0;
        NVIC_MPU_BASE_R = 0x00000000;
        NVIC_MPU_ATTR_R = 0X0306003F;
        NVIC_MPU_CTRL_R |=  NVIC_MPU_CTRL_ENABLE | NVIC_MPU_CTRL_PRIVDEFEN;

        // MPU0-3 for SRAM starting from 0x2000.0000 - 0x2000.7fff
        // AP 010//TEX 000//S 1//C 1//B 0// SIZE 01100
        // MPU 0 for SRAM region 1 0x2000.0000 - 0x2000.1FFF
        NVIC_MPU_NUMBER_R = 3;
        NVIC_MPU_BASE_R = 0x20000000;
        NVIC_MPU_ATTR_R = 0x0106FF19;
        NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE | NVIC_MPU_CTRL_PRIVDEFEN;

        // MPU 1 for SRAM region 1 0x2000.2000 - 0x2000.3FFF
        NVIC_MPU_NUMBER_R = 4;
        NVIC_MPU_BASE_R = 0x20002000;
        NVIC_MPU_ATTR_R = 0x0106FF19;
        NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE | NVIC_MPU_CTRL_PRIVDEFEN;

        // MPU 1 for SRAM region 1 0x2000.4000 - 0x2000.5FFF
        NVIC_MPU_NUMBER_R = 5;
        NVIC_MPU_BASE_R = 0x20004000;
        NVIC_MPU_ATTR_R = 0x0106FF19;
        NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE  | NVIC_MPU_CTRL_PRIVDEFEN;

        // MPU 1 for SRAM region 1 0x2000.6000 - 0x2000.7FFF
        NVIC_MPU_NUMBER_R = 7;
        NVIC_MPU_BASE_R = 0x20006000;
        NVIC_MPU_ATTR_R = 0x0106FF19;
        NVIC_SYS_HND_CTRL_R |= NVIC_SYS_HND_CTRL_MEM;                           //Enable MPU interrrupt
        NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE  | NVIC_MPU_CTRL_PRIVDEFEN;
}

// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
                                                // Approx clocks per us
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

// REQUIRED: add code to return a value from 0-31 indicating which of 5 PBs are pressed
uint8_t readPbs()
{
    uint8_t pbN = 0;

    if(!pb0)
    {
        pbN = 1;
    }
    if(!pb1)
    {
        pbN = 2;
    }
    if(!pb2)
    {
        pbN = 4;
    }
    if(!pb3)
    {
        pbN = 8;
    }
    if(!pb4)
    {
        pbN = 16;
    }
    if(!pb5)
    {
        pbN = 32;
    }

    return pbN;
}

//------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------
void getUart0String()
{

    count=0;

    while (uartStr != 0x00 )                                //will run till string reaches NULL character
    {
        char c = getcUart0();

        if( c == 0x08 )                                     // if c = backspace check count and take action accordingly
        {
           if ( count >0 )                                  // if count is greater then 0 then minus a count to go back to privous charachater
           {
               count--;
           }
           continue;
        }
        if ( c == 0x0D )                                    // if c = enter i.e carrage return, add NULL character and exit program
        {
            uartStr[count] = 0x00;
            break;
        }
        if ( c >= 0x20 )                                    // anything greater than spacebar i.e 0x20 is unprintable:
        {
                uartStr[count++] = c;

            if ( count == MAX_CHARS )                       // If max character is enter by user then put NULL character and EXIT the program
            {
                uartStr[count] = 0x00;
                break;
            }
        }
    }
}

/*Function will parse the string
 * will check and increment the character till any delimeter reaches
 * This will also Add "NULL" when the set condition does not match*/
void parseUart0String()
{
    fieldCount = 0;
    uint8_t len=str_Len(uartStr);
    int i;
    for(i=0;i<len;i++)
    {
        if((((uartStr[i]>=45 && uartStr[i]<=57))||(uartStr[i]>='a' && uartStr[i]<='z') ||(uartStr[i]>='A' && uartStr[i]<='Z')||(uartStr[i]==95)) && (((uartStr[i+1]>=45 && uartStr[i+1]<=57))||(uartStr[i+1]>='a' && uartStr[i+1]<='z') ||(uartStr[i+1]>='A' && uartStr[i+1]<='Z')||(uartStr[i+1]==95)))
            {
                 offset[fieldCount]=i;
                 i++;
                    while((((uartStr[i]>=45 && uartStr[i]<=57))||(uartStr[i]>='a' && uartStr[i]<='z')||(uartStr[i]>='A' && uartStr[i]<='Z')||(uartStr[i]==95)) && (((uartStr[i+1]>=45 && uartStr[i+1]<=57))||(uartStr[i+1]>='a' && uartStr[i+1]<='z')||(uartStr[i+1]>='A' && uartStr[i+1]<='Z')||(uartStr[i+1]==95)))
                    {
                        i++;
                    }
                    fieldCount++;
            }
         else
            {
                 offset[fieldCount]=i+1;
            }
    }
    for(i=0;i<len;i++)
    {
        if((((uartStr[i]>=45 && uartStr[i]<=57))||(uartStr[i]>='a' && uartStr[i]<='z') ||(uartStr[i]>='A' && uartStr[i]<='Z')||(uartStr[i]==95)))
        {
            uartStr[i]=uartStr[i];
        }
        else
        {
            uartStr[i]=0x00;
        }
    }
}

void d2h(int n)
{
    char hexN[10] = {0,0,0,0,0,0,0,0,0,0};
    int i = 0;

    while(n!=0)
    {
        int temp=0;
        temp = n % 16;
        // if reminder is less than 10 -> add 48 to get hexadecimal Num
        if(temp < 10)
        {
            hexN[i] = temp + 48;
            i++;
        }
        // if reminder is > than 10 -> add 55 to get hexadecimal Num
        else
        {
            hexN[i] = temp + 55;
            i++;
        }
        n = n/16;
    }
    // following line of Code will print the Hexadecimal num in 0xFFFFFFFF format
    hexN[++i]='\0';

        if((i-1) == 1 )
           {
               putsUart0(" 0x0000000");
               putsUart0(hexN);
           }
        else
        {
            if ( (i-1) != 8)
            {
                i--;
                while(i != 8)
                {
                    hexN[i] = '0';
                    i++;
                }
                revSt(hexN,i);
                putsUart0(" 0x");
                putsUart0(hexN);
            }
            else
            {
                revSt(hexN,i-1);
                putsUart0(" 0x");
                putsUart0(hexN);
            }
        }
}

// Aux function for d2h
void hh(uint32_t x)
{
    uint32_t temp;
    temp = x;
    d2h(temp);
}

//copy str2 into str1
void strCopy(char *str1,const char *str2)
{
    uint8_t i = 0;
    while(str2[i]!='\0')
    {
        str1[i]=str2[i];
        i++;
    }
}

// function for ATOI
int a2i(char* str)
{
    int res = 0;
    int i   = 0;

    for (i=0; str[i] != '\0'; ++i)
        {
            res = res * 10 + str[i] - '0';
        }

    return res;
}

// function to reverse the string
void revSt(char str[], int len)
{
    int start = 0, end;
    char temp;

    for(end = len-1 ; start < end; end--)
    {
        temp = *(str+start);
        *(str+start) = *(str+end);
        *(str+end) = temp;
        start++;
    }
}

// Function to convert upper case to lower case:
char* upLow(char *str1,char *str2)
{
    int i=0;
    while(str1[i]!='\0')
    {
        if(str1[i]>='A' && str1[i]<='Z')
        {
            str2[i]=str1[i]+32;
        }
        else if(str1[i]>='a'&& str1[i]<='z')
        {
            str2[i]=str1[i];
        }
        else if(str1[i]>='0' && str1[i]<='9')
        {
            str2[i]=str1[i];
        }
        ++i;
    }
    str2[i] ='\0';
    return str2;
}

// function for ITOA
char *i2a(uint16_t num, char* str)
{
 int i = 0;
 if(num == 0)
 {
     str[i] = 0;
     str[i++] = '\0';
     return str;
 }

 while(num)
 {
     int rem = num % 10;
     if(rem > 9)
     {
        str[i++] = (rem - 10) + 'a' ;
     }
     else
     {
        str[i++] = rem + '0';
     }
     num = num/10;
 }
     str[i] = '\0';
     revSt(str,i);

 return str;
}

//Function to compare the string
int str_Cmp(char *str1,char *str2)
{
    uint8_t i = 0;
 x:   while( (str1[i]) != '\0' || str2[i] != '\0' )
    {
        if ( str1[i] == str2[i])
        {
            i++;
            goto x;
        }
        else
            return 1;
    }
    return 0;
}

//Function to get a string length
int str_Len(char *str)
{
    int scount=0;
    int i=0;

    while( str[i] !='\0')
    {
        i++;
    }
    scount = i;

    if( scount == 0)
    {
        return scount+1;
    }

    return scount;
}

bool isCommand(char *commandName,int min_arg)
{
    if(str_Cmp(&uartStr[offset[0]],commandName)==0)
    {
        if (fieldCount>min_arg)
        {
            return true;
        }
    }
        return false;
}

int getValue(int argNumber)
{
    return a2i(&uartStr[offset[argNumber+1]]);
}

char* getString(int argNumber)
{
    return (&uartStr[offset[argNumber+1]]);
}

void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);
    UART0_DR_R = c;
//    yield();                              // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char *str)
{
    uint8_t i;
    for (i = 0; i < str_Len(str); i++)
     putcUart0(str[i]);
}

char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)                // wait if uart0 rx fifo empty
        yield();                                     // Dont wait and yield()
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void reboot()
{
    __asm(" SVC #117");
}

void ps()
{
     putsUart0("**PS called**");
     putsUart0("\r\n");
     __asm(" SVC #108");
}

void ipcs()
{
     putsUart0("**IPCS called**");
     putsUart0("\r\n");
     __asm(" SVC #107");
}

void kill(uint16_t pid)
{
    __asm(" SVC #116");
}

void pi(char *str)
{
    if ((str_Cmp(str,"ON")==0))
    {
        putsUart0("Priority inheritance is ON");
        putsUart0("\r\n");
        __asm(" SVC #109");
    }

    if ((str_Cmp(str,"OFF")==0))
    {
        putsUart0("Priority inheritance is OFF");
        putsUart0("\r\n");
        __asm(" SVC #110");
    }
}

void preempt(char *str)
{
        if ((str_Cmp(str,"ON")==0))
        {
            putsUart0("Preemption is ON");
            putsUart0("\r\n");
            __asm(" SVC #111");
        }

        if ((str_Cmp(str,"OFF")==0))
        {
            putsUart0("Preemption is OFF");
            putsUart0("\r\n");
            __asm(" SVC #112");
        }
}

void sched( char *str)
{

        if ((str_Cmp(str,"PRIO")==0))
        {
            putsUart0("priority sched is ON");
            putsUart0("\r\n");
            __asm(" SVC #113");
        }

        if ((str_Cmp(str,"RR")==0))
        {
            putsUart0("priority sched is OFF");
            putsUart0("\r\n");
            __asm(" SVC #114");
        }
}

void pidof(char name[])
{
    __asm(" SVC #115");
}

void rest(char task[])
{
    __asm(" SVC #118");
}
// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{
    while(true)
    {
        ORANGE_LED = 1;
        waitMicrosecond(1000);
        ORANGE_LED = 0;
        yield();
    }
}

void flash4Hz()
{
    while(true)
    {
        GREEN_LED ^= 1;
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        YELLOW_LED = 1;
        sleep(1000);
        YELLOW_LED = 0;
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i <5000; i++)
        {
            partOfLengthyFn();
        }
        RED_LED ^= 1;
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            YELLOW_LED ^= 1;
            RED_LED = 1;
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            RED_LED = 0;
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            destroyThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        BLUE_LED = 1;
        sleep(1000);
        BLUE_LED = 0;
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    while (true)
    {
        getUart0String();                                                       //Get a uart string
        parseUart0String();                                                     //parce a string for unwanted symbols
        if  (isCommand("reboot",0))
            {
                reboot();
            }
            else if(isCommand("ps",0))
            {
                ps();
            }
            else if(isCommand("ipcs",0))
            {
                ipcs();
            }
            else if(isCommand("kill",1))
            {
                int v = getValue(0);
                kill(v);
            }
            else if(isCommand("pi",1))
            {
                char* v = getString(0);
                pi(v);
            }
            else if(isCommand("preempt",1))
            {
                char* v = getString(0);
                preempt(v);
            }
            else if(isCommand("sched",1))
            {
                char* v = getString(0);
                sched(v);
            }
            else if(isCommand("pidof",1))
            {
                char* v = getString(0);
                pidof(v);
            }
            // sub command for proc_name&
            else if(isCommand("restart",1))
            {
                char* v = getString(0);
                rest(v);
            }
            else
            {
                char* v = getString(1);
                rest(v);
            }
    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;

    // Initialize hardware
    initHw();
    initRtos();
    setUpMPU();
    // Power-up flash
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);
    // Initialize semaphores
    keyPressed = createSemaphore(1,"keyPressed");
    keyReleased = createSemaphore(0,"keyReleased");
    flashReq = createSemaphore(5,"flashReq");
    resource = createSemaphore(1,"resource");

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 15, 1024);
//    ok =  createThread(idle1, "Idle1", 15, 1024);                 // idle1 just to check 5B

    // Add other processes
    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    ok &= createThread(debounce, "Debounce", 12, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 10, 1024);
    ok &= createThread(errant, "Errant", 8, 1024);
    ok &= createThread(shell, "Shell", 8, 1024);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;
    return 0;
}
