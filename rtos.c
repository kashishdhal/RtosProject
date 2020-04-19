// RTOS Framework - Spring 2020
// J Losh

// Student Name: Kashish Dhal
// TO DO: Add your name on this line.  Do not include your ID number in the file.

// Add xx_ prefix to all files in your project
// xx_rtos.c
// xx_tm4c123gh6pm_startup_ccs.c
// xx_other files (except uart0.x and wait.x)
// (xx is a unique number that will be issued in class)
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PA2
// Orange: PA3
// Yellow: PA4
// Green:  PE0
// PBs on these pins
// PB0:    PC4
// PB1:    PC5
// PB2:    PC6
// PB3:    PC7
// PB4:    PD6
// PB5:    PD7


//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "wait.h"

// REQUIRED: correct these bitbanding references for the off-board LEDs
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // on-board blue LED
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 2*4))) // off-board red LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 0*4))) // off-board green LED
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 4*4))) // off-board yellow LED
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 3*4))) // off-board orange LED


#define BLUE_LED_MASK 4
#define RED_LED_MASK 4
#define ORANGE_LED_MASK 8
#define YELLOW_LED_MASK 16
#define GREEN_LED_MASK 1
#define PB0_MASK 16
#define PB1_MASK 32
#define PB2_MASK 64
#define PB3_MASK 128
#define PB4_MASK 64
#define PB5_MASK 128

#define MAX_CHARS 80
#define MAX_FIELDS 6
#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")


// Global Variables for string processing
#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
char str1[MAX_CHARS+1];
char str2[MAX_CHARS+1];
uint8_t count=0;
char str[MAX_CHARS+1];
char intStr[10];
uint8_t pos[MAX_FIELDS];
uint8_t argCount=0;
uint8_t priorityScheduler=1;
uint8_t firstTime=1;
uint8_t preempt = 0; // Note this is not working perfectly so I have set it to 0. For default behavior (for project), set it to 1
uint8_t pi = 1;

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();


// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5
struct semaphore
{
    uint16_t count;
    uint16_t queueSize;
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
    char name[16];
    uint16_t previousUser;
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore


// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

uint32_t stack[MAX_TASKS][512];

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
    uint32_t sum;
    uint32_t stopValue;
    uint32_t status;
} tcb[MAX_TASKS];



void printline(){putsUart0("\r\n----------------------------------------------------------------\r\n");}



void yield();


// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE){yield();}               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}


// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}


// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char*str)
{
    uint8_t i;
    for (i = 0; i <strlen(str) ; i++)
      putcUart0(str[i]);
}

void getString(char str[], uint8_t maxChars)
{

    char c; count = 0;
        Loop1:
        c = getcUart0();
          if(c == 8 | c == 127)
            {
                if(count>0){count--;goto Loop1;}
                else{goto Loop1;}
            }

          if(c == 10 | c == 13){str[count] = 0x00; return;}
          else
            {
                if(c>=32){str[count++] = c;}
                else goto Loop1;
            }

            if(count == maxChars)
            {
                str[count] = 0x00;
                putsUart0("You have exceeded the maximum characters, you typed\r\n");
                return;
             }
            else goto Loop1;
}




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
}

// REQUIRED: Implement prioritization to 16 levels
int rtosScheduler()
{
    uint8_t task1; uint8_t priority=0;
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    if(!priorityScheduler)
    {
        while (!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
        return task;
    }
    else
    {
        if(firstTime || taskCurrent==taskCount-1)
        {
            task1=0;
            firstTime=0;
        }
        else
            task1=taskCurrent+1;
        while(!ok)
        {

            if(tcb[task1].currentPriority==priority)// && task1!=taskCurrent)
            {
                 ok = (tcb[task1].state == STATE_READY || tcb[task1].state == STATE_UNRUN);
                 if(ok)
                 {
                    break;
                 }  //dispatch the task
            }

            if((taskCurrent-task1)==0)
            {
                if(priority<15)
                    priority++;
                else if (priority==15)
                    priority=0;
            }

            if(task1<taskCount-1)
                task1++;
            else if (task1==(taskCount-1))
                task1=0;


        }
        return task1;
        }


}

bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED: store the thread name

    // add task if room in task list
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
            tcb[i].sp = &stack[i+1][0];
            tcb[i].spInit = &stack[i+1][0];
            tcb[i].priority = priority;
            tcb[i].currentPriority = priority;
            strcpy(tcb[i].name,name);

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
    __asm("   SVC #5");
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
    __asm("   SVC #6");
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    uint8_t i;
    for (i=0;i<taskCount;i++)
    {
        if(tcb[i].pid == fn)
        {
         tcb[i].priority = priority;
         break;
        }

    }
}

struct semaphore* createSemaphore(uint8_t count, char* str)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
        strcpy(pSemaphore->name,str);
    }
    return pSemaphore;
}

extern void setSp(uint32_t sp);
extern void setPsp(uint32_t *);
extern void turnPspOn(uint32_t *);
extern void pushReg();
extern void popReg();
extern uint32_t* getPsp();
extern uint32_t getSvcNo();
extern uint32_t getR0();
extern void setLr();



// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{

    taskCurrent = rtosScheduler();
    tcb[taskCurrent].state=STATE_READY;
    NVIC_ST_CTRL_R |= (1+2+4);                    //  set 0th, 1st and 2nd bit of Control Register
    turnPspOn((uint32_t *)tcb[taskCurrent].sp);
//    setPsp((uint32_t *)tcb[taskCurrent].sp);
    _fn fn = (_fn)(tcb[taskCurrent].pid);
    (*fn)();
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
// push registers, call scheduler, pop registers, return to new function
void yield()
{
   __asm(" SVC #1 ");
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
// push registers, set state to delayed, store timeout, call scheduler, pop registers,
// return to new function (separate unrun or ready processing)
void sleep(uint32_t tick)
{
    __asm(" SVC #2 ");
}

// REQUIRED: modify this function to wait a semaphore with priority inheritance
// return if avail (separate unrun or ready processing), else yield to scheduler using pendsv
void wait(struct semaphore *pSemaphore)
{
    __asm(" SVC #3 ");
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(struct semaphore *pSemaphore)
{
    __asm(" SVC #4 ");
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;
    for (i=0; i<taskCount;i++)
    {
        if (tcb[i].state==STATE_DELAYED & tcb[i].ticks>0)
            {
                tcb[i].ticks--;
            }
            else if(tcb[i].state==STATE_DELAYED & tcb[i].ticks==0)
            {
                tcb[i].state=STATE_READY;
            }

    }
    if(preempt)
    {
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
    }

}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    pushReg();
    tcb[taskCurrent].sp = getPsp();



    uint8_t i =0;
    for(i=0;i<MAX_TASKS;i++)
    {
        if(i==taskCurrent)
        {
            tcb[i].stopValue =  TIMER1_TAV_R;
            tcb[i].sum = ( 7*tcb[i].sum + tcb[i].stopValue  )>>3;
        }
        else
       {
            tcb[i].sum = 7*tcb[i].sum >> 3;
       }
    }


    taskCurrent = rtosScheduler(); // Dispatch the task

    TIMER1_TAV_R = 0; // Reset the timer when the task is dispatched

    setPsp(tcb[taskCurrent].sp);

    if (tcb[taskCurrent].state == STATE_READY)
       {
        popReg();
       }
    else
    {
        uint32_t *p = getPsp();
        p=p-1;
        *p = (1<<24); // write 1 to 24th bit of xPSR
        p=p-1;
        *p = (uint32_t)tcb[taskCurrent].pid; // write to PC
        p = p-6;
        setPsp(p);
        tcb[taskCurrent].state = STATE_READY;
      // setLr();
    }



}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{

    uint8_t N = (uint8_t)getSvcNo();
    //uint32_t* s = (uint32_t*)getR0();
    uint32_t* R0 = (uint32_t*)getR0();
    struct semaphore * s = (struct semaphore *)*R0;
    _fn fn;
    uint8_t i;

    switch(N)
    {
    case 1: //yield
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;

    case 2: //sleep
        tcb[taskCurrent].state = STATE_DELAYED;
        tcb[taskCurrent].ticks = *R0; //getR0();
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;
    case 3: // wait
        if(s->count>0){s->count--;}
        else
        {
            s->processQueue[s->queueSize++] = taskCurrent;
            tcb[taskCurrent].state = STATE_BLOCKED;
            tcb[taskCurrent].semaphore = s;
            if(pi==1 && tcb[taskCurrent].state==STATE_BLOCKED && tcb[s->previousUser].priority<tcb[taskCurrent].priority)
            {
                tcb[taskCurrent].currentPriority = 0;
            }
            NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        }
        s->previousUser = taskCurrent;
        break;
    case 4: //post
        s->count++;
        if(s->queueSize>0)
        {
            tcb[s->processQueue[0]].state = STATE_READY;
            tcb[s->processQueue[0]].semaphore = NULL;

            for (i = 0; i < s->queueSize; i++)
                {
                s->processQueue[i] = s->processQueue[i + 1];
                }
            s->queueSize--;
            s->count--;

        }
        if(pi==1)
        {
            tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
        }
        break;
    case 5: //restartThread
        fn = (_fn)*R0;
        for (i=0;i<taskCount;i++)
        {
            if(tcb[i].pid==fn && tcb[i].state==STATE_INVALID)
            {
             tcb[i].sp = tcb[i].spInit;
             tcb[i].state = STATE_UNRUN;
             break;
            }
        }
        break;
    case 6: //killThread
        fn = (_fn)*R0;
        for (i=0;i<taskCount;i++)
            {
                if(tcb[i].pid == fn)
                {
                 struct semaphore * s = (struct semaphore *)tcb[i].semaphore;
                 if(tcb[i].state == STATE_BLOCKED && s!=0)
                 {

                 uint8_t j;
                 for (j = 0; j <= s->queueSize; j++)
                 {
                       s->processQueue[j] = s->processQueue[j+1];
                 }

                 s->queueSize--;
                 }
                 tcb[i].state = STATE_INVALID;
                 break;
                }

            }
    case 7: // PI OFF
        for(i=0;i<MAX_TASKS;i++)
        {
            tcb[i].currentPriority = tcb[i].priority;
        }
    }
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons, and uart
void initHw()
{

    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
       SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

       // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
       // Note UART on port A must use APB
       SYSCTL_GPIOHBCTL_R = 0;

       // Enable GPIO port A C D F E peripherals
       SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOF | SYSCTL_RCGC2_GPIOE;

       // Configure Blue LED (on-board) ON PF2
        GPIO_PORTF_DIR_R |= BLUE_LED_MASK;  // bit 2 is output, other pins are inputs
        GPIO_PORTF_DR2R_R |= BLUE_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTF_DEN_R |= BLUE_LED_MASK;  // enable LED

        // Configure Red Orange Yellow LEDs (off-board) PA2 PA3 PA4
        GPIO_PORTA_DIR_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;  // bits 2,3,4 are output, other pins are inputs
        GPIO_PORTA_DR2R_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTA_DEN_R |= RED_LED_MASK | ORANGE_LED_MASK | YELLOW_LED_MASK;  // enable LEDs

        // Configure Green LED (off-board) ON PE0
        GPIO_PORTE_DIR_R |= GREEN_LED_MASK;  // bit 2 is output, other pins are inputs
        GPIO_PORTE_DR2R_R |= GREEN_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
        GPIO_PORTE_DEN_R |= GREEN_LED_MASK;  // enable LED

        // Configure Push Buttons 0-3 ON PC4 TO PC7
        GPIO_PORTC_DEN_R |= PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK; // enable push buttons
        GPIO_PORTC_PUR_R |= PB0_MASK | PB1_MASK | PB2_MASK | PB3_MASK; // enable internal pull-up for push button

        // Configure Push Buttons 4-5 ON PD6 AND PD7
        GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY;
        GPIO_PORTD_CR_R = 255;
        GPIO_PORTD_DEN_R |=  PB4_MASK | PB5_MASK; // enable push buttons
        GPIO_PORTD_PUR_R |=  PB4_MASK | PB5_MASK; // enable internal pull-up for push button


        // UART Interface:
        //   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
        //   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
        //   Configured to 115,200 baud, 8N1

        // Configure UART0 pins
           GPIO_PORTA_DIR_R |= 2;                           // enable output on UART0 TX pin: default, added for clarity
           GPIO_PORTA_DEN_R |= 3;                           // enable digital on UART0 pins: default, added for clarity
           GPIO_PORTA_AFSEL_R |= 3;                         // use peripheral to drive PA0, PA1: default, added for clarity
           GPIO_PORTA_PCTL_R &= 0xFFFFFF00;                 // set fields for PA0 and PA1 to zero
           GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
                                                            // select UART0 to drive pins PA0 and PA1: default, added for clarity

           // Configure UART0 to 115200 baud, 8N1 format
           SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other UARTs in same status
           delay4Cycles();                                  // wait 4 clock cycles
           UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
           UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
           UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
           UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
           UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
           UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;


        // Configure Timer 1 as the systickIsr

        SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; // Enable clocks
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
        TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
        TIMER1_TAMR_R =  TIMER_TAMR_TACDIR; //  count up // TIMER_TAMR_TAMR_CAP |
        TIMER1_TAILR_R = 40e6;                          // set load value to 40e3 for 1 KHz interrupt rate
        TIMER1_IMR_R = 0;                                 // turn-off interrupts
        TIMER1_TAV_R = 0;                               // zero counter for first period
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on counter
        NVIC_EN0_R &= ~(1 << (INT_TIMER1A-16));           // turn-off interrupt 37 (TIMER1A)


           // Configure the systickIsr

          NVIC_ST_RELOAD_R = 40000-1;                   // set load value to 40e3-1 clocks for 1 KHz interrupt rate
          NVIC_ST_CURRENT_R = 0;                        // Write any value to 0-23 bits of Current Register to enable the interrupt
//          NVIC_ST_CTRL_R |= (1+2+4);                    //  set 0th, 1st and 2nd bit of Control Register
}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    uint8_t pbValue=0;

    if((PB0_MASK & GPIO_PORTC_DATA_R)==0){pbValue += 1;}
    if((PB1_MASK & GPIO_PORTC_DATA_R)==0){pbValue += 2;}
    if((PB2_MASK & GPIO_PORTC_DATA_R)==0){pbValue += 4;}
    if((PB3_MASK & GPIO_PORTC_DATA_R)==0){pbValue += 8;}
    if((PB4_MASK & GPIO_PORTD_DATA_R)==0){pbValue += 16;}
    if((PB5_MASK & GPIO_PORTD_DATA_R)==0){pbValue += 32;}

    return pbValue;
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

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

void idle2()
{
    while(true)
    {
        YELLOW_LED = 1;
        waitMicrosecond(1000);
        YELLOW_LED = 0;
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
//    __asm("             MOV  R3, #990");          // 1
//    __asm("WMS_LOOP10:   MOV  R1, #6");          // 1
//    __asm("WMS_LOOP11:   SUB  R1, #1");          // 6
//    __asm("             CBZ  R1, WMS_DONE11");   // 5+1*3
//    __asm("             NOP");                  // 5
//    __asm("             NOP");                  // 5
//    __asm("             B    WMS_LOOP11");       // 5*2 (speculative, so P=1)
//    __asm("WMS_DONE11:   SUB  R3, #1");          // 1
//    __asm("             CBZ  R3, WMS_DONE10");   // 1
//    __asm("             NOP");                  // 1
//    __asm("             B    WMS_LOOP10");       // 1*2 (speculative, so P=1)
//    __asm("WMS_DONE10:");                        // ---
//                                                // 40 clocks/us + error
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        tcb[taskCurrent].status = 0;
        for (i = 0; i < 5000; i++)
        {
            tcb[taskCurrent].status = i;
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
        while (readPbs() == 32)
        {
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


void intToStr(char*intStr, uint32_t number)
{
    uint32_t i; uint32_t divider=1;
    for(i=0; i<10;i++)
    {
        divider = divider*10;
        if((uint32_t)(number/divider)<1)
            break;
    }
    uint8_t j; divider=1;
    for(j=0; j<=i;j++)
        {
            divider=divider*10;
            intStr[i-j]=(uint8_t)( 0x30 + (number%divider)/(divider/10) );
        }
    intStr[i+1] = 0x00;
    return;
}

uint32_t StrToInt(char* str2)
{
    uint8_t i; uint32_t divider=1;
        for(i=0; i<10;i++)
        {
            if(str2[i]<0x030 | str2[i]>0x039)
                {i--; break;}
        }

    uint8_t j; divider=1; uint32_t number=0;
    for(j=0;j<=i;j++)
    {
        number+=(uint8_t)(str2[i-j]-0x30)*divider;
        divider=divider*10;
    }

    return number;
}

void isCommand()
{
    uint8_t i =0; uint8_t j=0; uint8_t k=0;
    if(strcmp(str1,"pidof")==0)
    {
        for(i=0;i<MAX_TASKS;i++)
        {
            if(strcmp(tcb[i].name,str2)==0){break;}
        }
        putcUart0(0x0a); putcUart0(0x0d);
        uint8_t j;
        for(j=0;j<10;j++)
            intStr[j]=0;

        putsUart0("PID of "); putsUart0(tcb[i].name); putsUart0(" is ");
        intToStr(intStr,(uint32_t)tcb[i].pid);
        putsUart0(intStr);
    }

    else if(strcmp(str1,"ps")==0)
    {
        printline();
        putsUart0("PID   |  Name     |   Priority  |    State    |  % CPU Usage");
        printline();
        uint32_t totalSum=0; uint32_t cpuPcntI=0; uint32_t cpuPcntF=0;
        for(i=0;i<taskCount;i++)
        {
            totalSum += tcb[i].sum;
        }
        for(i=0;i<taskCount;i++)
        {
            for(j=0;j<10;j++){intStr[j]=0;}
            intToStr(intStr,(uint32_t)tcb[i].pid);
            putsUart0(intStr); putsUart0("     ");
            putsUart0(tcb[i].name);

            k=strlen(tcb[i].name);
            while(15-k>0){putsUart0(" ");k++;}


            for(j=0;j<10;j++){intStr[j]=0;}
            intToStr(intStr,tcb[i].priority);
            putsUart0(intStr);

            k=strlen(intStr);
            while(12-k>0){putsUart0(" ");k++;}

            if(tcb[i].state==0){putsUart0("INVALID");
            k=strlen("INVALID");
            while(18-k>0){putsUart0(" ");k++;}}
            else if(tcb[i].state==1){putsUart0("UNRUN");
            k=strlen("UNRUN"); while(18-k>0){putsUart0(" ");k++;}}
            else if(tcb[i].state==2){putsUart0("READY");
            k=strlen("READY"); while(18-k>0){putsUart0(" ");k++;}}
            else if(tcb[i].state==3){putsUart0("DELAYED");
            k=strlen("DELAYED"); while(18-k>0){putsUart0(" ");k++;}}
            else if(tcb[i].state==4){putsUart0("BLOCKED");
            k=strlen("BLOCKED"); while(18-k>0){putsUart0(" ");k++;}}

            cpuPcntI = (uint32_t)((tcb[i].sum*100)/totalSum);
            cpuPcntF = (uint32_t)((tcb[i].sum*10000)/totalSum) -cpuPcntI*100  ;
            for(j=0;j<10;j++){intStr[j]=0;}
            intToStr(intStr,cpuPcntI);
            putsUart0(intStr); putsUart0(".");

            for(j=0;j<10;j++){intStr[j]=0;}
            intToStr(intStr,cpuPcntF);
            putsUart0(intStr);
            putsUart0("\n\r");
        }
        for(j=0;j<10;j++){intStr[j]=0;}
        intToStr(intStr, (uint32_t)(tcb[1].status*100/5000));
        putsUart0("\n\r"); putsUart0("Status of Lengthy is "); putsUart0(intStr); putsUart0("%\n\r");
        printline();
    }

    else if(strcmp(str1,"ipcs")==0)
    {
         printline();
         putsUart0("  Name     |   Queue Size    |  Count");
         printline();

         for(i=0;i<semaphoreCount;i++)
         {

           putsUart0(semaphores[i].name);
           k=strlen(semaphores[i].name); while(20-k>0){putsUart0(" ");k++;}

           for(j=0;j<10;j++){intStr[j]=0;}
           intToStr(intStr,semaphores[i].queueSize);
           putsUart0(intStr);
           k=strlen(intStr); while(15-k>0){putsUart0(" ");k++;}

           for(j=0;j<10;j++){intStr[j]=0;}
           intToStr(intStr,semaphores[i].count);
           putsUart0(intStr); putsUart0("\n\r");

         }

         printline();
    }


    else if(strcmp(str1,"kill")==0)
        {
        uint32_t pid = StrToInt(str2);
        _fn fn = (_fn)pid;
        destroyThread(fn);

        }

    else if(strcmp(str2,"&")==0)
    {

        for(i=0;i<MAX_TASKS;i++)
        {
            if(strcmp(str1,tcb[i].name)==0)
                break;
        }
        _fn fn = (_fn)tcb[i].pid;
        restartThread(fn);
    }

    else if(strcmp(str1,"sched")==0)
        {
        if(strcmp(str2,"RR")==0)
        {
            priorityScheduler=0;
        }
        else if(strcmp(str2,"PRIO")==0)
        {
            priorityScheduler=1;
        }
        }

    else if(strcmp(str1,"pi")==0)
        {

        if(strcmp(str2,"ON")==0)
        {
            pi = 1;
        }
        else if(strcmp(str2,"OFF")==0)
        {
            __asm("   SVC #7");
//            tcb[1].currentPriority= tcb[1].priority;
            pi = 0;
        }

        }

    else if(strcmp(str1,"preempt")==0)
        {

        if(strcmp(str2,"ON")==0)
        {
            preempt = 1;
        }
        else if(strcmp(str2,"OFF")==0)
        {
            preempt = 0;
        }

        }
    else if(strcmp("reboot", str1)==0 )
            {
                putsUart0("\r\nRebooting.......................");
                NVIC_APINT_R = NVIC_APINT_VECTKEY| NVIC_APINT_SYSRESETREQ;
            }

    else if(strcmp("status", str1)==0 )
        {
            for(i=0;i<10;i++)
                intStr[i] =0;
             intToStr(intStr, tcb[1].status);
             putsUart0("\n\r"); putsUart0(intStr);
        }


    else{putsUart0("Invalid Command");}

    return;
}


void posArg(char*str)
{
    uint8_t i; argCount = 0;

    //empty out the argument position string
    for (i=0;i<5;i++){pos[i]=0;}

    // Here count is a global variable containing length of string
    for (i = 0; i < count; i++)
    {
        if(str[i]==32 | str[i]==44)
        {
            str[i]=0x00;
        }
        if(str[i]!=0x00 & str[i-1]==0x00)
        {
            pos[argCount] = i;
            argCount++;
        }

    }
    return;
}

void parseString(char* str,  uint8_t* pos, uint8_t argCount)
{
    uint8_t i; uint8_t tempCount=0;

    for(i=0;i<20;i++)
    {
        str1[i]=0; str2[i]=0;
    }

    for (i=pos[0];i<20;i++)
    {
        if(str[i]==0){break;}
        else str1[tempCount++] = str[i];
    }

    if(pos[1]!=0)
    {
    tempCount = 0;
    for (i=pos[1];i<20;i++)
    {
        if(str[i]==0){break;}
        else str2[tempCount++] = str[i];
    }
    }
    return;
}


void shell()
{
    while (true)
    {
        putcUart0(0x0a); putcUart0(0x0d); putcUart0(0x0a); putcUart0(0x0d);
        putsUart0("Please enter the command");
        putcUart0(0x0a); putcUart0(0x0d); putsUart0(">>");
        getString(str, MAX_CHARS);
        posArg(str); // To process the string, calculate the no. of arguments and their positions
        parseString(str,pos,argCount);
        isCommand();
        yield();
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

    putcUart0(0x0a); putcUart0(0x0d); putcUart0(0x0a); putcUart0(0x0d);
    printline();
    putsUart0("//////  This is the shell interface  ///////");
    printline();
//    putsUart0("Please enter the command");


    // Power-up flash
    //GPIO_PORTE_DATA_R |= 1;
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
//    ok &=  createThread(idle2, "Idle2", 15, 1024);

    // Add other processes

    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
    ok &= createThread(debounce, "Debounce", 12, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 12, 1024);
    ok &= createThread(shell, "Shell", 12, 1024);


    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
