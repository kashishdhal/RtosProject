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

char str[MAX_CHARS+1];
uint8_t pos[MAX_FIELDS];
uint8_t argCount=0;

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
} semaphores[MAX_SEMAPHORES];
uint8_t semaphoreCount = 0;

struct semaphore *keyPressed, *keyReleased, *flashReq, *resource, *s;

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

uint32_t stack[MAX_TASKS][511];

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
} tcb[MAX_TASKS];






// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
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

    char c; uint32_t count = 0;
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
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    while (!ok)
    {
        task++;
        if (task >= MAX_TASKS)
            task = 0;
        ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
    }
    return task;
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
            tcb[i].sp = &stack[i][512];
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
}

// REQUIRED: modify this function to destroy a thread
// REQUIRED: remove any pending semaphore waiting
void destroyThread(_fn fn)
{
}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
}

struct semaphore* createSemaphore(uint8_t count)
{
    struct semaphore *pSemaphore = 0;
    if (semaphoreCount < MAX_SEMAPHORES)
    {
        pSemaphore = &semaphores[semaphoreCount++];
        pSemaphore->count = count;
    }
    return pSemaphore;
}

extern void setSp(uint32_t sp);
extern void setPsp(uint32_t *);
extern void turnPspOn();
extern void pushReg();
extern void popReg();
extern uint32_t* getPsp();
extern uint32_t getSvcNo();
extern uint32_t getR0();



// REQUIRED: modify this function to start the operating system, using all created tasks
void startRtos()
{
    turnPspOn();
    taskCurrent = rtosScheduler();
    setPsp(tcb[taskCurrent].sp);
    _fn fn = (_fn)tcb[taskCurrent].pid;
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
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    pushReg();
    tcb[taskCurrent].sp = getPsp();
    taskCurrent = rtosScheduler();

    if (tcb[taskCurrent].state == STATE_READY)
       {
        setPsp(tcb[taskCurrent].sp);
        popReg();
       }
    else
    {
        uint32_t *p = getPsp();
        p=p-1;
        *p = (1<<24);
        p=p-1;
        *p = (uint32_t)tcb[taskCurrent].pid;
        p = p-6;
        tcb[taskCurrent].state = STATE_READY;
        setPsp(p);

    }



}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{

    uint8_t N = (uint8_t)getSvcNo();
    //uint32_t* s = (uint32_t*)getR0();
    uint32_t* R0 = (uint32_t*)getR0();
    s = (struct semaphore *)*R0;
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

        if(s->count>0)
        {
            s->count--;
        }
        else
        {
            s->processQueue[semaphoreCount] = taskCurrent;
            tcb[taskCurrent].state = STATE_BLOCKED;
            semaphoreCount++;
//            set pointer to semaphore
        }
        NVIC_INT_CTRL_R = NVIC_INT_CTRL_PEND_SV;
        break;

    case 4: //post
        s->count++;
        if(semaphoreCount>0)
        {
            tcb[s->processQueue[0]].state = STATE_READY;
            //Remove from queue

            for (i = 0; i < semaphoreCount; i++)
                {
                    s[i] = s[i + 1];
                }

            semaphoreCount--;
            s->count--;

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
        GPIO_PORTD_DEN_R |= PB4_MASK | PB5_MASK; // enable push buttons
        GPIO_PORTD_PUR_R |= PB4_MASK | PB5_MASK; // enable internal pull-up for push button


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
        TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          // configure for periodic mode (count down)
        TIMER1_TAILR_R = 40000;                       // set load value to 40e3 for 1 KHz interrupt rate
        TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
        NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             // turn-on interrupt 37 (TIMER1A)
        TIMER1_CTL_R |= TIMER_CTL_TAEN;                  // turn-on timer

}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    return 0;
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
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
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

// REQUIRED: add processing for the shell commands through the UART here
//           your solution should not use C library calls for strings, as the stack will be too large
void shell()
{
    while (true)
    {
        if ( (UART0_FR_R & UART_FR_RXFE)==0 ) // if fifo is non-empty get the string
        {
            getString(str, MAX_CHARS);
        }
//        if ((UART0_FR_R & UART_FR_RXFE)==0)  // if fifo is non-empty print the string
//        {
//            putsUart0(str);
//        }
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

    putcUart0(0x0a); putcUart0(0x0d);
    putsUart0("Welcome to the shell interface");
    putcUart0(0x0a); putcUart0(0x0d);
    putsUart0("Please enter the command");
    putcUart0(0x0a); putcUart0(0x0d);
    putsUart0(">> ");

    // Power-up flash
    //GPIO_PORTE_DATA_R |= 1;
    GREEN_LED = 1;
    waitMicrosecond(250000);
    GREEN_LED = 0;
    waitMicrosecond(250000);

    // Initialize semaphores
    keyPressed = createSemaphore(1);
    keyReleased = createSemaphore(0);
    flashReq = createSemaphore(5);
    resource = createSemaphore(1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 15, 1024);
//    ok &=  createThread(idle2, "Idle2", 15, 1024);

    // Add other processes

//    ok &= createThread(lengthyFn, "LengthyFn", 12, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 8, 1024);
    ok &= createThread(oneshot, "OneShot", 4, 1024);
//    ok &= createThread(readKeys, "ReadKeys", 12, 1024);
//    ok &= createThread(debounce, "Debounce", 12, 1024);
//    ok &= createThread(important, "Important", 0, 1024);
//    ok &= createThread(uncooperative, "Uncoop", 10, 1024);
    ok &= createThread(shell, "Shell", 8, 1024);


    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        RED_LED = 1;

    return 0;
}
