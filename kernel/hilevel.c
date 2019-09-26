/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of
 * which can be found via http://creativecommons.org (and should be included as
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"


/* Since we *know* there will be 2 processes, stemming from the 2 user
 * programs, we can
 *
 * - allocate a fixed-size process table (of PCBs), and then maintain
 *   an index into it for the currently executing process,
 * - employ a fixed-case of round-robin scheduling: no more processes
 *   can be created, and neither is able to terminate.
 */

pcb_t pcb[100];
pipe_t pipes[100];
int activeProcesses = 1;
int executing = 0;

void updatePriorities() {
  for (int i = 0; i < activeProcesses; i++) {
    if (pcb[i].status == STATUS_READY) pcb[i].priority++;
    else if (pcb[i].status == STATUS_EXECUTING) pcb[i].priority = 0;
  }
}

void priorityScheduler( ctx_t* ctx ) {
  updatePriorities();
  pcb_t next = pcb[executing];
  int nextIndex = executing;
  for (int i = 0; i < activeProcesses; i++) {
    if (pcb[i].priority >= next.priority) {
      next = pcb[i];
      nextIndex = i;
    }
  }
  if (nextIndex == executing) return;
  else {
    memcpy( &pcb[executing].ctx, ctx, sizeof( ctx_t ) );
    pcb[executing].status = STATUS_READY;
    memcpy( ctx, &pcb[nextIndex].ctx, sizeof( ctx_t ) );
    pcb[nextIndex].status = STATUS_EXECUTING;
    executing = nextIndex;
  }
  return;
}

extern void main_console();
extern uint32_t tos_process;

void hilevel_handler_rst( ctx_t* ctx ) {
  /* Configure the mechanism for interrupt handling by
   *
   * - configuring timer st. it raises a (periodic) interrupt for each
   *   timer tick,
   * - configuring GIC st. the selected interrupts are forwarded to the
   *   processor via the IRQ interrupt signal, then
   * - enabling IRQ interrupts.
   */

  TIMER0->Timer1Load  = 0x00100000; // select period = 2^20 ticks ~= 1 sec
  TIMER0->Timer1Ctrl  = 0x00000002; // select 32-bit   timer
  TIMER0->Timer1Ctrl |= 0x00000040; // select periodic timer
  TIMER0->Timer1Ctrl |= 0x00000020; // enable          timer interrupt
  TIMER0->Timer1Ctrl |= 0x00000080; // enable          timer

  GICC0->PMR          = 0x000000F0; // unmask all            interrupts
  GICD0->ISENABLER1  |= 0x00000010; // enable timer          interrupt
  GICC0->CTLR         = 0x00000001; // enable GIC interface
  GICD0->CTLR         = 0x00000001; // enable GIC distributor

  int_enable_irq();

  /* Initialise PCBs representing processes stemming from execution of
   * the two user programs.  Note in each case that
   *
   * - the CPSR value of 0x50 means the processor is switched into USR
   *   mode, with IRQ interrupts enabled, and
   * - the PC and SP values matche the entry point and top of stack.
   */

  memset( &pcb[0], 0, sizeof(pcb_t));
  pcb[0].pid = 0;
  pcb[0].status = STATUS_READY;
  pcb[ 0 ].ctx.cpsr = 0x50;
  pcb[ 0 ].ctx.pc   = ( uint32_t )( &main_console );
  pcb[ 0 ].ctx.sp   = ( uint32_t )( &tos_process  );
  pcb[ 0 ].priority = 1;

  /* Once the PCBs are initialised, we (arbitrarily) select one to be
   * restored (i.e., executed) when the function then returns.
   */

  memcpy( ctx, &pcb[ 0 ].ctx, sizeof( ctx_t ) );
  pcb[ 0 ].status = STATUS_EXECUTING;
  executing = 0;

  return;
}

void hilevel_handler_irq( ctx_t* ctx ) {
  // Step 2: read  the interrupt identifier so we know the source.

  uint32_t id = GICC0->IAR;

  // Step 4: handle the interrupt, then clear (or reset) the source.

  if( id == GIC_SOURCE_TIMER0 ) {
    priorityScheduler( ctx );
    TIMER0->Timer1IntClr = 0x01;
  }

  // Step 5: write the interrupt identifier to signal we're done.

  GICC0->EOIR = id;

  return;
}

void hilevel_handler_svc( ctx_t* ctx, uint32_t id ) {
  /* Based on the identified encoded as an immediate operand in the
   * instruction,
   *
   * - read  the arguments from preserved usr mode registers,
   * - perform whatever is appropriate for this system call,
   * - write any return value back to preserved usr mode registers.
   */


  switch( id ) {
    case 0x00 : { // 0x00 => yield()
      priorityScheduler( ctx );
      break;
    }

    case 0x01 : { // 0x01 => write( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      if (fd == 1) { //write to emulation terminal
        for( int i = 0; i < n; i++ ) {
          PL011_putc( UART0, *x++, true );
        }
      }
      else if (fd >= 3) { //write to a pipe
        pipes[fd-3].data = x;
      }
      ctx->gpr[ 0 ] = n;
      break;
    }

    case 0x02 : { // 0x02 => read( fd, x, n )
      int   fd = ( int   )( ctx->gpr[ 0 ] );
      char*  x = ( char* )( ctx->gpr[ 1 ] );
      int    n = ( int   )( ctx->gpr[ 2 ] );

      if (fd >= 3) { //read from a pipe
        x = pipes[fd-3].data;
      }
      ctx->gpr[ 0 ] = (uint32_t)(x);
      break;
    }



    case 0x03 : { // 0x03 => fork()
      int parentPID = executing;
      int newPID = activeProcesses;
      memset( &pcb[newPID], 0, sizeof(pcb_t));
      //Give new process a unique PID.
      pcb[newPID].pid = newPID;
      pcb[newPID].parent = parentPID;
      //Copy priority and state from parent process.
      pcb[newPID].priority = pcb[parentPID].priority;
      memcpy( &pcb[newPID].ctx, ctx, sizeof( ctx_t ) );
      pcb[newPID].ctx.pc = ctx->pc;
      //Allocate space in memory.
      pcb[newPID].ctx.sp = (uint32_t)(&(tos_process)+1000*newPID);
      pcb[newPID].status = STATUS_READY;
      activeProcesses++;
      //Return 0 for child, PID of child for parent.
      pcb[newPID].ctx.gpr[0] = 0;
      ctx->gpr[0] = newPID;
      break;
    }

    case 0x04 : { //0x04 => exit(x)
      int x = (int)(ctx->gpr[0]);

      int pid = executing;
      int parent = pcb[executing].parent;
      // erase process from pcb
      if (x == 0) {
        memset( &pcb[pid], 0, sizeof(pcb_t));
        for (int i = pid; i < activeProcesses; i++) {
          pcb[i] = pcb[i+1];
          pcb[i].pid--;
        }
        activeProcesses--;
        // return to execution of parent process
        memcpy( ctx, &pcb[parent].ctx, sizeof( ctx_t ) );
        pcb[parent].status = STATUS_EXECUTING;
        executing = parent;
      }
      break;
    }

    case 0x05 : { //0x05 => exec(x)
      void*  x = ( void* )( ctx->gpr[ 0 ] );
      if (x == NULL) break;
      ctx->pc = (uint32_t)(x);
      break;
    }

    case 0x06 : { //0x06 => kill(pid, x)
      int  pid = (int)( ctx->gpr[ 0 ] );
      int  x  = (int)( ctx->gpr[ 1 ] );
      memset( &pcb[pid], 0, sizeof(pcb_t));
      for (int i = pid; i < activeProcesses; i++) {
        pcb[i] = pcb[i+1];
        pcb[i].pid--;
      }
      activeProcesses--;
      break;
    }

    case 0x08 : { //0x08 => mkfifo(pipeID, pidA, pidB)
      int  pipeID   = (int)( ctx->gpr[ 0 ] );
      int  pidA = (int)( ctx->gpr[ 1 ] );
      int  pidB = (int)( ctx->gpr[ 2 ] );
      for (int i = 0; i < 100; i++) {
        if (!pipes[i].inUse) {
          pipes[i].pidA = pidA;
          pipes[i].pidB = pidB;
          pipes[i].inUse = true;
          pipes[i].endA = false;
          pipes[i].endB = false;
          ctx->gpr[0] = 0;
          break;
        }
      }
      ctx->gpr[0] = -1;
      break;
    }

    case 0x09 : { //0x09 => open(pipeID, pid)
      int  pipeID = (int)( ctx->gpr[ 0 ] );
      int  pid = (int)( ctx->gpr[ 1 ] );
      if (pid == pipes[pipeID].pidA) pipes[pipeID].endA = true;
      else if (pid == pipes[pipeID].pidB) pipes[pipeID].endB = true;
      ctx->gpr[0] = pipeID+3;
      break;
    }

    default   : { // 0x?? => unknown/unsupported
      break;
    }
  }

  return;
}
