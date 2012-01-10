/* Timer example for 68HC11
   Copyright (C) 2000, 2001, 2002, 2003 Free Software Foundation, Inc.
   Written by Stephane Carrez (stcarrez@nerim.fr)	

This file is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation; either version 2, or (at your option) any
later version.

In addition to the permissions in the GNU General Public License, the
Free Software Foundation gives you unlimited permission to link the
compiled version of this file with other programs, and to distribute
those programs without any restriction coming from the use of this
file.  (The General Public License restrictions do apply in other
respects; for example, they cover modification of the file, and
distribution when not linked into another program.)

This file is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; see the file COPYING.  If not, write to
the Free Software Foundation, 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.  */

/*! @page timerExample Display the time using the microcontroller timer and interrupts.

    The \c timer example uses the \b RTI interrupt to update the time
    of the day and display the current time on the serial line.

  @htmlonly
  Source file: <a href="timer_8c-source.html">timer.c</a>
  @endhtmlonly

 */

#include "TimerLED.h"

#ifdef USE_INTERRUPT_TABLE

/* Interrupt table used to connect our timer_interrupt handler.

   Note: the `XXX_handler: foo' notation is a GNU extension which is
   used here to ensure correct association of the handler in the struct.
   This is why the order of handlers declared below does not follow
   the HC11 order.  */
struct interrupt_vectors __attribute__((section(".vectors"))) vectors = 
{
  res0_handler:           fatal_interrupt, /* res0 */
  res1_handler:           fatal_interrupt,
  res2_handler:           fatal_interrupt,
  res3_handler:           fatal_interrupt,
  res4_handler:           fatal_interrupt,
  res5_handler:           fatal_interrupt,
  res6_handler:           fatal_interrupt,
  res7_handler:           fatal_interrupt,
  res8_handler:           fatal_interrupt,
  res9_handler:           fatal_interrupt,
  res10_handler:          fatal_interrupt, /* res 10 */
  sci_handler:            fatal_interrupt, /* sci */
  spi_handler:            fatal_interrupt, /* spi */
  acc_overflow_handler:   fatal_interrupt, /* acc overflow */
  acc_input_handler:      fatal_interrupt,
  timer_overflow_handler: fatal_interrupt,
  output5_handler:        fatal_interrupt, /* out compare 5 */
  output4_handler:        fatal_interrupt, /* out compare 4 */
  output3_handler:        fatal_interrupt, /* out compare 3 */
  output2_handler:        fatal_interrupt, /* out compare 2 */
  output1_handler:        fatal_interrupt, /* out compare 1 */
  capture3_handler:       fatal_interrupt, /* in capt 3 */
  capture2_handler:       fatal_interrupt, /* in capt 2 */
  capture1_handler:       fatal_interrupt, /* in capt 1 */
  irq_handler:            fatal_interrupt, /* IRQ */
  xirq_handler:           fatal_interrupt, /* XIRQ */
  swi_handler:            fatal_interrupt, /* swi */
  illegal_handler:        fatal_interrupt, /* illegal */
  cop_fail_handler:       fatal_interrupt,
  cop_clock_handler:      fatal_interrupt,

  /* What we really need.  */
  rtii_handler:           timer_interrupt,
  reset_handler:          _start
};

#endif

#define TIMER_DIV  (8192L)
#define TIMER_TICK (M6811_CPU_E_CLOCK / TIMER_DIV)

// BIT 6 on PORTA is for the LED
#define LED_BIT (1<<5)

unsigned long timer_count;
unsigned long boot_time;

/* Timer interrupt handler.  */
void __attribute__((interrupt))
timer_interrupt (void)
{
  timer_count++;
  timer_acknowledge ();
}

/* Returns the current number of ticks that ellapsed since we started.  */
static inline unsigned long
timer_get_ticks ()
{
  unsigned long t;

  lock ();
  t = timer_count;
  unlock ();
  return t;
}

/* Translate the number of ticks into some seconds.  */
static unsigned long
timer_seconds (unsigned long ntime)
{
  unsigned long n;

  /* To compute SECS = NTIME * TIMER_DIV / M6811_CPU_E_CLOCK accurately,
     use Bezous relation (A = BQ + R).  */
  n = ntime * (TIMER_DIV / M6811_CPU_E_CLOCK);
  n += (ntime * (TIMER_DIV % M6811_CPU_E_CLOCK)) / M6811_CPU_E_CLOCK;
  n += boot_time;
  return n;
}

/* Translate the number of ticks into some microseconds.  */
static unsigned long
timer_microseconds (unsigned long ntime)
{
  unsigned long n;

  /* To compute SECS = NTIME * TIMER_DIV / M6811_CPU_E_CLOCK accurately,
     use Bezous relation (A = BQ + R).  */
  n = ntime * (TIMER_DIV / 2);
  n += (ntime * (TIMER_DIV % 2)) / 2;
  n = n % 1000000L;
  return n;
}

/* Translate the string pointed to by *p into a number.
   Update *p to point to the end of that number.  */
static unsigned short
get_value (char **p)
{
  char *q;
  unsigned short val;
  
  q = *p;
  while (*q == ' ')
    q++;
  
  val = 0;
  while (1)
    {
      char c = *q++;
      if (c < '0' || c > '9')
        break;
      val = (val * 10) + (c - '0');
    }
  q--;
  *p = q;
  return val;
}

/* Ask for the boot time.  */
static void
get_time ()
{
  char buf[32];
  int pos;
  char c;
  unsigned short hours, mins, secs;
  char *p;
  int error = 0;
  
  print ("\r\nBoot time ? ");
  pos = 0;
  while (1)
    {
      c = serial_recv ();
      if (c == '\r' || c == '\n')
        break;

      if (c == '\b')
        {
          print ("\b \b");
          pos--;
          if (pos < 0)
            pos = 0;
        }
      else if (pos < sizeof (buf) - 1)
        {
          buf[pos] = c;
          buf[pos+1] = 0;
          print (&buf[pos]);
          pos++;
        }
    }

  print ("\n");
  buf[pos] = 0;
  p = buf;
  hours = get_value (&p);
  if (*p++ != ':')
    error = 1;
  mins = get_value (&p);
  if (*p++ != ':' || mins >= 60)
    error = 1;
  secs = get_value (&p);
  if (*p++ != 0 || secs >= 60)
    error = 1;

  if (error == 0)
    {
      boot_time = (hours * 3600) + (mins * 60) + (secs);
      print ("Boot time is set.\r\n");
    }
  else
    {
      print ("Invalid boot time.\r\n");
      print ("Format is: HH:MM:SS\r\n");
    } 
}

/* Display the current time on the serial line.  */
static void
display_time (unsigned long ntime)
{
  unsigned long seconds;
  unsigned short hours, mins;
  unsigned long nus;
  char buf[12];

  static unsigned long last_sec = 0xffffffff;
  static unsigned long last_us = 0;

  /* Translate the number of ticks in seconds and milliseconds.  */
  seconds = timer_seconds (ntime);
  nus = timer_microseconds (ntime);
          
  nus = nus / 100000L;

  /* If the seconds changed, re-display everything.  */
  if (seconds != last_sec)
    {
      last_sec = seconds;
      last_us = nus;
      hours = (unsigned short) (seconds / 3600L);
      mins = (unsigned short) (seconds % 3600L);
      seconds = (unsigned long) (mins % 60);
      mins = mins / 60;
      buf[0] = '0' + (hours / 10);
      buf[1] = '0' + (hours % 10);
      buf[2] = ':';
      buf[3] = '0' + (mins / 10);
      buf[4] = '0' + (mins % 10);
      buf[5] = ':';
      buf[6] = '0' + (seconds / 10);
      buf[7] = '0' + (seconds % 10);
      buf[8] = '.';
      buf[9] = '0' + nus;
      buf[10] = 0;
      serial_print ("\r");
      serial_print (buf);

      // Toggle the LED
      _io_ports[M6811_PORTA] ^= LED_BIT;
    }

  /* Only re-display the tens of a second.  */
  else if (last_us != nus)
    {
      last_us = nus;
      buf[0] = '0' + nus;
      buf[1] = 0;
      serial_print ("\b");
      serial_print (buf);
    }
  serial_flush ();
}

int
main ()
{
  unsigned long prev_time;
  
  serial_init ();
  lock ();
  boot_time = 0;
  timer_count = 0;

  /* Set interrupt handler for bootstrap mode.  */
  set_interrupt_handler (RTI_VECTOR, timer_interrupt);

  /* Initialize the timer.  */
  timer_initialize_rate (M6811_TPR_16);
  prev_time = timer_count;

  unlock ();

  /* Ask for the boot time.  */
  get_time ();

  /* Loop waiting for the time to change and redisplay it.  */
  while (1)
    {
      unsigned long ntime;

      /* Reset the COP (in case it is active).  */
      cop_optional_reset ();

      /* If something is received on the serial line,
         ask for the boot time again.  */
      if (serial_receive_pending ())
        get_time ();

      /* Get current time and see if we must re-display it.  */
      ntime = timer_get_ticks ();
      if (ntime != prev_time)
        {
	  prev_time = ntime;
	  display_time (ntime);
        }
    }
}
