/****************************************************************************
 * apps/testing/sched/readylist/readylist.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/* Measures the sorted insertion that sem_post() performs, with interrupts
 * disabled, when a timer interrupt wakes a task W.
 *
 * The main thread runs at MAIN_PRIO and busy-waits, so it plays the task
 * that the interrupt preempts.  N filler threads are made ready-to-run
 * behind it; they cannot run because the main thread outranks them.  A
 * watchdog (which runs in the timer interrupt) then posts the semaphore W
 * is blocked on, and times that sem_post() with perf_gettime().
 *
 * nxsched_add_prioritized() steps over every ready task whose priority is
 * at or above W's, so the test expects
 *
 *   (MAIN_PRIO >= W ? 1 : 0) + (fillers ready ? fillers at or above W : 0)
 *
 * The test itself must not add other long walks with interrupts disabled,
 * or they would be blamed on the wake-up being measured.  Between wake-ups
 * the main thread sleeps (instead of lowering its priority, which would
 * re-insert it behind every filler), and with -d the fillers get distinct
 * priorities from F up to MAIN_PRIO - 1 and are posted lowest first, so
 * each post steps over at most a couple of entries.
 *
 * Each wake-up is checked twice:
 *
 *   - The count reported by the kernel (g_readylist_visits) must equal the
 *     number of ready tasks at or above W's priority, found by scanning
 *     the whole list just before sem_post().  A mismatch fails the test.
 *     Without CONFIG_SCHED_READYLIST_STATS this check is skipped, so the
 *     timing can be compared against an uninstrumented kernel.
 *   - If that number is not the expected one, some other task (usually
 *     hpwork, woken by the same tick) was also ready.  Such wake-ups are
 *     counted under "other" and left out of the timing.
 *
 * With -u DEV the test also reads DEV, which must be receiving a
 * continuous 0x00..0xff byte stream from an external sender, and counts
 * the bytes that went missing while interrupts were disabled.  The list
 * scan is skipped in this mode because it would itself keep interrupts
 * disabled for longer, so only the kernel's count is checked.  On the
 * ESP32 the UART2 hardware RX FIFO overflow flag is also reported.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <fcntl.h>
#include <inttypes.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/queue.h>
#include <nuttx/sched.h>
#include <nuttx/wdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MAIN_PRIO       200 /* The interrupted task; outranks every filler */
#define READER_PRIO     10  /* Below W, so the walk never steps over it */

/* The reader calls poll() and read(), so it needs more than a filler */

#define READER_STACKSIZE 4096

#define DEFAULT_WPRIO   100
#define DEFAULT_FPRIO   100
#define DEFAULT_ITERS   100

/* Give up on a point after this many wake-ups per requested sample */

#define MAX_TRIES       4

/* ESP32 UART2 raw interrupt status, to see RX FIFO overflows directly */

#if defined(CONFIG_ARCH_CHIP_ESP32) && defined(CONFIG_ESP32_UART2)
#  define UART2_INT_RAW_REG  0x3ff6e004
#  define UART2_INT_CLR_REG  0x3ff6e010
#  define UART_RXFIFO_OVF    (1 << 4)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct filler_s
{
  pthread_t thread;
  sem_t sem;
  int prio;
};

/****************************************************************************
 * External Definitions
 ****************************************************************************/

/* The kernel's ready-to-run list (flat build only) */

extern dq_queue_t g_readytorun;

/****************************************************************************
 * Private Data
 ****************************************************************************/

static sem_t g_wsem;
static struct wdog_s g_wdog;
static int g_wprio;
static bool g_raw;
static bool g_scan = true;
static int g_levels = 1;

static volatile bool g_stop;
static volatile bool g_fired;
static volatile bool g_isr;
static volatile clock_t g_elapsed;
static volatile uint32_t g_ready;
static volatile uint32_t g_visits;

static volatile uint32_t g_rxbytes;
static volatile uint32_t g_lost;
static volatile uint32_t g_gaps;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static FAR void *waiter(FAR void *arg)
{
  while (!g_stop)
    {
      sem_wait(&g_wsem);
    }

  return NULL;
}

static FAR void *filler(FAR void *arg)
{
  FAR sem_t *sem = arg;

  while (!g_stop)
    {
      sem_wait(sem);
    }

  return NULL;
}

/* Checks that each received byte is one more than the last */

static FAR void *reader(FAR void *arg)
{
  struct pollfd pfd;
  uint8_t buf[256];
  uint8_t next = 0;
  bool synced = false;
  ssize_t n;
  ssize_t i;

  pfd.fd     = (int)(intptr_t)arg;
  pfd.events = POLLIN;

  while (!g_stop)
    {
      if (poll(&pfd, 1, 100) <= 0)
        {
          continue;
        }

      n = read(pfd.fd, buf, sizeof(buf));
      for (i = 0; i < n; i++)
        {
          if (synced && buf[i] != next)
            {
              g_lost += (uint8_t)(buf[i] - next);
              g_gaps++;
            }

          next   = buf[i] + 1;
          synced = true;
        }

      if (n > 0)
        {
          g_rxbytes += n;
        }
    }

  return NULL;
}

/* Runs in the timer interrupt */

static void post_from_isr(wdparm_t arg)
{
  FAR struct tcb_s *tcb;
  uint32_t ready = 0;
  clock_t start;

  /* Scan the whole list, without relying on its order */

  for (tcb = g_scan ? (FAR struct tcb_s *)g_readytorun.head : NULL;
       tcb != NULL; tcb = tcb->flink)
    {
      if (tcb->sched_priority >= g_wprio)
        {
          ready++;
        }
    }

  g_ready = ready;
  g_isr = up_interrupt_context();
#ifdef CONFIG_SCHED_READYLIST_STATS
  g_readylist_visits = UINT32_MAX;
#endif

  start = perf_gettime();
  sem_post(&g_wsem);
  g_elapsed = perf_gettime() - start;

#ifdef CONFIG_SCHED_READYLIST_STATS
  g_visits = g_readylist_visits;
#else
  g_visits = ready; /* No kernel count to check; time only */
#endif

  g_fired = true;
}

/* Let every lower priority thread run until it blocks again.  Sleeping
 * only removes the main thread from the ready-to-run list and puts it back
 * at the head, so it adds no long walk of its own.
 */

static void drain(void)
{
  usleep(CONFIG_USEC_PER_TICK);
}

static int start_thread(FAR pthread_t *thread, int prio, size_t stacksize,
                        pthread_startroutine_t entry, FAR void *arg)
{
  struct sched_param param;
  pthread_attr_t attr;
  int ret;

  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, stacksize);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  param.sched_priority = prio;
  pthread_attr_setschedparam(&attr, &param);

  ret = pthread_create(thread, &attr, entry, arg);
  pthread_attr_destroy(&attr);
  return ret;
}

static int compare_clock(FAR const void *a, FAR const void *b)
{
  clock_t x = *(FAR const clock_t *)a;
  clock_t y = *(FAR const clock_t *)b;

  return x < y ? -1 : x > y;
}

/* Convert to ns, or leave in perf counter units (CPU cycles on most
 * targets) with -c
 */

static uint64_t to_units(clock_t elapsed)
{
  struct timespec ts;

  if (g_raw)
    {
      return elapsed;
    }

  perf_convert(elapsed, &ts);
  return (uint64_t)ts.tv_sec * NSEC_PER_SEC + ts.tv_nsec;
}

/* Wake W from the timer interrupt until iters wake-ups have seen exactly
 * the expected ready tasks, and return how many did.  Wake-ups where the
 * kernel's count disagrees with the list, or that did not run in
 * interrupt context, are counted in wrong; those where other tasks were
 * also ready are counted in other.  Wake-ups after which the UART RX FIFO
 * had overflowed are counted in ovf.
 */

static int measure(FAR struct filler_s *fillers, int n, bool blocked,
                   int iters, uint32_t expect, FAR clock_t *samples,
                   FAR int *wrong, FAR int *other, FAR int *ovf)
{
  uint32_t ready;
  int got = 0;
  int tries;
  int lvl;
  int i;

  for (tries = 0; got < iters && tries < MAX_TRIES * iters; tries++)
    {
      /* Lowest priority first, so each post steps over few entries */

      for (lvl = 0; !blocked && lvl < g_levels; lvl++)
        {
          for (i = lvl; i < n; i += g_levels)
            {
              sem_post(&fillers[i].sem);
            }
        }

      g_fired = false;
      wd_start(&g_wdog, 1, post_from_isr, 0);
      while (!g_fired)
        {
        }

      /* Without the scan, only the kernel's count is known */

      ready = g_scan ? g_ready : g_visits;
      if (g_visits != ready || !g_isr)
        {
          (*wrong)++;
        }
      else if (ready != expect)
        {
          (*other)++;
        }
      else
        {
          samples[got++] = g_elapsed;
        }

      drain();

#ifdef UART2_INT_RAW_REG
      if (!g_scan &&
          (*(FAR volatile uint32_t *)UART2_INT_RAW_REG &
           UART_RXFIFO_OVF) != 0)
        {
          *(FAR volatile uint32_t *)UART2_INT_CLR_REG = UART_RXFIFO_OVF;
          (*ovf)++;
        }
#endif
    }

  return got;
}

/* Print the least-squares slope of y against N */

static void print_slope(FAR const char *what, int64_t npoints, int64_t sx,
                        int64_t sxx, int64_t sy, int64_t sxy)
{
  int64_t slope = 10 * (npoints * sxy - sx * sy) /
                  (npoints * sxx - sx * sx);
  int64_t mag = slope < 0 ? -slope : slope;

  printf("readylist: %s cost per added filler: %s%" PRId64 ".%" PRId64
         " %s\n", what, slope < 0 ? "-" : "", mag / 10, mag % 10,
         g_raw ? "ticks" : "ns");
}

static void show_usage(FAR const char *progname)
{
  printf("Usage: %s [-n N] [-s STEP] [-w PRIO] [-f PRIO] [-b] [-d] "
         "[-i ITERS] [-c] [-u DEV]\n", progname);
  printf("  -n N      Filler threads (sweep 0..N when -s is given). "
         "Default 0\n");
  printf("  -s STEP   Sweep step\n");
  printf("  -w PRIO   Priority of the woken task W. Default %d\n",
         DEFAULT_WPRIO);
  printf("  -f PRIO   Priority of the fillers, below %d. Default %d\n",
         MAIN_PRIO, DEFAULT_FPRIO);
  printf("  -b        Leave the fillers blocked instead of ready\n");
  printf("  -d        Spread the fillers over priorities F..%d\n",
         MAIN_PRIO - 1);
  printf("  -i ITERS  Wake-ups per point. Default %d\n", DEFAULT_ITERS);
  printf("  -c        Report perf counter ticks instead of ns\n");
  printf("  -u DEV    Count bytes lost from a 0x00..0xff stream on DEV\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  FAR struct filler_s *fillers;
  FAR clock_t *samples;
  struct sched_param param;
  FAR const char *devpath = NULL;
  pthread_t wthread;
  pthread_t rthread;
  bool blocked = false;
  bool distinct = false;
  int fd = -1;
  int fprio = DEFAULT_FPRIO;
  int iters = DEFAULT_ITERS;
  int nmax = 0;
  int step = 0;
  int created = 0;
  int failed = 0;
  int64_t npoints = 0;
  int64_t sx = 0;
  int64_t sxx = 0;
  int64_t smean = 0;
  int64_t smed = 0;
  int64_t sxmean = 0;
  int64_t sxmed = 0;
  int opt;
  int n;
  int i;

  g_wprio = DEFAULT_WPRIO;
  while ((opt = getopt(argc, argv, "n:s:w:f:bdi:cu:h")) != ERROR)
    {
      switch (opt)
        {
          case 'n':
            nmax = atoi(optarg);
            break;

          case 's':
            step = atoi(optarg);
            break;

          case 'w':
            g_wprio = atoi(optarg);
            break;

          case 'f':
            fprio = atoi(optarg);
            break;

          case 'b':
            blocked = true;
            break;

          case 'd':
            distinct = true;
            break;

          case 'i':
            iters = atoi(optarg);
            break;

          case 'c':
            g_raw = true;
            break;

          case 'u':
            devpath = optarg;
            g_scan  = false;
            break;

          default:
            show_usage(argv[0]);
            return opt == 'h' ? EXIT_SUCCESS : EXIT_FAILURE;
        }
    }

  if (nmax < 0 || step < 0 || iters <= 0 ||
      g_wprio < SCHED_PRIORITY_MIN || g_wprio > SCHED_PRIORITY_MAX ||
      fprio < SCHED_PRIORITY_MIN || fprio >= MAIN_PRIO ||
      (devpath != NULL && g_wprio <= READER_PRIO))
    {
      show_usage(argv[0]);
      return EXIT_FAILURE;
    }

#ifndef CONFIG_SCHED_READYLIST_STATS
  if (devpath != NULL)
    {
      printf("readylist: -u needs CONFIG_SCHED_READYLIST_STATS\n");
      return EXIT_FAILURE;
    }
#endif

  fillers = calloc(nmax > 0 ? nmax : 1, sizeof(*fillers));
  samples = malloc(iters * sizeof(*samples));
  if (fillers == NULL || samples == NULL)
    {
      printf("readylist: out of memory\n");
      free(fillers);
      free(samples);
      return EXIT_FAILURE;
    }

  param.sched_priority = MAIN_PRIO;
  sched_setscheduler(0, SCHED_FIFO, &param);

  g_stop = false;
  sem_init(&g_wsem, 0, 0);
  if (start_thread(&wthread, g_wprio,
                   CONFIG_TESTING_READYLIST_FILLER_STACKSIZE,
                   waiter, NULL) != 0)
    {
      printf("readylist: failed to start W\n");
      free(fillers);
      free(samples);
      return EXIT_FAILURE;
    }

  if (devpath != NULL)
    {
      fd = open(devpath, O_RDONLY);
      if (fd < 0 || start_thread(&rthread, READER_PRIO,
                                 READER_STACKSIZE, reader,
                                 (FAR void *)(intptr_t)fd) != 0)
        {
          printf("readylist: failed to read %s\n", devpath);
          g_stop = true;
          sem_post(&g_wsem);
          pthread_join(wthread, NULL);
          if (fd >= 0)
            {
              close(fd);
            }

          free(fillers);
          free(samples);
          return EXIT_FAILURE;
        }
    }

  if (distinct)
    {
      g_levels = MAIN_PRIO - fprio;
    }

  printf("readylist: W prio %d, fillers prio %d..%d (%s), interrupted "
         "task prio %d, %d wake-ups per point\n",
         g_wprio, fprio, fprio + g_levels - 1,
         blocked ? "blocked" : "ready", MAIN_PRIO, iters);
  if (devpath != NULL)
    {
      printf("readylist: counting lost bytes on %s\n", devpath);
    }

  printf("readylist: times in %s\n",
         g_raw ? "perf counter ticks" : "ns");
  printf("%5s %6s %7s %7s %7s %7s %5s %5s",
         "N", "visits", "min", "med", "mean", "max", "wrong", "other");
  if (devpath != NULL)
    {
      printf(" %7s %5s %5s %4s", "rx", "lost", "gaps", "ovf");
    }

  printf("\n");

  for (n = 0; n <= nmax; n += step > 0 ? step : (nmax > 0 ? nmax : 1))
    {
      uint32_t expect;
      uint32_t above = 0;
      uint64_t mean = 0;
      uint64_t med = 0;
      uint32_t rx;
      uint32_t lost;
      uint32_t gaps;
      int wrong = 0;
      int other = 0;
      int ovf = 0;
      int got;

      for (; created < n; created++)
        {
          sem_init(&fillers[created].sem, 0, 0);
          fillers[created].prio = fprio + created % g_levels;
          if (start_thread(&fillers[created].thread,
                           fillers[created].prio,
                           CONFIG_TESTING_READYLIST_FILLER_STACKSIZE,
                           filler, &fillers[created].sem) != 0)
            {
              printf("readylist: failed to start filler %d\n", created);
              break;
            }
        }

      if (created < n)
        {
          failed++;
          break;
        }

      /* Let W and the new fillers reach their sem_wait() */

      drain();

      for (i = 0; i < n; i++)
        {
          if (fillers[i].prio >= g_wprio)
            {
              above++;
            }
        }

      expect = (MAIN_PRIO >= g_wprio ? 1 : 0) + (blocked ? 0 : above);

      rx   = g_rxbytes;
      lost = g_lost;
      gaps = g_gaps;

      got = measure(fillers, n, blocked, iters, expect, samples,
                    &wrong, &other, &ovf);

      rx   = g_rxbytes - rx;
      lost = g_lost - lost;
      gaps = g_gaps - gaps;

      if (wrong > 0 || got == 0 || (devpath != NULL && rx == 0))
        {
          failed++;
        }

      if (got == 0)
        {
          printf("%5d %6" PRIu32 " %7s %7s %7s %7s %5d %5d",
                 n, expect, "-", "-", "-", "-", wrong, other);
        }
      else
        {
          qsort(samples, got, sizeof(*samples), compare_clock);
          for (i = 0; i < got; i++)
            {
              mean += to_units(samples[i]);
            }

          mean /= got;
          med = to_units(samples[got / 2]);

          printf("%5d %6" PRIu32 " %7" PRIu64 " %7" PRIu64 " %7" PRIu64
                 " %7" PRIu64 " %5d %5d",
                 n, expect, to_units(samples[0]), med, mean,
                 to_units(samples[got - 1]), wrong, other);
        }

      if (devpath != NULL)
        {
          printf(" %7" PRIu32 " %5" PRIu32 " %5" PRIu32 " %4d",
                 rx, lost, gaps, ovf);
        }

      printf("\n");
      if (got == 0)
        {
          continue;
        }

      npoints++;
      sx     += n;
      sxx    += (int64_t)n * n;
      smean  += mean;
      smed   += med;
      sxmean += (int64_t)n * mean;
      sxmed  += (int64_t)n * med;
    }

  if (npoints > 1 && npoints * sxx != sx * sx)
    {
      print_slope("median", npoints, sx, sxx, smed, sxmed);
      print_slope("mean", npoints, sx, sxx, smean, sxmean);
    }

  g_stop = true;
  for (i = 0; i < created; i++)
    {
      sem_post(&fillers[i].sem);
      pthread_join(fillers[i].thread, NULL);
      sem_destroy(&fillers[i].sem);
    }

  sem_post(&g_wsem);
  pthread_join(wthread, NULL);
  sem_destroy(&g_wsem);

  if (devpath != NULL)
    {
      pthread_join(rthread, NULL);
      close(fd);
      if (g_rxbytes == 0)
        {
          printf("readylist: nothing received on %s\n", devpath);
        }
    }

  free(fillers);
  free(samples);

  printf("readylist: %s\n", failed == 0 ? "PASS" : "FAIL");
  return failed == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
