#ifndef ARGOS_STRAIGHT_GUARD_H
#define ARGOS_STRAIGHT_GUARD_H

#include <stdint.h>

/* Positive error means the left wheel is ahead. Result is in 0.01 RPM. */
static inline int32_t Argos_CruiseTrim(int64_t error_counts)
{
  if (error_counts > 2000) { return 200; }
  if (error_counts < -2000) { return -200; }
  return (int32_t)(error_counts / 10);
}

/* Progress is per wheel in thousandths of its requested encoder distance.
   Low byte: 0=continue, 1=time, 2=reverse, 3=overtravel, 4=mismatch, 5=done.
   Bits 8/9 hold left/right motor. Hysteresis prevents rapid switching. */
static inline uint32_t Argos_StraightDecision(int32_t left, int32_t right,
                                             uint32_t elapsed, uint32_t previous)
{
  if (elapsed > 5000U) { return 1U; }
  if ((left < -30) || (right < -30)) { return 2U; }
  if ((left > 1150) || (right > 1150)) { return 3U; }
  int32_t error = left - right;
  if ((error > 250) || (error < -250)) { return 4U; }
  if ((left >= 950) && (right >= 950)) { return 5U; }
  uint32_t hold = 0U;
  if ((left >= 950) || (error > 40) ||
      (((previous & 256U) != 0U) && (error > 15))) { hold |= 256U; }
  if ((right >= 950) || (error < -40) ||
      (((previous & 512U) != 0U) && (error < -15))) { hold |= 512U; }
  return hold;
}

#endif
