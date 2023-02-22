#ifndef __BASIC_H__
#define __BASIC_H__


/**********************************************************************/
#ifdef __cplusplus
  #define  C_DECLS_BEGIN  extern "C" {
  #define  C_DECLS_END    }
#else
  #define  C_DECLS_BEGIN  /* empty */
  #define  C_DECLS_END    /* empty */
#endif


#define  ATTRIBUTE_USED   __attribute__((used))


/**********************************************************************/
#ifdef STM32F10X_MD
  #include "stm32f10x_conf.h"
#endif

#ifdef STM32F0XX
  #include "stm32f0xx_conf.h"
#endif


/**********************************************************************/
#include <stdbool.h>

#define  HIGH   1
#define  LOW    0

#define  HL16(hi, lo)  ( (((uint16_t)(hi)) <<  8) | (lo) )
#define  HL32(hi, lo)  ( (((uint32_t)(hi)) << 16) | (lo) )
#define  HL64(hi, lo)  ( (((uint64_t)(hi)) << 32) | (lo) )

#define  LH16(lo, hi)  ( (lo) | (((uint16_t)(hi)) <<  8) )
#define  LH32(lo, hi)  ( (lo) | (((uint32_t)(hi)) << 16) )
#define  LH64(lo, hi)  ( (lo) | (((uint64_t)(hi)) << 32) )

#define  L08(x)  ( ((x) >>  0) & 0xFF       )
#define  H08(x)  ( ((x) >>  8) & 0xFF       )
#define  L16(x)  ( ((x) >>  0) & 0xFFFF     )
#define  H16(x)  ( ((x) >> 16) & 0xFFFF     )
#define  L32(x)  ( ((x) >>  0) & 0xFFFFFFFF )
#define  H32(x)  ( ((x) >> 32) & 0xFFFFFFFF )

/**********************************************************************/
// Arduino
#define  min(a, b)  (((a) < (b)) ? (a) : (b))
#define  max(a, b)  (((a) > (b)) ? (a) : (b))
#define  abs(x)     (((x) >  0 ) ? (x) :-(x))
#define  sq(x)      ((x) * (x))

#define  round(x)                  ((long)( (x) + (((x) >= 0) ? +0.5 : -0.5) ))
#define  constrain(x, low, high)   (((x) < (low)) ? (low) : (((x) > (high)) ? (high) : (x)))

#define  bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define  bitSet(value, bit)   ((value) |=  (1UL << (bit)))
#define  bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define  bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : bitClear(value, bit))

#define  lowByte(w)  ((uint8_t) ((w) & 0xff))
#define  highByte(w) ((uint8_t) ((w) >> 8))

/**********************************************************************/
#define  _BV(pos)  (1UL << (pos))

#define  BITMASK_IS_SET(reg, mask)    ((reg) & (mask))
#define  BITMASK_IS_CLR(reg, mask)  (!((reg) & (mask)))
#define  BITMASK_IS_CLEAR  BITMASK_IS_CLR

#define  BITMASK_SET(reg, mask)  ((reg) |=  (mask))
#define  BITMASK_CLR(reg, mask)  ((reg) &= ~(mask))
#define  BITMASK_TGL(reg, mask)  ((reg) ^=  (mask))
#define  BITMASK_CLEAR   BITMASK_CLR
#define  BITMASK_TOGGLE  BITMASK_TGL

#define  BITPOS_SET(reg, pos, cfg, mask)  ((reg) = ((reg) & ~((mask) << (pos))) | (((cfg) & (mask)) << (pos)))

#define  BITPOS_SET_X1(reg, pin, cfg)  BITPOS_SET(reg, (pin)*1, cfg, 0b1   )
#define  BITPOS_SET_X2(reg, pin, cfg)  BITPOS_SET(reg, (pin)*2, cfg, 0b11  )
#define  BITPOS_SET_X3(reg, pin, cfg)  BITPOS_SET(reg, (pin)*3, cfg, 0b111 )
#define  BITPOS_SET_X4(reg, pin, cfg)  BITPOS_SET(reg, (pin)*4, cfg, 0b1111)

/**********************************************************************/
#endif // __BASIC_H__
