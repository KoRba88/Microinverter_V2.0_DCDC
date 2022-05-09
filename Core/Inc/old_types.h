/*
 * old_types.h
 *
 *  Created on: Aug 7, 2020
 *      Author: user01
 */

#ifndef INC_OLD_TYPES_H_
#define INC_OLD_TYPES_H_


#define s8 int8_t
#define u8 uint8_t
#define s16 int16_t
#define u16 uint16_t
#define vu16 volatile u16
#define s32 int32_t
#define u32 uint32_t
#define vu32 volatile u32

#define U8_MAX     ((u8)255)
#define S8_MAX     ((s8)127)
#define S8_MIN     ((s8)-128)
#define U16_MAX    ((u16)65535u)
#define S16_MAX    ((s16)32767)
#define S16_MIN    ((s16)-32768)
#define U32_MAX    ((u32)4294967295uL)
#define S32_MAX    ((s32)2147483647)
#define S32_MIN    ((s32)-2147483648)
#endif /* INC_OLD_TYPES_H_ */
