/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : topology.h
 Description   : Array topology definition.

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __topology_H
#define __topology_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

#define __N	9					// Number of array modules

// Array modules
#define _mod1	1<<3
#define _mod2	2<<3
#define _mod3	3<<3
#define _mod4	4<<3
#define _mod5	5<<3
#define _mod6	6<<3
#define _mod7	7<<3
#define _mod8	8<<3
#define _mod9	9<<3

// Topology
static uint16_t array[__N ][7] ={
	{_H01R0, _mod2 | P6, 0, _mod4 | P4, 0, _mod7 | P5, 0}, // Module 1
	{_H0BR4, 0, 0, _mod3 | P5, 0, 0, _mod1 | P1},          // Module 2
	{_H1FR5, 0, 0, 0, 0, _mod2 | P3, 0},                   // Module 3
	{_H2BR1, _mod5 | P1, 0, 0, _mod1 | P3, 0, 0},		   // Module 4
	{_H01R0, _mod4 | P1, 0, 0, _mod6 | P3, 0, 0},   	   // Module 5
	{_H3BR2, 0, 0, _mod5 | P4, 0, 0, 0},			       // Module 6
	{_H08R7, 0, _mod8 | P1, 0, 0, _mod1 | P5, 0},	       // Module 7
	{_H0AR9, _mod7 | P2, 0, 0, _mod9 | P6, 0, 0},		   // Module 8
	{_H0BR4, 0, 0, 0, 0, 0, _mod8 | P4},			       // Module 9
};

// Configurations for duplex serial ports
#if ( _module == 1 )
	#define	H01R0	        1
	#define	_P1pol_reversed	1
	#define	_P2pol_normal	1
	#define	_P3pol_reversed	1
	#define	_P4pol_normal	1
	#define	_P5pol_reversed	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 2 )
	#define	H0BR4			1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 3 )
	#define	H1FR5			1
    #define	_P1pol_normal	1
    #define	_P2pol_normal	1
    #define	_P3pol_normal	1
    #define	_P4pol_normal	1
    #define	_P5pol_reversed	1
    #define	_P6pol_normal	1
#endif

#if ( _module == 4 )
	#define	H2BR1			1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 5 )
	#define	H01R0			1
	#define	_P1pol_reversed	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_reversed	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 6 )
	#define	H3BR2			1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 7 )
	#define	H08R7			1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_normal	1
#endif

#if ( _module == 8 )
	#define	H0AR9			1
    #define	_P1pol_reversed	1
    #define	_P2pol_normal	1
    #define	_P3pol_normal	1
    #define	_P4pol_reversed	1
    #define	_P5pol_normal	1
    #define	_P6pol_normal	1
#endif

#if ( _module == 9 )
	#define	H0BR4			1
	#define	_P1pol_normal	1
	#define	_P2pol_normal	1
	#define	_P3pol_normal	1
	#define	_P4pol_normal	1
	#define	_P5pol_normal	1
	#define	_P6pol_reversed	1
#endif

#ifdef __cplusplus
}
#endif
#endif /*__ topology_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
