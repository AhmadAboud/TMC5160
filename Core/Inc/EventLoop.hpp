/*
 * EventLoop.hpp
 *
 *  Created on: 05.09.2023
 *      Author: ahmad.alaboud
 */

#ifndef INC_EVENTLOOP_HPP_
#define INC_EVENTLOOP_HPP_
#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdio.h>

// Define func pointers for reading hw timers and timer overflows
typedef uint32_t(*ReadHwTimer)();
typedef uint32_t (*ReadOverflow)();

void EventLoopCpp();   // Cpp function to call into main event loop
void EventCreateCpp();


#ifdef __cplusplus
extern "C"
{
#endif
    void EventLoopC();  // C function to call into Cpp event loop from main
    void EventCreateC();

#ifdef __cplusplus
}
#endif


#endif /* INC_EVENTLOOP_HPP_ */
