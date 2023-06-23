#ifndef UTILS_H
#define UTILS_H
#include <Arduino.h>

//#define DEBUG

#define DEBUG_PRINTER Serial

#ifdef DEBUG
#define DEBUG_PRINT(...)                                            \
    {   DEBUG_PRINTER.print(__VA_ARGS__);                           \
    }
#define DEBUG_PRINTLN(...)                                          \
    {                                                               \
        DEBUG_PRINTER.println(__VA_ARGS__);                         \
    }
#define DEBUG_PRINTF(...)                                           \
    {                                                               \
        DEBUG_PRINTER.printf(__VA_ARGS__);                          \
    }
#else
#define DEBUG_PRINT(...) \
    {                    \
    }
#define DEBUG_PRINTLN(...) \
    {                      \
    }
#define DEBUG_PRINTF(...) \
    {                     \
    }
#endif

inline bool everyXms(long* timer, long delay){
    if((millis()-*timer)>delay){
        *timer = millis();
        return true;
    }
    return false;
}

#endif