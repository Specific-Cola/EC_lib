#ifndef STRUCT_TYPEDEF_H
#define STRUCT_TYPEDEF_H

#include "main.h"

/* exact-width signed integer types */
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef float fp32;
typedef double fp64;

typedef enum{
    FALSE=0   ,
    TRUE=1
}Bool_t;
typedef enum{
    OK=0,  
    NO=1
}Return_t;
enum{
    ONLINE = 0,
    OFFLINE = 1
};

#define RETURN_ERROR    NO
#define RETURN_SUCCESS  OK
#define bool_t          Bool_t

typedef struct{
    fp32    YawAngle;
    fp32    PitchAngle;
    fp32    RollAngle;
    fp32    YawSpeed;
    fp32    PitchSpeed;
    fp32    RollSpeed;
} EulerSystemMeasure_t;

#endif



