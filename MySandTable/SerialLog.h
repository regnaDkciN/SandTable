/////////////////////////////////////////////////////////////////////////////////
// SerialLog.h
//
// Contains macros to selectively print information based on a user-defined
// debug level.  Prints will only generate output if the corresponding macro is
// evaluates to 'true'.
//
// Macros are named as follows:
//    LOGF : This macro implements a formatted print.  It uses a printf style
//           format to create a string to print.  The limitation of this macro is
//           that it cannot print 32-bit integers or floats.  Its arguments are:
//           - level   : A value that will be evaluated as either 'true' or 'false'.
//                       If 'true', then the printf style string will be sent out
//                       the serial port.  Otherwise no action will be taken.
//           - pBuf    : A pointer to a char buffer big enough to hold the largest
//                       formatted string that will be logged.
//           - bufSize : The size, in bytes, of the buffer pjointed to by pBuf.
//           - args    : This is a printf style string with optional additional
//                       arguments.
//    LOGU : This macro implements an unformatted print.  It simply prints the
//           given single value as long as 'level' evaluates to 'true'.  Its
//           arguments are:
//           - level : A value that will be evaluated as either 'true' or 'false'.
//                     If 'true', then the value will be sent out the serial port.
//                     Otherwise no action will be taken.
//           - v     : This is the value that will printed if 'level' evaluates
//                     to 'true'.  It can be any valid printable data type.
//
//  Notes:
//   - If the 'level argument can be evaluated as 'false' by the compiler at
//     compile time, then the code for the resulting macro will be optimized
//     out by the compiler (i.e. no code will be generated).  This is very
//     important given the Arduino Uno small memory size.
//
// History:
// - 25-FEB-2025 JMC Original creation.
//
// Copyright (c) 2025, Joseph M. Corbett
/////////////////////////////////////////////////////////////////////////////////

#if !defined SERIALLOG_H
#define SERIALLOG_H


#define LOGF(pBuf, bufSize, level, ...)             \
    do                                              \
    {                                               \
        if(level)                                   \
        {                                           \
            snprintf(pBuf, bufSize, __VA_ARGS__);   \
            Serial.print(pBuf);                     \
        }                                           \
    } while (0)



#define LOGU(level, v)              \
    do                              \
    {                               \
        if(level)                   \
        {                           \
            Serial.print(v);        \
        }                           \
    } while (0)




#endif // SERIALLOG_H