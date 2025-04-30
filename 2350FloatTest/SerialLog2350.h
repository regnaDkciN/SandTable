/////////////////////////////////////////////////////////////////////////////////
// SerialLog2350.h
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
//           - args    : This is a printf style string with optional additional
//                       arguments.
//
//  Notes:
//   - If the 'level argument can be evaluated as 'false' by the compiler at
//     compile time, then the code for the resulting macro will be optimized
//     out by the compiler (i.e. no code will be generated).
//
// History:
// - 21-APR-2025 JMC Original creation from SerialLog.h
//
// Copyright (c) 2025, Joseph M. Corbett
/////////////////////////////////////////////////////////////////////////////////

#if !defined SERIALLOG2350_H
#define SERIALLOG2350_H


#define LOG_F(level, args...)     \
    do                            \
    {                             \
        if(level)                 \
        {                         \
            Serial.printf(args);  \
        }                         \
    } while (0)


#endif // SERIALLOG2350_H