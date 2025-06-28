/////////////////////////////////////////////////////////////////////////////////
// SerialLogFreeRTOS.h
//
// Contains a macro to selectively print information based on a user-defined
// debug level.  Prints will only generate output if the corresponding macro
// evaluates to 'true'.
//
// A supporting class (SerialLogFreeRTOS) is also defined.  This class handles
// FreeRTOS integration by protecting temporary buffer handling and sending
// log data to the print task via a specified queue.
//
// History:
// - 28-JUN-2025 JMC
//   - Original creation from SerialLog2350.h.  Updated for use with FreeRTOS.
// - 21-APR-2025 JMC Original creation from SerialLog.h
//
// Copyright (c) 2025, Joseph M. Corbett
/////////////////////////////////////////////////////////////////////////////////

#if !defined SERIALLOGFREERTOS_H
#define SERIALLOGFREERTOS_H

#include <FreeRTOS.h>       // For FreeRTOS core.
#include <semphr.h>         // For FreeRTOS semaphores.
#include <cstdlib>          // For malloc() and free().
#include <cstdarg>          // For va_??? variable argument handling.
#include <cstdio>           // For vsnprintf().


/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
class SerialLogFreeRTOS
{
public:
    /////////////////////////////////////////////////////////////////////////////
    // Constructor
    //
    // Creates our mutex, queue, and buffer for logging data to the serial port.
    //
    // Arguments:
    //   - bufSize: Size, in bytes of our temporary log buffer.
    //   - q      : Pre-defined queue used to send log dat to the print task.
    /////////////////////////////////////////////////////////////////////////////
    SerialLogFreeRTOS(size_t bufSize, QueueHandle_t q);
    
    /////////////////////////////////////////////////////////////////////////////
    // Destructor
    //
    // Frees our mutex and temporary log buffer.
    /////////////////////////////////////////////////////////////////////////////
    ~SerialLogFreeRTOS();
    
    /////////////////////////////////////////////////////////////////////////////
    // PrintLog()
    //
    // Takes a printf style format string and arguments, converts them to a 
    // C string, then sends the string to the print task for outputting.
    //
    // Arguments:
    //   Standard printf style format string and supporting arguments.
    //   See printf() documentation.
    /////////////////////////////////////////////////////////////////////////////
    static void PrintLog(const char *pFmt, ...);

private:
    // Private non-implemented methods so the user can't call them.
    SerialLogFreeRTOS();
    SerialLogFreeRTOS &operator=(SerialLogFreeRTOS &);
    SerialLogFreeRTOS(const SerialLogFreeRTOS &);

    static SemaphoreHandle_t m_Mutex;   // Mutex used by PrintLog().
    static QueueHandle_t     m_Queue;   // Queue to send data to.
    static size_t            m_BufSize; // Size of log temporary storage  buffer.
    static char             *m_pBuffer; // Log temporary storage buffer.
    static size_t            m_Index;   // String buffer index.
    static size_t            m_QLength; // Max number of entries in queue.
    static size_t            m_MaxMsgLen;// Maximum size in bytes of one message.

}; // End SerialLogFreeRTOS c;ass.


/////////////////////////////////////////////////////////////////////////////////
// LOG_F() macro
//
// Macro to selectively print formatted information based on a user-defined
// debug level.  Prints will only generate output if the corresponding macro
// evaluates to 'true'.
//
// Arguments:
//    - level   : A value that will be evaluated as either 'true' or 'false'.
//                If 'true', then the printf style string will be sent out
//                the serial port.  Otherwise no action will be taken.
//    - args    : This is a printf style string with optional additional
//                arguments.
//
//  Note:
//   - If the 'level argument can be evaluated as 'false' by the compiler at
//     compile time, then the code for the resulting macro will be optimized
//     out by the compiler (i.e. no code will be generated).

/////////////////////////////////////////////////////////////////////////////////
#define LOG_F(level, format, ...)                                   \
    do                                                              \
    {                                                               \
        if(level)                                                   \
        {                                                           \
            SerialLogFreeRTOS::PrintLog(format , ##__VA_ARGS__);    \
        }                                                           \
    } while (0)


#endif // SERIALLOGFREERTOS_H