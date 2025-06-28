/////////////////////////////////////////////////////////////////////////////////
// SerialLogFreeRTOS.CPP
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
#include "SerialLogFreeRTOS.h"
#include <HardwareSerial.h>         //%%%jmc only used while debugging.  Remove later.
extern HardwareSerial Serial;       //%%%jmc only used while debugging.  Remove later.

// Initialize our class (SerialLogFreeRTOS) static members.
SemaphoreHandle_t SerialLogFreeRTOS::m_Mutex = NULL;
QueueHandle_t     SerialLogFreeRTOS::m_Queue = NULL;
size_t            SerialLogFreeRTOS::m_BufSize = 0;
char             *SerialLogFreeRTOS::m_pBuffer = NULL;
size_t            SerialLogFreeRTOS::m_Index = 0; 
size_t            SerialLogFreeRTOS::m_QLength = 0; 
size_t            SerialLogFreeRTOS::m_MaxMsgLen = 0;

/////////////////////////////////////////////////////////////////////////////////
// Constructor
//
// Creates our mutex, queue, and buffer for logging data to the serial port.
//
// Arguments:
//   - bufSize: Size, in bytes of our temporary log buffer.
//   - q      : Pre-defined queue used to send log dat to the print task.
/////////////////////////////////////////////////////////////////////////////////
SerialLogFreeRTOS::SerialLogFreeRTOS(size_t bufSize, QueueHandle_t q)
{
  
    // if this is the first time here, then create our mutex and buffer.
    if (m_pBuffer == NULL)
    {
        // Create our mutex and make sure it is clear.
        m_Mutex = xSemaphoreCreateMutex();
        xSemaphoreGive(m_Mutex);
        
        // Save our queue for later use.
        m_Queue = q;
        
        // Create our temporary buffer per user requested size.
        m_BufSize = bufSize;
        m_pBuffer = (char *)malloc(bufSize);
        
        // Reset our buffer index and buffer related members..
        taskENTER_CRITICAL();
        m_Index = 0;
        xQueueReset(q);
        m_QLength = uxQueueSpacesAvailable(q);
        m_MaxMsgLen = bufSize / m_QLength;
        taskEXIT_CRITICAL();
    }
} // End constructor.


/////////////////////////////////////////////////////////////////////////////////
// Destructor
//
// Frees our mutex and temporary log buffer.
/////////////////////////////////////////////////////////////////////////////////
SerialLogFreeRTOS::~SerialLogFreeRTOS()
{
    // Get rid or any resources we allolcated.
    m_BufSize = 0;
    if (m_Mutex)
    {
        vSemaphoreDelete(m_Mutex);
    }
    if (m_pBuffer)
    {
        free(m_pBuffer);
    }
} // End destructor.


/////////////////////////////////////////////////////////////////////////////////
// PrintLog()
//
// Takes a printf style format string and arguments, converts them to a 
// C string, then sends the string to the print task for outputting.
//
// Arguments:
//   Standard printf style format string and supporting arguments.
//   See printf() documentation.
/////////////////////////////////////////////////////////////////////////////////
void SerialLogFreeRTOS::PrintLog(const char *pFmt, ...)
{
    // Only try to send the log if we have already been initialized.
    if (m_Mutex)
    {
        // Set up variable argument handling.
        va_list args;
        va_start(args, pFmt);
        
        // xQueueSend() requires that a pointer to the pointer to the data be 
        // used as an argument.  Here we create a pointer to the buffer which
        // xQueueSend() can use.
        char *pString = &m_pBuffer[m_Index];
        
        // Protect our temporary data setup and sending.
        xSemaphoreTake(m_Mutex, pdMS_TO_TICKS(60000));
        int_fast32_t count = vsnprintf(&m_pBuffer[m_Index], m_MaxMsgLen, pFmt, args);
        if (count > 0)
        {
            xQueueSend(m_Queue, &pString, pdMS_TO_TICKS(500));
            m_Index += count + 1;
            if (m_Index >= m_BufSize - m_MaxMsgLen)
            {
                m_Index = 0;
            }
        }
        xSemaphoreGive(m_Mutex);
        
        // Clean up our variable argument handling.
        va_end(args);
    }
} // End PrintLog().
