#ifndef __ORION_UTILITIES_TIMEOUT_H__
#define __ORION_UTILITIES_TIMEOUT_H__

#include "Orion.h"

#include <stdint.h>

namespace Orion
{
    namespace Utilities
    {
        class Timeout
        {
            public:
                /**
                 * This function will execute a function till it returns
                 * true (success) or untill duration of function was exceeded.
                 * 
                 * @param func The function pointer to execute.
                 * @param ptr The argument to pass to function when executing.
                 * @param duration The milliseconds to wait till function time's out.
                 * 
                 * @return True if success or false if Timed out or false returned from function. 
                 */
                static bool WaitTimeout(bool(*func)(void*), void* ptr, uint32_t duration);
                
                /**
                 * This function will execute a function till it returns
                 * true (success) or untill duration of function was exceeded.
                 * 
                 * @param func The function pointer to execute.
                 * @param duration The milliseconds to wait till function time's out.
                 * 
                 * @return True if success or false if Timed out or false returned from function. 
                 */
                static bool WaitTimeout(bool(*func)(), uint32_t duration);

                /**
                 * This function will execute a function till it returns
                 * or untill duration of function was exceeded.
                 * 
                 * @param func The function pointer to execute.
                 * @param ptr The argument to pass to function when executing.
                 * @param duration The milliseconds to wait till function time's out.
                 */
                static void WaitTimeout(void(*func)(void*), void* ptr, uint32_t duration);

                /**
                 * This function will execute a function till it returns
                 * or untill duration of function was exceeded.
                 * 
                 * @param func The function pointer to execute.
                 * @param duration The milliseconds to wait till function time's out.
                 */
                static void WaitTimeout(void(*func)(), uint32_t duration);

                /**
                 * This function will execute a function till it returns
                 * true (success) or untill duration of function was exceeded.
                 * 
                 * @param func The function pointer to execute.
                 * @param ptr The argument to pass to function when executing.
                 * @param duration The milliseconds to wait till function time's out.
                 * 
                 * @return Return Pointer from function.
                 */
                static void* WaitTimeout(void*(*func)(void*), void* ptr, uint32_t duration);

                /**
                 * This function will execute a function till it returns
                 * non NULL pointer or untill duration of function was exceeded.
                 * 
                 * @param func The function pointer to execute.
                 * @param duration The milliseconds to wait till function time's out.
                 * 
                 * @return Return Pointer from function.
                 */
                static void* WaitTimeout(void*(*func)(), uint32_t duration);
        };
    }
}

#endif//__ORION_UTILITIES_TIMEOUT_H__