#ifndef __UTIL_H__
#define __UTIL_H__

// #include <vector>
// #include <string>

#ifdef __cplusplus
extern "C" {
#endif

bool strequali(const char*p, const char*tag);  ///< case-insensitive string-equals

//float getTime(); ///< In seconds (consistent with ODE
void sleep_ms(unsigned long ms);

string formatted_double(double v, int dp);     ///<  return a string with fixed dp


/* closing bracket for extern "C" */
#ifdef __cplusplus
}
#endif

#endif

