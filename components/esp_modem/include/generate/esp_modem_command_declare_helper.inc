

// Parameters
// * handle different parameters for C++ and C API
// * make parameter unique names, so they could be easily referenced and forwarded
#define _ARG(param, name) param
#define INT_IN(param, name) int  _ARG(param, name)
#define FLOAT_IN(param, name) float  _ARG(param, name)
#ifdef __cplusplus
#include <string>
#define STRING_IN(param, name) const std::string& _ARG(param, name)
#define STRING_OUT(param, name) std::string& _ARG(param, name)
#define BOOL_IN(param, name) const bool  _ARG(param, name)
#define BOOL_OUT(param, name) bool&  _ARG(param, name)
#define INT_OUT(param, name) int&  _ARG(param, name)
#define FLOAT_OUT(param, name) float&  _ARG(param, name)
#define INTEGER_LIST_IN(param, name) const int* _ARG(param, name)

#define STRUCT_OUT(struct_name, p1)  struct_name& p1
#else
#define STRING_IN(param, name) const char* _ARG(param, name)
#define STRING_OUT(param, name) char* _ARG(param, name)
#define BOOL_IN(param, name) const bool _ARG(param, name)
#define BOOL_OUT(param, name) bool* _ARG(param, name)
#define INT_OUT(param, name) int* _ARG(param, name)
#define FLOAT_OUT(param, name) float* _ARG(param, name)
#define INTEGER_LIST_IN(param, name) const int* _ARG(param, name)
#define STRUCT_OUT(struct_name, p1)  esp_modem_ ## struct_name ## _t* p1
#endif
