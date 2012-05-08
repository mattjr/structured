#ifndef OGF_F2C
#define OGF_F2C

typedef signed char ogf_f2c_int8 ;
typedef short ogf_f2c_int16 ;
typedef int ogf_f2c_int32 ;

typedef unsigned char ogf_f2c_uint8 ;
typedef unsigned short ogf_f2c_uint16 ;
typedef unsigned int ogf_f2c_uint32 ;

typedef char* ogf_f2c_pointer ;

typedef float ogf_f2c_float32 ;
typedef double ogf_f2c_float64 ;

#ifdef WIN32
typedef __int64 ogf_f2c_int64;
typedef unsigned __int64 ogf_f2c_uint64;
#else
typedef long long ogf_f2c_int64 ;
typedef unsigned long long ogf_f2c_uint64 ;
#endif


/********************************************************/

typedef ogf_f2c_int32 integer;
typedef ogf_f2c_uint32 uinteger;
typedef ogf_f2c_int64 longint ;
typedef ogf_f2c_uint64 ulongint ;
typedef ogf_f2c_pointer address;
typedef ogf_f2c_int16 shortint;
typedef ogf_f2c_float32 real;
typedef ogf_f2c_float64 doublereal;
typedef struct { real r, i; } complex;
typedef struct { doublereal r, i; } doublecomplex;
typedef ogf_f2c_int32 logical;
typedef ogf_f2c_int16 shortlogical;
typedef ogf_f2c_int8 logical1;
typedef ogf_f2c_int8 integer1;

#define TRUE_ (1)
#define FALSE_ (0)


#endif
