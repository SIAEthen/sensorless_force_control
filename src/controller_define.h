#define DEBUG_CONTROLLER
#define DEBUG_OBSERVER
#define DEBUG_ROSTOPIC
// #define DEBUG_JOYSTICK
// #define DEBUG_ADMITTANCE

#define USE_LOG
#define USE_CONTROL
#define USE_ADMITTANCE
// #define USE_VARIABLE_ADMITTANCE
#define USE_IROS_FORCE
// choose one dynamic controller to define
#define STSMC
// #define PID

// #define S1
// #define S2
// #define FREE_FLOATING
#define OBSERVER_TEST

//qp allocator damping 
#define QP_ALLOCATOR_055
// #define QP_ALLOCATOR_040
// #define QP_ALLOCATOR_030
// #define QP_ALLOCATOR_020
// #define QP_ALLOCATOR_010
// #define QP_ALLOCATOR_000

#ifdef USE_ADMITTANCE
  #ifdef USE_VARIABLE_ADMITTANCE
  
  #elif defined(USE_IROS_FORCE)
  
  #else
  
  #endif
#endif

// #define USE_TPC
#define USE_HQP