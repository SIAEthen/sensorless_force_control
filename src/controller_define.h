#define DEBUG_CONTROLLER
// #define DEBUG_OBSERVER
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

//HQP has several methods, for example, 
// from joejoe, this is the best hqp, it didnt consider the activate and deactivate tasks
// from korean guys (continuous), it consider the activate the tasks, but it has discontinuty when beta turns to 0 and 1
// because of the slack variables
// from korean guys but never deactivate the tasks. it will always compute the next nominal task, high compute cost

#ifdef USE_HQP
  // #define USE_HQP_JOJO
  #define USE_HQP_CONTINUOUS
  // #define USE_HQP_Korean
#endif