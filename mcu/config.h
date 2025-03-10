#define DEBUG

#define DOF 6
#ifdef DEBUG
    #define MOVES_CACHE_SIZE 1
#else
    #define MOVES_CACHE_SIZE 10
#endif
#define US_PER_TICK 20