#ifndef INTERPOLATE_H
#define INTERPOLATE_H
# define isnan(a) (a!=a)
#define println_E(a) printf("\n");printf(a)
#define print_E(a) printf(a)
#define println_W(a) printf("\n");printf(a)
#define print_W(a) printf(a)
#define println_I(a) printf("\n");printf(a)
#define print_I(a) printf(a)
#define p_fl_I(a) printf("%f",a)
#define p_fl_I(a) printf("%f",a)
#define p_fl_E(a) printf("%f",a)
#define p_fl_E(a) printf("%f",a)
#define p_fl_W(a) printf("%f",a)
#define p_fl_W(a) printf("%f",a)

  class Interpolate {
    public:
    //Target value for the interpolator to get to
  	float set;
  	//Initial starting point value of target
  	float start;
  	//How many ms the interpolation should take
  	float setTime;
  	//The timestamp of when the interpolation began.
  	float startTime;
    Interpolate();
    float go( float currentTime) ;
    bool between(float targetupper, float actual, float targetLower) ;
    float  totalDistance;
    float elapsed;
    float currentDistance;
    float currentLocation;

  };
#endif
