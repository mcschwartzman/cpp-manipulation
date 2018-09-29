#include "Interpolate.h"
//#include <stdint.h>
#include <mbed.h>

Interpolate::Interpolate(){

  //Target value for the interpolator to get to
  set=0;
  //Initial starting point value of target
  start=0;
  //How many ms the interpolation should take
  setTime=0;
  //The timestamp of when the interpolation began.
  startTime=0;
}
float Interpolate::go( float currentTime) {
  totalDistance=0;
   elapsed=0;
   currentDistance=0;
    currentLocation=0;


  if(isnan(set)){
    //setPrintLevelErrorPrint();
    // println_E("set NaN");
    return 0;
  }
  if(isnan(start)){
    //setPrintLevelErrorPrint();
    // println_W("start NaN");
    return set;
  }
  if(isnan(setTime)){
  //  setPrintLevelErrorPrint();
    // println_W("setTime NaN");
    return set; // can not divide by zero
  }
  if(isnan(startTime)){
    //setPrintLevelErrorPrint();
    // println_W("startTime NaN");
    return set;
  }
  if(isnan(currentTime)){
  //setPrintLevelErrorPrint();
  // println_W("currentTime NaN");
  return set;
}

//

  // If the time is imediate, then the target should be returned no matter what.
  if(setTime<=0){
    if(setTime<0){
    //   println_E("FAIL, set time is negative");
    // println_E("Time= ");p_fl_E(currentTime);
    // print_W(" Set= ");p_fl_W(set);
    // print_E(" start= ");p_fl_E(start);
    // print_W(" setTime= ");p_fl_W(setTime);
    // print_E(" startTime= ");p_fl_E(startTime);
    //
    // println_W("elapsedTime = ");p_fl_W(elapsed);
    // print_E(" incremental distance = ");p_fl_E(currentDistance);
    // print_W(" Target = ");p_fl_W(currentLocation);
    }
    return set;
  }

  elapsed = currentTime - (startTime);
  //interpolation is done
  if( elapsed < 0){
  //   println_E("FAIL, elapsed time is negative");
  // println_E("Time= ");p_fl_E(currentTime);
  // print_W(" Set= ");p_fl_W(set);
  // print_E(" start= ");p_fl_E(start);
  // print_W(" setTime= ");p_fl_W(setTime);
  // print_E(" startTime= ");p_fl_E(startTime);
  //
  // println_W("elapsedTime = ");p_fl_W(elapsed);
  // print_E(" incremental distance = ");p_fl_E(currentDistance);
  // print_W(" Target = ");p_fl_W(currentLocation);
  setTime=0;
      return set;
  }
  if(elapsed >= setTime ){
    // println_I("Interp Done");
    setTime=0;
      return set;
  }
  if(elapsed<=0){
    return start;
  }

  totalDistance = set-start;

  // elapsed must always be greater than the set time, current distance will be lower then
  // total distance
  currentDistance = totalDistance * (elapsed/setTime);

  // location will be an offset from the start
  currentLocation = currentDistance+start;

  if(isnan(currentLocation)){
  //   println_E("FAIL, setpoint calculated as NaN");
  // println_E("Time= ");p_fl_E(currentTime);
  // print_W(" Set= ");p_fl_W(set);
  // print_E(" start= ");p_fl_E(start);
  // print_W(" setTime= ");p_fl_W(setTime);
  // print_E(" startTime= ");p_fl_E(startTime);
  //
  // println_W("elapsedTime = ");p_fl_W(elapsed);
  // print_E(" incremental distance = ");p_fl_E(currentDistance);
  // print_W(" Target = ");p_fl_W(currentLocation);
  setTime=0;
    return set;

  }
  return currentLocation;
}
bool Interpolate::between(float targetupper, float actual, float targetLower) {
if(targetupper>targetLower ){
  return (actual>targetLower) && (actual<targetupper) ;
}else{
  return (actual<targetLower) && (actual>targetupper) ;
}
}
