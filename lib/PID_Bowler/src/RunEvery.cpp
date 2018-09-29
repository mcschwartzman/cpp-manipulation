#include "RunEvery.h"
RunEveryObject::RunEveryObject(){
  //The start time for the schedule
  timeBaseIndex = 0;
  //The time from the start time to loop over
  setPoint = 1000;
}
RunEveryObject::RunEveryObject(double currentTime, double setpoint){
  //The start time for the schedule
  timeBaseIndex =currentTime;
  //The time from the start time to loop over
  setPoint = setpoint;
}
/**
 * RunEvery
 * This function returns not 0 if it has been at least as long as the "setPoint" field says since the last time it returned not 0.
 * All timeing is handeled internally
 * @param data Pointer to a data storage table
 * @return float of MS after the assigned time that this function is running. A value of 0 means it has not been long enough
 */
float RunEveryObject::RunEvery(float currentTime){
  float diff;
  if(currentTime< timeBaseIndex)
    timeBaseIndex=currentTime;//Check and fix overflow
  diff =(currentTime-timeBaseIndex);
  if (diff > setPoint){
    if(timeBaseIndex+setPoint<currentTime)
      timeBaseIndex = currentTime;
    else
      timeBaseIndex += setPoint;
    return diff-setPoint;
  }
  return 0;
}
