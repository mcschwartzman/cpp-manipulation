#include "PID_Bowler.h"
//#include <mbed.h>
PIDBowler::PIDBowler(){
  //printf("\nConstructor PIDBowler ");
  state.vel.enabled = false;
  currentEvent.type = NO_LIMIT;
  state.config.Enabled = false;
  state.config.Async = 0;
  state.config.IndexLatchValue = 0;
  state.config.stopOnIndex = 0;
  state.config.useIndexLatch = 0;
  state.config.K.P = .1;
  state.config.K.I = 0;
  state.config.K.D = 0;
  state.config.V.P = .1;
  state.config.V.D = 0;
  state.config.Polarity = 1;
  state.config.stop = 0;
  state.config.upperHistoresis = 0;
  state.config.lowerHistoresis = 0;
  state.config.outputMaximum=255;
  state.config.outputMinimum=0;
  state.config.outputIncrement=1;
  state.config.offset = 0.0;
  state.config.calibrationState = CALIBRARTION_Uncalibrated;
  state.config.tipsScale=1.0;
  state.interpolate.set=0;
  state.interpolate.setTime=0;
  state.interpolate.start=0;
  state.interpolate.startTime=0;
  state.integralSize = 20.0;
  //printf("\nInterpolation check ");
  state.interpolate.go(0);
  SetPIDEnabled( false);
  //printf(" done");
}
/**
 * First step set the gains for the PID controller
 */
void PIDBowler::setPIDConstants( float p, float i, float d) {
    state.config.K.P = p;
    state.config.K.I = i;
    state.config.K.D = d;
}
/**
 * Next initialize the controller object
 */
void  PIDBowler::InitilizePidController() {
      state.config.calibrationState = CALIBRARTION_DONE;
      pidReset( state.CurrentState);
      OnPidConfigure();
}
/**
 * Third update the position
 */
void PIDBowler::updatePosition(){
  state.CurrentState = getPosition() - state.config.offset;
}
/**
 * finally set the position to the output
 */
void PIDBowler::updateControl(){
  if (state.config.Enabled == true) {
      state.SetPoint = state.interpolate.go(getMs());
      MathCalculationPosition(getMs());
      if (GetPIDCalibrateionState() == CALIBRARTION_DONE ||
          GetPIDCalibrateionState() == CALIBRARTION_Uncalibrated) {
          setOutput(state.Output);
      } else if (GetPIDCalibrateionState() == CALIBRARTION_hysteresis) {
          pidHysterisis();
      } else if ((GetPIDCalibrateionState() == CALIBRARTION_home_down) ||
              (GetPIDCalibrateionState() == CALIBRARTION_home_up) ||
              (GetPIDCalibrateionState() == CALIBRARTION_home_velocity)) {
          checkLinkHomingStatus();
      }
  }else if(state.vel.enabled==true){
    MathCalculationVelocity(getMs());
    state.Output=state.vel.currentOutputVel;
    if(state.calibration.state<=CALIBRARTION_DONE)
        setOutput(state.Output);
  }

  if(checkPIDLimitEvents()->type!=NO_LIMIT){

    switch(checkPIDLimitEvents()->type){
      // Error cases
      case LOWERLIMIT:
      case UPPERLIMIT:
      case OVERCURRENT:
      case CONTROLLER_ERROR:
        setOutput(0);// send stop value to motor
        // if limit occurs, shut down controller
        state.config.Enabled =false;
        break;
      case INDEXEVENT:
      case HOME_EVENT:
        break;
    }

  }
  if(checkPIDLimitEvents()->type!=NO_LIMIT){

    switch(checkPIDLimitEvents()->type){
      // Error cases
      case LOWERLIMIT:
      case UPPERLIMIT:
      case OVERCURRENT:
      case CONTROLLER_ERROR:
        setOutput(0);// send stop value to motor
        // if limit occurs, shut down controller
        state.config.Enabled =false;
        break;
      case INDEXEVENT:
      case HOME_EVENT:
        break;
    }

  }
}

void PIDBowler::InitAbsPID( float KP, float KI, float KD, float time) {
    InitAbsPIDWithPosition( KP, KI, KD, time, 0);
}

float PIDBowler::runPdVelocityFromPointer(float currentState,float KP, float KD){

    float currentTime = getMs();
		float timeMsDiff =  (currentTime -state.vel.lastTime);
		float timeDiff =  timeMsDiff/1000;
		float posDiff=currentState -state.vel.lastPosition;
		float currentVelocity = posDiff/timeDiff;
		//float velocityDiff = currentVelocity-state.vel.lastVelocity;
		float velocityDiff=0;
		float proportional =  currentVelocity-state.vel.unitsPerSeCond;
		float set = (proportional*KP)+(velocityDiff*KD)*timeMsDiff;
		state.vel.currentOutputVel-=(set);

		if (state.vel.currentOutputVel>state.config.outputMaximum){
			state.vel.currentOutputVel=state.config.outputMaximum;
                }else if(state.vel.currentOutputVel<state.config.outputMinimum){
			state.vel.currentOutputVel=state.config.outputMinimum;
                }

		// println_I("\t Velocity: set=   ");p_fl_I(state.vel.unitsPerSeCond );print_I(" ticks/seCond" );
    //             println_I("\t current state=   ");p_fl_I(currentState );print_I(" ticks" );
    //             println_I("\t last state=      ");p_fl_I(state.vel.lastPosition );print_I(" ticks" );
		// println_I("\t position diff=   ");p_fl_I(posDiff );print_I(" ticks" );
		// println_I("\t MS diff=         ");p_fl_I(timeMsDiff );
		// println_I("\t current=         ");p_fl_I(currentVelocity );print_I(" ticks/seCond" );
		// println_I("\t Velocity offset= ");p_fl_I(set );
		// println_I("\t Velocity set=    ");p_fl_I(state.vel.currentOutputVel );

		//cleanup
		state.vel.lastPosition=currentState;
		state.vel.lastVelocity=currentVelocity;
		state.vel.lastTime=currentTime;
    return state.vel.currentOutputVel;
}
/**
 * RunAbstractPIDCalc
 * @param state A pointer to the AbsPID struct to run the calculations on
 * @param CurrentTime a float of the time it is called in MS for use by the PID calculation
 */
void PIDBowler::RunAbstractPIDCalc( float CurrentTime) {
    float error;
    float derivative;

    float timeMsDiff =  (CurrentTime -state.vel.lastTime);
	float timeDiff =  timeMsDiff/1000;
	float posDiff=state.CurrentState -state.vel.lastPosition;
	float currentVelocity = posDiff/timeDiff;

    //calculate set error
    error = state.SetPoint - state.CurrentState;

    //remove the value that is INTEGRALSIZE cycles old from the integral calculation to avoid overflow
    //state.integralTotal -= state.IntegralCircularBuffer[state.integralCircularBufferIndex];
    //add the latest value to the integral
    state.integralTotal = (error * (1.0 / state.integralSize)) +
            (state.integralTotal * ((state.integralSize - 1.0) / state.integralSize));

    //This section clears the integral buffer when the zero is crossed
    if ((state.PreviousError >= 0 && error < 0) ||
            (state.PreviousError < 0 && error >= 0)) {
        state.integralTotal = 0;
    }


    //calculate the derivative
    derivative = (error - state.PreviousError); // / ((CurrentTime-state.PreviousTime));
    state.PreviousError = error;

    //do the PID calculation
    state.Output = ((state.config.K.P * error) +
            (state.config.K.D * derivative)
            +(state.config.K.I * state.integralTotal)
            );

    if (state.config.Polarity == false)
        state.Output *= -1.0;
    //Store the current time for next iterations previous time
    state.PreviousTime = CurrentTime;
	state.vel.lastPosition=state.CurrentState;
	state.vel.lastVelocity=currentVelocity;
	state.vel.lastTime=CurrentTime;

}
float PIDBowler::getVelocity(){
	return state.vel.lastVelocity;
}

void PIDBowler::RunPDVel(){
	//println_I("Running PID vel");
	if(state.vel.enabled==true) {
		state.Output=runPdVelocityFromPointer(
                        state.CurrentState,
                        state.config.V.P,
                        state.config.V.D
                        );
    printf("Running velocity\n\n");
    if(state.calibration.state<=CALIBRARTION_DONE)
        setOutput(state.Output);

	}
}

void PIDBowler::StartPDVel(float unitsPerSeCond,float ms){

        if(ms<.1){
            //println_I("Starting Velocity");
            state.vel.enabled=true;
            state.config.Enabled=false;
            state.vel.lastPosition=GetPIDPosition();
            state.vel.lastTime=getMs();
            state.vel.unitsPerSeCond=unitsPerSeCond;
            state.vel.currentOutputVel =0;
        }else{
            //println_I("Starting Velocity Timed");
            float seConds = ms/1000;
            float dist = (float) unitsPerSeCond*(float) seConds;
            float delt = ((float) (GetPIDPosition())-dist);
            SetPIDTimed( delt, ms);
        }


}

void PIDBowler::OnPidConfigure() {
    onPidConfigureLocal();
}

PD_VEL * PIDBowler::getPidVelocityDataTable() {
    return &state.vel;
}

AbsPID * PIDBowler::getPidGroupDataTable() {

    // Internal reference stores the address of the base of the array
    // Add to that the size of the struct times the index. THis should create
    // a pointer to the address of this specific array address
    return &state;
}

bool PIDBowler::isPidEnabled() {
    return state.config.Enabled;
}

void PIDBowler::SetPIDEnabled( bool enabled) {
    state.config.Enabled = enabled;
}


void PIDBowler::SetPIDCalibrateionState(PidCalibrationType incoming) {
    state.config.calibrationState = incoming;
    OnPidConfigure();
}

PidCalibrationType PIDBowler::GetPIDCalibrateionState() {

    return state.config.calibrationState;
}

bool PIDBowler::ZeroPID() {
    //b_println("Resetting PID channel from zeroPID:",INFO_PRINT);
    pidReset( 0);
    return true;
}

bool PIDBowler::ClearPID() {
    state.config.Enabled = false;
    return true;
}

bool PIDBowler::SetPIDTimedPointer( float val, float current, float ms) {
    if (ms < .01)
        ms = 0;
    //local_groups[chan].config.Enabled=true;
    state.interpolate.set = val;
    state.interpolate.setTime = ms;
    state.interpolate.start = current;
    state.interpolate.startTime = getMs();
    state.SetPoint = val;
    //conf->config.Enabled=true;
    InitAbsPIDWithPosition( state.config.K.P,
                            state.config.K.I,
                            state.config.K.D,
                            getMs(),
                            current);
    return true;
}

bool PIDBowler::SetPIDTimed(float val, float ms) {
    state.vel.enabled = false;
    return SetPIDTimedPointer( val, GetPIDPosition(), ms);
}

bool PIDBowler::SetPID( float val) {
    SetPIDTimed( val, 0);
    return true;
}

float PIDBowler::GetPIDPosition() {
    //state.CurrentState=(int)getPosition(chan);
    return state.CurrentState;
}

float PIDBowler::pidResetNoStop( float val) {
    //float value = (float)resetPosition(chan,val);
    float current = state.CurrentState;
    float raw = current + state.config.offset;
    float value = (float) val;
    state.config.offset = (raw - value);
    state.CurrentState = raw - state.config.offset;
//    //println_E("From pidReset Current State: ");
//    p_fl_E(current);
//    print_E(" Target value: ");
//    p_fl_E(value);
//    print_E(" Offset: ");
//    p_int_E(state.config.offset);
//    print_E(" Raw: ");
//    p_int_E(raw);
    float time = getMs();
    state.lastPushedValue = val;
    InitAbsPIDWithPosition( state.config.K.P,
       state.config.K.I, state.config.K.D, time, val);
    state.vel.lastPosition = val;
    state.vel.lastTime = time;
    return val;
}


void PIDBowler::pidReset( float val) {

    float value = pidResetNoStop( val);

    state.interpolate.set = value;
    state.interpolate.setTime = 0;

    state.interpolate.start = value;
    state.interpolate.startTime = getMs();
    state.SetPoint = value;
    bool enabled = state.config.Enabled;
    state.config.Enabled = true; //Ensures output enabled to stop motors
    state.Output = 0.0;
    setOutput( state.Output);
    state.config.Enabled = enabled;
    printf("\nResetting PID");
}

/**
 * InitAbsPID
 * @param state A pointer to the AbsPID the initialize
 * @param KP the Proportional Constant
 * @param KI the Integral Constant
 * @param KD the Derivative Constant
 * @param time the starting time
 */
void PIDBowler::InitAbsPIDWithPosition( float KP, float KI, float KD, float time, float currentPosition) {
    state.config.K.P = KP;
    state.config.K.I = KI;
    state.config.K.D = KD;
    //state.integralCircularBufferIndex = 0;
    state.integralTotal = 0.0;
    state.integralSize = 20.0;
    state.SetPoint = currentPosition;
    state.PreviousError = 0;
    state.Output = 0.0;
    state.PreviousTime = time;

}

bool PIDBowler::isPIDInterpolating() {
    return state.interpolate.setTime != 0;
}

bool PIDBowler::isPIDArrivedAtSetpoint( float plusOrMinus) {
    if (state.config.Enabled)
        return bound(state.SetPoint,
            state.CurrentState,
            plusOrMinus,
            plusOrMinus);
    return true;
}

void PIDBowler::RunPIDControl() {
   updatePosition();
   updateControl();
}

// void RunPIDComs(BowlerPacket *Packet, bool(*pidAsyncCallbackPtr)(BowlerPacket *Packet)) {
//     int i;
//     for (i = 0; i < getNumberOfPidChannels(); i++) {
//         pushPIDLimitEvent(Packet, pidAsyncCallbackPtr, checkPIDLimitEvents(i));
//     }
//     updatePidAsync(Packet, pidAsyncCallbackPtr);
// }
//
// void RunPID(BowlerPacket *Packet, bool(*pidAsyncCallbackPtr)(BowlerPacket *Packet)) {
//     RunPIDControl();
//     RunPIDComs(Packet, pidAsyncCallbackPtr);
// }




void PIDBowler::setOutput( float val) {
    if(bound(0,state.config.tipsScale, .001, .001)){
      //  println_W("PID TPS Sclale close to zero");p_fl_W(state.config.tipsScale);
      state.config.tipsScale=1;
    }

    val *= state.config.tipsScale;
    val += getPidStop();
    if (val > getPidStop() && val < getUpperPidHistoresis() ){
        val = getUpperPidHistoresis();
        ////println_E("Upper histerisys");
    }
    if (val < getPidStop() && val > getLowerPidHistoresis()){
        val = getLowerPidHistoresis();
      //  //println_E("Lower histerisys");
    }

    if(val>state.config.outputMaximum)
     val=state.config.outputMaximum;
    if(val<state.config.outputMinimum)
     val=state.config.outputMinimum;
    state.OutputSet = val;
    //
    setOutputLocal( val);
}


void PIDBowler::incrementHistoresis() {
    state.config.upperHistoresis += state.config.outputIncrement;
    //calcCenter( group);
}

void PIDBowler::decrementHistoresis() {
    state.config.lowerHistoresis -= state.config.outputIncrement;
}

void PIDBowler::calcCenter() {
    float diff = (state.config.upperHistoresis + state.config.lowerHistoresis) / 2;
    state.config.stop = diff;
}

void PIDBowler::checkCalibration() {
    if (state.calibration.calibrated != true) {
        state.config.upperHistoresis = 0;
        state.config.lowerHistoresis = 0;
        state.config.stop = 0;
        state.calibration.calibrated = true;
    }
}

float PIDBowler::getUpperPidHistoresis() {
    checkCalibration();
    return state.config.upperHistoresis;
}

float PIDBowler::getLowerPidHistoresis() {
    checkCalibration();
    return state.config.lowerHistoresis;
}

float PIDBowler::getPidStop() {
    checkCalibration();
    return state.config.stop;
}

void PIDBowler::runPidHysterisisCalibration() {

    if (!state.config.Enabled) {
        ////println_E("Axis disabled for calibration #");
        //p_int_E();
        state.config.Enabled = true;
    }
    state.config.lowerHistoresis = 0;
    state.config.upperHistoresis = 0;
    state.config.stop = 0;
    //    println_I("\tReset PID");
    pidReset( 0); // Zero encoder reading
    //   println_I("\tDisable PID Output");
    SetPIDEnabled( true);
    SetPIDCalibrateionState( CALIBRARTION_hysteresis);

    state.calibration.state =   _CAL_forward;
    printf("\n\tSetting slow move");
    setOutput( -state.config.outputIncrement*3);
    state.timer.setPoint = 2000;
    state.timer.timeBaseIndex = getMs();

}

CAL_STATE PIDBowler::pidHysterisis() {

    if (state.timer.RunEvery(getMs()) > 0) {
      printf("pidHysterisis running...");
      //  //Print_Level l = getPrintLevel();
        //setPrintLevelInfoPrint();
        float boundVal = state.config.outputIncrement*150.0;
        float extr = GetPIDPosition();
        if (bound(0, extr, boundVal, boundVal)) {// check to see if the encoder has moved
            //we have not moved
            //          println_I("NOT moved ");p_fl_I(extr);
            if (state.calibration.state ==   _CAL_forward) {
                incrementHistoresis();
            } else if (state.calibration.state ==   _CAL_backward) {
                decrementHistoresis();
            }
            float  historesisBound= state.config.outputIncrement*25;
            if (state.config.lowerHistoresis < (-historesisBound) &&
                    state.calibration.state ==   _CAL_backward) {
                ////println_E("Backward Motor seems damaged, more then counts of historesis #");
              //  p_int_I();
                state.calibration.state =   _CAL_forward;
            }
            if (state.config.upperHistoresis > (historesisBound) &&
                    state.calibration.state ==   _CAL_forward) {
                ////println_E("Forward Motor seems damaged, more then counts of historesis #");
                //p_int_I();
                state.calibration.state =   _CAL_done;
            }
        } else {
            pidReset( 0);
            setOutput( 0);
            ////println_E("Moved ");
          //  p_fl_E(extr);
            if (state.calibration.state ==   _CAL_forward) {
                //println_I("Backward Calibrated for link# ");
                //p_int_I();
                state.calibration.state =   _CAL_backward;
            } else {
                //println_I("Calibration done for link# ");
                //p_int_I();
                state.calibration.state =   _CAL_done;

                float offset = .9;
                state.config.lowerHistoresis *= offset;
                state.config.upperHistoresis *= offset;
                calcCenter();
            }

        }
        if (state.calibration.state ==   _CAL_forward) {
            setOutput( 1.0f);
        } else if (state.calibration.state ==   _CAL_backward) {
            setOutput( -1.0f);
        }
        ////setPrintLevel(l);
    }
    if (state.calibration.state ==   _CAL_done)
        SetPIDCalibrateionState( CALIBRARTION_DONE);
    return state.calibration.state;
}

void PIDBowler::startHomingLink( PidCalibrationType type, float homedValue) {
    float speed = state.config.outputIncrement*20.0;
    if (type == CALIBRARTION_home_up)
        speed *= 1.0;
    else if (type == CALIBRARTION_home_down)
        speed *= -1.0;
    else {
        //println_E("Invalid homing type");
        return;
    }
    printf("Start Homing link...");
    state.config.tipsScale = 1;
    SetPIDCalibrateionState( type);
    setOutput( speed);
    state.timer.timeBaseIndex = getMs();
    state.timer.setPoint = 1000;
    state.homing.previousValue = GetPIDPosition();
    state.homing.lastTime = getMs();
    state.homing.homedValue = homedValue;
    SetPIDEnabled(true);
}

void PIDBowler::checkLinkHomingStatus() {
    if (!(GetPIDCalibrateionState() == CALIBRARTION_home_down ||
            GetPIDCalibrateionState() == CALIBRARTION_home_up ||
            GetPIDCalibrateionState() == CALIBRARTION_home_velocity
            )
            ) {
        return; //Calibration is not running
    }
    float current = GetPIDPosition();
    float currentTime = getMs();
    if (state.timer.RunEvery(getMs()) > 0) {
        printf("\nCheck Homing ");
        if (GetPIDCalibrateionState() != CALIBRARTION_home_velocity) {
            float boundVal = state.homing.homingStallBound;

            if (bound(state.homing.previousValue,
                    current,
                    boundVal,
                    boundVal
                    )
                    ) {
                pidReset( state.homing.homedValue);
                //after reset the current value will have changed
                current = GetPIDPosition();
                state.config.tipsScale = 1;
                //println_W("Homing Velocity for group ");
                //p_int_W();
                //print_W(", Resetting position to: ");
                //p_fl_W(state.homing.homedValue);
              //  print_W(" current ");
                //p_fl_W(current);

                float speed = -state.config.outputIncrement*20.0;
                if (GetPIDCalibrateionState() == CALIBRARTION_home_up)
                    speed *= 1.0;
                else if (GetPIDCalibrateionState() == CALIBRARTION_home_down)
                    speed *= -1.0;
                else {
                    ////println_E("Invalid homing type");
                    return;
                }
                //Print_Level l = getPrintLevel();
                //setPrintLevelInfoPrint();
                setOutput( speed);
                //setPrintLevel(l);
                state.timer.timeBaseIndex = getMs();
                state.timer.setPoint = 2000;
                SetPIDCalibrateionState( CALIBRARTION_home_velocity);
                state.homing.lastTime = currentTime;
            }
        } else {
            current = GetPIDPosition();
            float posDiff = current - state.homing.previousValue; //ticks
            float timeDiff = (currentTime - state.homing.lastTime) / 1000.0; //
            float tps = (posDiff / timeDiff);
            state.config.tipsScale = state.config.outputIncrement*20 / tps;

            // //println_E("New scale factor: ");
            // p_fl_E(state.config.tipsScale);
            // print_E(" speed ");
            // p_fl_E(tps);
            // print_E(" on ");
            // p_int_E();
            // print_E(" Position difference ");
            // p_fl_E(posDiff);
            // print_E(" time difference ");
            // p_fl_E(timeDiff);


            OnPidConfigure();
            SetPIDCalibrateionState( CALIBRARTION_DONE);
        }
        state.homing.previousValue = current;
    }
}
