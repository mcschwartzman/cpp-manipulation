#ifndef PID_BOWLER_H
#define PID_BOWLER_H
 #include <stdint.h>
 #include "Interpolate.h"
 #include "RunEvery.h"
 #include "mbed.h"
typedef enum _PidLimitType {
	//NO_LIMIT(0x00),
	/** The lowerlimit. */
	//LOWERLIMIT(0x01),

	/** The indexevent. */
	//INDEXEVENT(0x02),

	/** The upperlimit. */
	//UPPERLIMIT(0x04),

	/** The overcurrent. */
	//OVERCURRENT(0x08),
	//CONTROLLER_ERROR(0x10),
	//HOME_EVENT(0x20)
    NO_LIMIT = (0x00),

    LOWERLIMIT = (0x01),

    INDEXEVENT = (0x02),

    UPPERLIMIT = (0x04),

    OVERCURRENT = (0x08),

    CONTROLLER_ERROR = (0x10),
	HOME_EVENT = (0x20)
} PidLimitType;

typedef enum _PidCalibrationType {
    CALIBRARTION_Uncalibrated = (0),
    CALIBRARTION_DONE = (1),
    CALIBRARTION_hysteresis = (2),
    CALIBRARTION_home_up = (3),
    CALIBRARTION_home_down = (4),
    CALIBRARTION_home_velocity = (5)

} PidCalibrationType;

typedef struct  _PidLimitEvent {
    PidLimitType type;
    float time;
    float value;
    float latchTickError;
    //	bool stopOnIndex;
}
PidLimitEvent;

/**
 * These are your Control Constants
 */
typedef enum _CAL_STATE {
    _CAL_forward = 0,
      _CAL_backward = 1,
      _CAL_done = 2
} CAL_STATE;
/**
 * This is the storage struct for all the information needed to run the PID calculation
 * Note that this has no assumptions on the type of inputs or type of outputs
 * It also has no assumptions on the time step it is run over. It stores previous time and
 * will calculate scaling based on that and the current time
 */
typedef struct  _AbsPID_Config {

    bool Enabled;
    bool Polarity;
    float IndexLatchValue;
    bool stopOnIndex;
    bool useIndexLatch;
    bool Async;

    struct {
        float P;
        float I;
        float D;
    }
    K;

    struct  {
        float P;
        float D;
    }
    V;
    float upperHistoresis;
    float lowerHistoresis;
    float stop;
    float outputMaximum;
    float outputMinimum;
    float outputIncrement;
    PidCalibrationType calibrationState;
    float offset;
    float tipsScale;

}
AbsPID_Config;

typedef struct  _PD_VEL {
    bool enabled;
    float unitsPerSeCond;
    float lastPosition;
    float lastVelocity;
    float lastTime;
    float currentOutputVel;
}
PD_VEL;

typedef struct  _AbsPID {

    //unsigned char           channel;
    float SetPoint;
    float CurrentState;
    float PreviousError;
    //unsigned int            integralCircularBufferIndex;
    float integralTotal;
    float integralSize;
    float Output;
    float OutputSet;
    // This must be in MS
    float PreviousTime;
    float lastPushedValue;
    float lastPushedTime;

    struct  {
        bool calibrating;
        bool calibrated;
        CAL_STATE state;
        //RunEveryData timer;
    } calibration;

    struct  {
        //RunEveryData timer;
        float homingStallBound;
        float previousValue;
        float lastTime;
        float homedValue;
    } homing;
    RunEveryObject timer;
    AbsPID_Config config;
    Interpolate interpolate;
    PD_VEL vel;
}
AbsPID;

// typedef struct _DYIO_PID {
//     unsigned char inputMode;
//     unsigned char inputChannel;
//     unsigned char outputMode;
//     unsigned char outputChannel;
//     unsigned char outVal;
//     bool flagValueSync;
// }
// DYIO_PID;



class PIDBowler {
public:
  // Implement in the subclass
  PIDBowler();
  /**
   * @param groups a pointer the the array of PID groups
   * @param the number of PID groups
   * @param getPositionPtr function pointer to the get position function
   * @param setPositionPtr function pointer to the set position function
   * @param resetPositionPtr function pointer to the re-set position function
   * @param pidAsyncCallbackPtr function pointer to push an async value
   */
  virtual float getPosition()=0;
  virtual void setOutputLocal( float)=0;
  virtual float resetPosition(float )=0;
  virtual void onPidConfigureLocal()=0;
  virtual void MathCalculationPosition( float)=0;
  virtual void MathCalculationVelocity( float)=0;
  virtual PidLimitEvent* checkPIDLimitEvents()=0;
  virtual float getMs()=0;

  void MathCalculationVelocityDefault( float currentTime);
  /**
   * RunAbstractPIDCalc
   * @param state A pointer to the AbsPID struct to run the calculations on
   * @param CurrentTime a float of the time it is called in MS for use by the PID calculation
   */
  void RunAbstractPIDCalc(float CurrentTime);

  /**
   * Set the PID constants
   * @param group which group to set
   * @param p constant
   * @param i constant
   * @param d constant
   */
  void setPIDConstants( float p, float i, float d);
  /**
   * InitAbsPID
   * @param state A pointer to the AbsPID the initialize
   * @param KP the Proportional Constant
   * @param KI the Integral Constant
   * @param KD the Derivative Constant
   * @param time the starting time
   */
  void InitAbsPIDWithPosition( float KP, float KI, float KD, float time, float currentPosition);
  void InitAbsPID( float KP, float KI, float KD, float time);
  /**
   * Handle a PID packet.
   * @return True if the packet was processed, False if it was not  PID packet
   */
  //bool ProcessPIDPacket(BowlerPacket * Packet);

  void SetPIDEnabled( bool enabled);

  bool isPidEnabled();
  bool SetPIDTimedPointer(float val, float current,float ms);
  bool SetPIDTimed( float val, float ms);
  bool SetPID(float val);
  float GetPIDPosition();

  bool ZeroPID();
  /**
   * Runs both Control and Coms
   */
  //void RunPID(BowlerPacket *Packet, bool (*pidAsyncCallbackPtr)(BowlerPacket *Packet));
  /**
   * THis function runs the Comunication for the PID controller
   */
//void RunPIDComs(BowlerPacket *Packet, bool (*pidAsyncCallbackPtr)(BowlerPacket *Packet));
  /**
   * This runs the get input/math/set output for the PID controller
   */
  void RunPIDControl();
  void RunPDVel();
  void RunVel();
  float runPdVelocityFromPointer( float currentState,float KP, float KD);
  void StartPDVel( float unitsPerSeCond, float ms);
  //void pushPID(BowlerPacket *Packet, bool (*pidAsyncCallbackPtr)(BowlerPacket *Packet), uint8_t chan, int32_t value, float time);
  //void pushPIDLimitEvent(BowlerPacket *Packet, bool (*pidAsyncCallbackPtr)(BowlerPacket *Packet), PidLimitEvent * event);

  void checkLinkHomingStatus();
  /***
   * This is a getter for the interpolation state
   */
  bool isPIDInterpolating();

  /**
   * This function checks the PID channel to see if it has settled at the setpoint plus or minus a bound
   * @param index
   * @param plusOrMinus
   * @return
   */
  bool isPIDArrivedAtSetpoint( float plusOrMinus);

  //bool processPIDGet(BowlerPacket * Packet);

  //bool processPIDPost(BowlerPacket * Packet);
  //bool processPIDCrit(BowlerPacket * Packet);

  //NAMESPACE_LIST * getBcsPidNamespace();

  AbsPID * getPidGroupDataTable();
  PD_VEL * getPidVelocityDataTable();
  //void pushAllPIDPositions(BowlerPacket *Packet, bool (*pidAsyncCallbackPtr)(BowlerPacket *Packet));

  void SetPIDCalibrateionState( PidCalibrationType state);

  PidCalibrationType GetPIDCalibrateionState();

  float getUpperPidHistoresis();
  float getLowerPidHistoresis();
  float getPidStop();
  void checkCalibration();

  void updatePidAsync();
  void pidReset( float val);
  float pidResetNoStop( float val);
  //void pushAllPIDPositions(BowlerPacket *Packet, bool (*pidAsyncCallbackPtr)(BowlerPacket *Packet));

  CAL_STATE pidHysterisis();
  void startHomingLink( PidCalibrationType type,float homedValue);
  void runPidHysterisisCalibration();

  //bool processRunAutoCal(BowlerPacket * Packet);

  void OnPidConfigure();
  bool ClearPID();
  void setOutput( float val);
  void  InitilizePidController();
  bool bound(float target, float actual, float plus, float minus) {
    return ((actual)<(target + plus) && (actual)>(target - minus));
  }
  PidLimitEvent currentEvent;
  AbsPID  state;

  void updatePosition();
  void updateControl();
  float getVelocity();
private:
  void incrementHistoresis();

  void decrementHistoresis() ;
  void calcCenter();

};

#endif
