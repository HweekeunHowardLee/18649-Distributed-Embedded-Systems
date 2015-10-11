/* 
 * 18649 Fall 2014
 * Group 4 - Zhu Zhang (zhuzhang), Hua Liu (hual1), Howard Lee (hweekeul)
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import jSimPack.SimTime.SimTimeUnit;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

//translators for network message
import simulator.elevatorcontrol.DriveSpeedCanPayloadTranslator;
import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.DoorOpenedCanPayloadTranslator;
import simulator.elevatormodules.DoorReversalCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;

//physical state message
import simulator.payloads.DoorMotorPayload;
import simulator.payloads.DoorMotorPayload.WriteableDoorMotorPayload;

/*
 * The DoorControl sends out signal such as mDoorMotor to control the behavior of the doors.
 * For instantiation, the countdown and dwell value must be set
 * 
 * @author Howard Lee
 */
public class DoorControl extends Controller {
    /*****************************************************************************************
     * Declarations
     * **************************************************************************************/
    //note that inputs are Readable objects, while outputs are Writable objects
    
    //local physical state
    private WriteableDoorMotorPayload DoorMotor;

    //network interference
    // translator for the at floor message
    private Utility.AtFloorArray mAtFloor;

    // received drive speed message
    private ReadableCanMailbox networkDriveSpeed;
    // translator for the drive speed message
    private DriveSpeedCanPayloadTranslator mDriveSpeed;

    // received desired floor message
    private ReadableCanMailbox networkDesiredFloor;
    // translator for the desired floor message
    private DesiredFloorCanPayloadTranslator mDesiredFloor;

    // received message value of door closed
    private ReadableCanMailbox networkDoorClosed;
    // translator for the networkDoorClosed;
    private DoorClosedCanPayloadTranslator mDoorClosed;

    // received message value of door opened
    private ReadableCanMailbox networkDoorOpened;
    // translator for the networkDoorOpened;
    private DoorOpenedCanPayloadTranslator mDoorOpened;

    // received message value of door reversal for left and right.
    private ReadableCanMailbox networkDoorReversalLeft;
    private ReadableCanMailbox networkDoorReversalRight;
    // translator for the message of door reversal for left and right.
    private DoorReversalCanPayloadTranslator mDoorReversalLeft;
    private DoorReversalCanPayloadTranslator mDoorReversalRight;

    // translator for the message of car call
    private Utility.CarCallArray mCarCall;

    private Utility.HallCallArray mHallCall;

    //received message value of car weight
    private ReadableCanMailbox networkCarWeight;
    // translator for the message of car weight
    private CarWeightCanPayloadTranslator mCarWeight;

   // send message value of Door Motor from other module.
   private WriteableCanMailbox networkDoorMotor;
   // translator for the message of door motor
   private DoorMotorCommandCanPayloadTranslator mDoorMotor;

   //these variables keep track of which instance this is
   private final Hallway hallway;
   private final SimTime period;
   
   //additional internal state variables
   private SimTime counter = SimTime.ZERO;
   private SimTime dwell= new SimTime(5, SimTimeUnit.SECOND);
   private int nudgeCount;
   private static final int nudgeLimit = 3;

   //enumerate states
   private enum State {
	STATE_DOOR_CLOSED,
	STATE_DOOR_OPENING,
	STATE_DOOR_OPENED,
	STATE_DOOR_CLOSING,
	STATE_OVERWEIGHT,
	STATE_DOOR_NUDGING,
	STATE_DOOR_REVERSAL,
   }
   //state variable initialized to the initial state DOOR_CLOSED
   private State state = State.STATE_DOOR_CLOSED;

   /*
    * The arguments listed in the .cf configuration file should match the order and type given here.
    *
    * For your elevator controllers, you should make sure that the constructor matches the method
    * signatures in ControllerBuilder.makeAll().
    */

   public DoorControl(Hallway hallway, Side side, SimTime period, boolean verbose){
       //call to the controller superclass constructor is required
       super("DoorControl" + ReplicationComputer.makeReplicationString(hallway, side), verbose);

       //stored the constructor arguments in internal state
       this.hallway = hallway;
       this.period = period;
       this.state = State.STATE_DOOR_CLOSED;
       

       /*
        * The log() method is inherited from the Controller class.  It takes an array of objects which
        * will be converted to strings and concatenated only if the log message is actually written.
        *
        * For performance reasons, call with comma- separated lists, e.g.:
        *     log("object=", object);
        * Do NOT call with concatenated objects like:
        *     log("object=" + object);
        *
        */
       log("Created doorcontrol with period = ", period);

      //*****initialize physical state*****
       
       //create a payload object using factory method in DoorMotorPayload.
      DoorMotor = DoorMotorPayload.getWriteablePayload(hallway, side);
      
      // register the payload with the physical interface, periodically sending message.
      physicalInterface.sendTimeTriggered(DoorMotor, period);

      //*****initialize network interface*****
      
      //initialize network interface for AtFloor
      mAtFloor = new Utility.AtFloorArray(canInterface);

      //initialize network interface for HallCall
      mHallCall = new Utility.HallCallArray(canInterface);
      
      //initialize network interface for CarCall
      mCarCall = new Utility.CarCallArray(canInterface);
      
      //initialize network interface for DriveSpeed
      networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
      mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
      canInterface.registerTimeTriggered(networkDriveSpeed);
  
      //initialize network interface for desiredfloor
      networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
      mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
      canInterface.registerTimeTriggered(networkDesiredFloor);
        
      //initialize network interface for door closed
      networkDoorClosed = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway,side));
      mDoorClosed = new DoorClosedCanPayloadTranslator(networkDoorClosed, hallway, side);
      canInterface.registerTimeTriggered(networkDoorClosed);
       
      //initialize network interface for door opened 
      networkDoorOpened = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_OPEN_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway,side));
      mDoorOpened = new DoorOpenedCanPayloadTranslator(networkDoorOpened, hallway, side);
      canInterface.registerTimeTriggered(networkDoorOpened);
        
      //initialize network interface for door reversal
      networkDoorReversalLeft = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway,Side.LEFT));
      mDoorReversalLeft = new DoorReversalCanPayloadTranslator(networkDoorReversalLeft, hallway, Side.LEFT);
      canInterface.registerTimeTriggered(networkDoorReversalLeft);
      
      networkDoorReversalRight = CanMailbox.getReadableCanMailbox(MessageDictionary.DOOR_REVERSAL_SENSOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway,Side.RIGHT));
      mDoorReversalRight = new DoorReversalCanPayloadTranslator(networkDoorReversalRight, hallway, Side.RIGHT);
      canInterface.registerTimeTriggered(networkDoorReversalRight);
            
      //initialize network interface for car weight 
      networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
      mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
      canInterface.registerTimeTriggered(networkCarWeight);
      
      //initialize network interface for door motor
      networkDoorMotor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
      mDoorMotor = new DoorMotorCommandCanPayloadTranslator(networkDoorMotor, hallway, side);
      canInterface.sendTimeTriggered(networkDoorMotor, period);
        
      timer.start(period);
} 

public void timerExpired(Object callbackData){
    State newState = state;
	int currentFloor = mAtFloor.getCurrentFloor();
    switch(state) {
	case STATE_DOOR_CLOSED:
		//Sets values for state output.
		DoorMotor.set(DoorCommand.STOP);
		mDoorMotor.set(DoorCommand.STOP);
		counter = SimTime.ZERO;
		
		//NudgeCount must be refreshed
		nudgeCount = 0;
		
		// Hua's condition
		//Transition to STATE_DOOR_OPENING; 
		//#transition 'DC1'
		if (currentFloor > 0 && mDriveSpeed.getSpeed() == 0 && mAtFloor.isAtFloor(currentFloor, this.hallway)
			&& currentFloor == mDesiredFloor.getFloor()
			&& (mCarCall.isCarCall(currentFloor, hallway)
			|| mDesiredFloor.getDirection() == Direction.STOP && (mHallCall.isHallCall(currentFloor, hallway, Direction.DOWN) || mHallCall.isHallCall(currentFloor, hallway, Direction.UP))
			|| mDesiredFloor.getDirection() != Direction.STOP && (mHallCall.isHallCall(currentFloor, hallway, mDesiredFloor.getDirection())))) {
			
			newState = State.STATE_DOOR_OPENING;
		}
		
		break;
		
	case STATE_DOOR_OPENING:
		//Sets values for state output.
		DoorMotor.set(DoorCommand.OPEN);
		mDoorMotor.set(DoorCommand.OPEN);
		counter = dwell;
		
		//Transition to STATE_DOOR_OPENED;
		//#transition 'DC2'
		if(mDoorOpened.getValue() == true)
			newState = State.STATE_DOOR_OPENED;
		break;
		
	case STATE_DOOR_OPENED:
		//Sets values for state output.
		DoorMotor.set(DoorCommand.STOP);
		mDoorMotor.set(DoorCommand.STOP);
		counter = SimTime.subtract(counter, period);
		

		//Transition to STATE_DOOR_NUDGING
		//#transition 'DC6'
		if(counter.isLessThan(SimTime.ZERO)){
			if(nudgeCount >= nudgeLimit)
				newState = State.STATE_DOOR_NUDGING;
		//Transition to STATE_DOOR_CLOSING
		//#transition 'DC4'
			else
				newState = State.STATE_DOOR_CLOSING;
		}		
		break;
		
	case STATE_DOOR_CLOSING:
		//Sets values for state output.
		DoorMotor.set(DoorCommand.CLOSE);
		mDoorMotor.set(DoorCommand.CLOSE);
		
		//Transition to STATE_DOOR_CLOSED 
		//#transition 'DC3'
		if(mDoorClosed.getValue() == true)
			newState = State.STATE_DOOR_CLOSED;
		
		//Transition to STATE_DOOR_OPENING
		//#transition 'DC5'
		if( (mCarWeight.getWeight() > Elevator.MaxCarCapacity) ||
			(mHallCall.isHallCall(currentFloor, hallway, mDesiredFloor.getDirection())) ||
		    (mCarCall.isCarCall(currentFloor, hallway)) )
			newState = State.STATE_DOOR_OPENING;
		
		//Transition to STATE_DOOR_REVERSAL
		//#transition 'DC9'		
		if(mDoorReversalLeft.getValue() || mDoorReversalRight.getValue())
			newState = State.STATE_DOOR_REVERSAL;
		
		break;
		
	case STATE_DOOR_NUDGING:
		DoorMotor.set(DoorCommand.NUDGE);
		mDoorMotor.set(DoorCommand.NUDGE);
		
		//Transition to STATE_DOOR_CLOSED
		//#transition 'DC7'
		if(mDoorClosed.getValue())
			newState = State.STATE_DOOR_CLOSED;
		
		//Transition to STATE_DOOR_OPENING
		//#transition 'DC8'
		if(mCarCall.isCarCall(currentFloor, hallway) ||
			mHallCall.isHallCall(currentFloor, hallway, mDesiredFloor.getDirection()) ||
			mCarWeight.getWeight() > Elevator.MaxCarCapacity)
			newState = State.STATE_DOOR_OPENING;
		
		break;
	case STATE_DOOR_REVERSAL:
		//Set Value for Output and State Variable
		DoorMotor.set(DoorCommand.OPEN);
		mDoorMotor.set(DoorCommand.OPEN);
		nudgeCount += 1;
		
		//Transition to STATE_DOOR_OPENING
		//#transition 'DC10'
		newState = State.STATE_DOOR_OPENING;
		break;
	default:
		throw new RuntimeException("State " + state + " was not recognized.");
    }
    //log the results of this iteration
    if (state == newState){
    	log("remains in state: ",state);
    } else {
    	log("Transition:",state,"->",newState);
    }
    
    //update the state variable
    state = newState;
    
    //report the current state
    setState(STATE_KEY,newState.toString());
    
    timer.start(period);;
    
}
}