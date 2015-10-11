/*
 * 18649 Fall 2014
 * Group 4 - Zhu Zhang (zhuzhang), Hua Liu (hual1), Howard Lee (hweekeul)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatorcontrol.Utility.DoorMotorArray;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
import simulator.elevatormodules.LevelingCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Speed;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox;
import simulator.payloads.DrivePayload;
import simulator.payloads.DriveSpeedPayload;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.DrivePayload.WriteableDrivePayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;

/**
 * There is only one DriveControl in the elevator
 * 
 * @author Zhu Zhang (zhuzhang)
 */
public class DriveControl extends Controller {
	
	/***************************************************************************
     *                      Variable Declarations
     **************************************************************************/
    // note that inputs are Readable objects, while outputs are Writeable objects
	
	// local physical interface
	private ReadableDriveSpeedPayload localDriveSpeed;
	private WriteableDrivePayload localDrive;
	
	// network interface
	// mAtFloor[f,b] input
	private AtFloorArray atFloorArray;
	// mDesiredFloor input
	private ReadableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	// Front doors input
	private DoorClosedArray doorClosedArrayFront;
	// Back doors input
	private DoorClosedArray doorClosedArrayBack;
	// Level Up Sensor input
	private ReadableCanMailbox networkLevelUp;
	private LevelingCanPayloadTranslator mLevelUp;
	// Level Up Sensor input
	private ReadableCanMailbox networkLevelDown;
	private LevelingCanPayloadTranslator mLevelDown;
	// mEmergencyBrake input
	private ReadableCanMailbox networkEmergencyBrake;
	private EfficientBooleanCanPayloadTranslator mEmergencyBrake;
	// mCarLevelPosition input
	private ReadableCanMailbox networkCarLevelPosition;
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
	// mDoorMotor input
	private DoorMotorArray doorMotorArray;
	// mDriveSpeed output
	private WriteableCanMailbox networkDriveSpeed;
	private DriveSpeedCanPayloadTranslator mDriveSpeed;
	
	// local state variables
	private Direction desiredDirection;
	private int currentFloor;

	// store the period for the controller
    private SimTime period;
    
    // shortest distance needed to change the speed to fast
    private int distance;
    
    // enumerate states
    private enum State {
        STATE_STOP,
        STATE_SLOW_UP,
        STATE_SLOW_DOWN,
        STATE_LEVEL_UP,
        STATE_LEVEL_DOWN,
        STATE_FAST_UP,
        STATE_FAST_DOWN,
    }
    // state variable initialized to the initial state STATE_STOP
    private State state = State.STATE_STOP;
    
	public DriveControl(SimTime period, boolean verbose) {
		super("DriveControl", verbose);
		
		this.period = period;
		
		// log Controller creation information
		log("Created DriveControl with period = ", period);
		
		// initialize physical state
		// local DriveSpeed, input
		localDriveSpeed = DriveSpeedPayload.getReadablePayload();
		physicalInterface.registerTimeTriggered(localDriveSpeed);
		
		// local Drive payload, output
		localDrive = DrivePayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(localDrive, period);
		
		// mDesiredFloor network, input
		networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);
		
		// AtFloor network
		atFloorArray = new AtFloorArray(canInterface);
		
		// mDoorMotor input
		doorMotorArray = new DoorMotorArray(canInterface);
		
		// mDoorClosedFrontLeft network
		doorClosedArrayFront = new DoorClosedArray(Hallway.FRONT, canInterface);
		
		// mDoorClosedBackRight network
		doorClosedArrayBack = new DoorClosedArray(Hallway.BACK, canInterface);
		
		// mLevelUp input
		networkLevelUp = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.UP));
		mLevelUp = new LevelingCanPayloadTranslator(networkLevelUp, Direction.UP);
		canInterface.registerTimeTriggered(networkLevelUp);
		
		// mLevelUp input
		networkLevelDown = CanMailbox.getReadableCanMailbox(MessageDictionary.LEVELING_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Direction.DOWN));
		mLevelDown = new LevelingCanPayloadTranslator(networkLevelDown, Direction.DOWN);
		canInterface.registerTimeTriggered(networkLevelDown);
		
		// mEmergencyBrake input
		networkEmergencyBrake = CanMailbox.getReadableCanMailbox(MessageDictionary.EMERGENCY_BRAKE_CAN_ID);
		mEmergencyBrake = new EfficientBooleanCanPayloadTranslator(networkEmergencyBrake);
		canInterface.registerTimeTriggered(networkEmergencyBrake);
		
		// mCarLevelPosition input
		networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
		canInterface.registerTimeTriggered(networkCarLevelPosition);
		
		// mDriveSpeed network, output
		networkDriveSpeed = CanMailbox.getWriteableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
		canInterface.sendTimeTriggered(networkDriveSpeed, period);
		
		// initialize local states
		currentFloor = atFloorArray.getCurrentFloor();
		desiredDirection = Direction.STOP;
		
		// start
		timer.start(period);
	}

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
		
		currentFloor = atFloorArray.getCurrentFloor();
		computeDirection();
		distance = Utility.computeNeededDistance(localDriveSpeed.speed());
		
		switch(state) {
			case STATE_STOP:
				localDrive.set(Speed.STOP, Direction.STOP);
				mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				
				//#transition 'DrC2'
				if (doorClosedArrayBack.getBothClosed() && doorClosedArrayFront.getBothClosed() && desiredDirection == Direction.UP 
						&& localDriveSpeed.direction() != Direction.DOWN && doorMotorArray.getAllStopped()
						&& !mEmergencyBrake.getValue()) {
					newState = State.STATE_SLOW_UP;
				//#transition 'DrC3'
				} else if (doorClosedArrayBack.getBothClosed() && doorClosedArrayFront.getBothClosed() && desiredDirection == Direction.DOWN
						&& localDriveSpeed.direction() != Direction.UP && doorMotorArray.getAllStopped()
						&& !mEmergencyBrake.getValue()) {
					newState = State.STATE_SLOW_DOWN;
				//#transition 'DrC9'
				} else if (!mLevelUp.getValue() && !mEmergencyBrake.getValue() && localDriveSpeed.direction() != Direction.DOWN) {
					newState = State.STATE_LEVEL_UP;
				//#transition 'DrC10'
				} else if (!mLevelDown.getValue() && !mEmergencyBrake.getValue() && localDriveSpeed.direction() != Direction.UP) {
					newState = State.STATE_LEVEL_DOWN;
				} else {
					newState = state;
				}
				break;
			case STATE_SLOW_UP:
				localDrive.set(Speed.SLOW, Direction.UP);
//				mDrive.set(Speed.SLOW, Direction.UP);
				mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				
				//#transition 'DrC7'
				if (mEmergencyBrake.getValue()) {
					newState = State.STATE_STOP;
				}
				//#transition 'DrC1'
				else if (desiredDirection == Direction.STOP) {
					newState = State.STATE_LEVEL_UP;
				}
				//#transition 'DrC13'
				else if (localDriveSpeed.speed() == DriveObject.SlowSpeed 
						&& (mDesiredFloor.getFloor()-1)*5000 - mCarLevelPosition.getValue() > distance) {
					newState = State.STATE_FAST_UP;
				} else {
					newState = state;
				}
				break;
			case STATE_SLOW_DOWN:
				localDrive.set(Speed.SLOW, Direction.DOWN);
//				mDrive.set(Speed.SLOW, Direction.DOWN);
				mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				
				//#transition 'DrC8'
				if (mEmergencyBrake.getValue()) {
					newState = State.STATE_STOP;
				}
				//#transition 'DrC4'
				else if(desiredDirection == Direction.STOP) {
					newState = State.STATE_LEVEL_DOWN;
				} 
				//#transition 'DrC11'
				else if (localDriveSpeed.speed() == DriveObject.SlowSpeed 
						&& mCarLevelPosition.getValue() - (mDesiredFloor.getFloor()-1)*5000 > distance) {
					newState = State.STATE_FAST_DOWN;
				} else {
					newState = state;
				}
				break;
			case STATE_LEVEL_UP:
				localDrive.set(Speed.LEVEL, Direction.UP);
//				mDrive.set(Speed.LEVEL, Direction.UP);
				mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				
				//#transition 'DrC5'
				if((localDriveSpeed.speed() <= DriveObject.LevelingSpeed && mLevelUp.getValue())
						|| mEmergencyBrake.getValue()) {
					newState = State.STATE_STOP;
				} else {
					newState = state;
				}
				break;
			case STATE_LEVEL_DOWN:
				localDrive.set(Speed.LEVEL, Direction.DOWN);
//				mDrive.set(Speed.LEVEL, Direction.DOWN);
				mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				
				//#transition 'DrC6'
				if((localDriveSpeed.speed() <= DriveObject.LevelingSpeed && mLevelDown.getValue())
						|| mEmergencyBrake.getValue()) {
					newState = State.STATE_STOP;
				} else {
					newState = state;
				}
				break;
			case STATE_FAST_UP:
				localDrive.set(Speed.FAST, Direction.UP);
//				mDrive.set(Speed.FAST, Direction.UP);
				mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				
				//#transition 'DrC14'
				if(mEmergencyBrake.getValue() || (mDesiredFloor.getFloor()-1)*5000 - mCarLevelPosition.getValue() <= distance) {
					newState = State.STATE_SLOW_UP;
				}
				break;
			case STATE_FAST_DOWN:
				localDrive.set(Speed.FAST, Direction.DOWN);
//				mDrive.set(Speed.FAST, Direction.DOWN);
				mDriveSpeed.set(localDriveSpeed.speed(), localDriveSpeed.direction());
				
				//#transition 'DrC12'
				if(mEmergencyBrake.getValue() || mCarLevelPosition.getValue() - (mDesiredFloor.getFloor()-1)*5000 <= distance) {
					newState = State.STATE_SLOW_DOWN;
				}
				break;
			default:
				throw new RuntimeException("State " + state + " was not recognized.");
		}
		
		//log the results of this iteration
        if (state == newState) {
            log("remains in state: ",state);
        } else {
            log("Transition:",state,"->",newState);
        }

        //update the state variable
        state = newState;

        //report the current state
        setState(STATE_KEY,newState.toString());

        //schedule the next iteration of the controller
        timer.start(period);
	}
	
	/**
	 * implicitly compute desiredDirection
	 */
	private void computeDirection() {
		if(mDesiredFloor.getFloor() < 1 || mDesiredFloor.getFloor() > 8)
			return;
		if(currentFloor == mDesiredFloor.getFloor())
			desiredDirection = Direction.STOP;
		else
			desiredDirection = currentFloor < mDesiredFloor.getFloor()? Direction.UP : Direction.DOWN;
	}

}
