package simulator.elevatorcontrol;

import jSimPack.SimTime;
import jSimPack.SimTime.SimTimeUnit;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.elevatorcontrol.Utility.TargetDesired;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.elevatormodules.CarWeightCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.IntegerCanPayloadTranslator;

public class Dispatcher extends Controller {

	// Input interface (network)
	private DoorClosedArray frontDoorClosedArray;
	private DoorClosedArray backDoorClosedArray;
	
	private ReadableCanMailbox networkCarWeight;
	private CarWeightCanPayloadTranslator mCarWeight;
	
	private ReadableCanMailbox networkDriveSpeed;
	private DriveSpeedCanPayloadTranslator mDriveSpeed;
	
	private ReadableCanMailbox networkCarLevelPosition;
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
	
	// Output interface (network)
	private WriteableCanMailbox networkDesiredFloor;
	private DesiredFloorCanPayloadTranslator mDesiredFloor;
	
	private WriteableCanMailbox networkDesiredDwellFront;
	private IntegerCanPayloadTranslator mDesiredDwellFront;
	private WriteableCanMailbox networkDesiredDwellBack;
	private IntegerCanPayloadTranslator mDesiredDwellBack;
	
	//States
	private enum State {
		STATE_DOOR_OPEN,
		STATE_DOOR_CLOSE
	}
	
	private State state = State.STATE_DOOR_CLOSE;
	
	//State Variables
	private Utility.CarCallArray carCallArray;
	private Utility.HallCallArray hallCallArray;
	private TargetDesired targetDesired;
	
	private SimTime period;
	
	private final int DWELL_SECONDS = 10;
	private final SimTime DELAY_SECONDS = new SimTime(500, SimTimeUnit.MILLISECOND);
	private SimTime countDown = SimTime.ZERO;
	
	
	public Dispatcher(int numFloors, SimTime period, boolean verbose) {
		super("Dispatcher", verbose);
		
		log("Created Dispatcher with period = ", period);
		this.period = period;
		
		// Init input interface		
		frontDoorClosedArray = new DoorClosedArray(Hallway.FRONT, canInterface);
		backDoorClosedArray = new DoorClosedArray(Hallway.BACK, canInterface);
		
		networkCarWeight = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_WEIGHT_CAN_ID);
		mCarWeight = new CarWeightCanPayloadTranslator(networkCarWeight);
		canInterface.registerTimeTriggered(networkCarWeight);
		
		networkDriveSpeed = CanMailbox.getReadableCanMailbox(MessageDictionary.DRIVE_SPEED_CAN_ID);
		mDriveSpeed = new DriveSpeedCanPayloadTranslator(networkDriveSpeed);
		canInterface.registerTimeTriggered(networkDriveSpeed);
		
		networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
		canInterface.registerTimeTriggered(networkCarLevelPosition);
		
		// Init output interface
		networkDesiredFloor = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.sendTimeTriggered(networkDesiredFloor, period);
		
		networkDesiredDwellFront = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.FRONT));
		mDesiredDwellFront = new IntegerCanPayloadTranslator(networkDesiredDwellFront);
		canInterface.sendTimeTriggered(networkDesiredDwellFront, period);
		networkDesiredDwellBack = CanMailbox.getWriteableCanMailbox(MessageDictionary.DESIRED_DWELL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(Hallway.BACK));
		mDesiredDwellBack = new IntegerCanPayloadTranslator(networkDesiredDwellBack);
		canInterface.sendTimeTriggered(networkDesiredDwellBack, period);
				
		carCallArray = new Utility.CarCallArray(canInterface);
		hallCallArray = new Utility.HallCallArray(canInterface);
		
		targetDesired = new TargetDesired(1, Hallway.NONE, Direction.STOP);
		
		timer.start(period);
	}

	@Override
	public void timerExpired(Object callbackData) {
		
		State newState = state;
				
		switch (state) {
			case STATE_DOOR_CLOSE:
				countDown = SimTime.subtract(countDown, period);
				if (countDown.isLessThan(SimTime.ZERO)) {
					targetDesired = Utility.getTargetForDoorClosed(
							carCallArray, hallCallArray, targetDesired,
							mCarLevelPosition.getPosition(), mDriveSpeed.getSpeed(),
							mDriveSpeed.getDirection(), mCarWeight.getWeight());
				}		
				
				mDesiredFloor.set(targetDesired.floor, targetDesired.direction, targetDesired.hallway);
				mDesiredDwellBack.set(DWELL_SECONDS);
				mDesiredDwellFront.set(DWELL_SECONDS);
				
				//#transition 'DT1'
				if (!frontDoorClosedArray.getBothClosed() ||
					!backDoorClosedArray.getBothClosed()) {
					newState = State.STATE_DOOR_OPEN;
				} else {
					newState = state;
				}
								
				break;
				
			case STATE_DOOR_OPEN:
				
				targetDesired = Utility.getTargetForDoorOpened(carCallArray, hallCallArray, targetDesired);

				mDesiredFloor.set(targetDesired.floor, targetDesired.direction, targetDesired.hallway);
				mDesiredDwellBack.set(DWELL_SECONDS);
				mDesiredDwellFront.set(DWELL_SECONDS);
				countDown = DELAY_SECONDS;
				
				//#transition 'DT2'
				if (frontDoorClosedArray.getBothClosed() &&
					backDoorClosedArray.getBothClosed()) {
					newState = State.STATE_DOOR_CLOSE;
				} else {
					newState = state;
				}
				
				break;
				
			default:
				break;
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
        //you must do this at the end of the timer callback in order to restart
        //the timer
        timer.start(period);
	}

}