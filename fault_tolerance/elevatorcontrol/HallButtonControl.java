package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.HallCallPayload.ReadableHallCallPayload;
import simulator.payloads.HallLightPayload;
import simulator.payloads.HallLightPayload.WriteableHallLightPayload;
import simulator.payloads.HallCallPayload;

public class HallButtonControl extends Controller {
	
	//these variables keep track of which instance this is.
    private final Hallway hallway;
    private final Direction direction;
    private final int floor;
    
    // Input interface (network message)
    private ReadableCanMailbox networkAtFloor;
    
    private ReadableCanMailbox networkDesiredFloor;
    private DesiredFloorCanPayloadTranslator mDesiredFloor;
    
    // Input interface (physical message)
    private ReadableHallCallPayload localHallCall;
    
    private WriteableCanMailbox networkHallCall;
    private EfficientBooleanCanPayloadTranslator mHallCall;
    
    // Output interface (physical message)
    private WriteableHallLightPayload localHallLight;
    
    // State variables and Helpers
    private DoorClosedArray doorClosedArray;
    private AtFloorArray atFloorArray;
    
    // Controller states
    private enum State {
    	STATE_HALL_LIGHT_ON,
    	STATE_HALL_LIGHT_OFF
    };
    private State state = State.STATE_HALL_LIGHT_OFF;
    
    private SimTime period;
	
	public HallButtonControl(int floor, Hallway hallway, Direction direction, SimTime period, boolean verbose) {
		super("HallButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway, direction), verbose);
		
		this.floor = floor;
		this.hallway = hallway;
		this.direction = direction;
		this.period = period;
		
		log("Created HallButtonControl with period = ", period);
		
		// Initialize and register input messages.
		
		networkAtFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
		canInterface.registerTimeTriggered(networkAtFloor);
		
		networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
		mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
		canInterface.registerTimeTriggered(networkDesiredFloor);
		
		localHallCall = HallCallPayload.getReadablePayload(floor, hallway, direction);
		physicalInterface.registerTimeTriggered(localHallCall);
		
		// Initialize and register output messages
		
		networkHallCall = CanMailbox.getWriteableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway, direction));
		mHallCall = new EfficientBooleanCanPayloadTranslator(networkHallCall);
		canInterface.sendTimeTriggered(networkHallCall, period);
		
		localHallLight = HallLightPayload.getWriteablePayload(floor, hallway, direction);
		physicalInterface.sendTimeTriggered(localHallLight, period);
		
		// Initialize state variable
		atFloorArray = new AtFloorArray(canInterface);
		doorClosedArray = new DoorClosedArray(hallway, canInterface);
		
		timer.start(period);
	}

	public void timerExpired(Object callbackData) {
		// Update variables
		
		State newState = state;
		switch (state) {
			
			case STATE_HALL_LIGHT_OFF:
				localHallLight.set(false);
				mHallCall.set(false);
				
				//#transition 'HBCT1'
				if (localHallCall.pressed()) {
					newState = State.STATE_HALL_LIGHT_ON;
				} else {
					newState = state;
				}
				break;
			
			case STATE_HALL_LIGHT_ON:
				localHallLight.set(true);
				mHallCall.set(true);
				
				//#transition 'HBCT2'
				if (atFloorArray.isAtFloor(floor, hallway)
					&& mDesiredFloor.getDirection() == direction
					&& doorClosedArray.getBothClosed() == false) {
					newState = State.STATE_HALL_LIGHT_OFF;
				} else {
					newState = state;
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
        //you must do this at the end of the timer callback in order to restart
        //the timer
        timer.start(period);
	}

}
