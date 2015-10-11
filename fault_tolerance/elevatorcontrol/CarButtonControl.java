package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatorcontrol.Utility.AtFloorArray;
import simulator.elevatorcontrol.Utility.DoorClosedArray;
import simulator.framework.Controller;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.payloads.CanMailbox;
import simulator.payloads.CarCallPayload;
import simulator.payloads.CarLightPayload;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.CarCallPayload.ReadableCarCallPayload;
import simulator.payloads.CarLightPayload.WriteableCarLightPayload;

public class CarButtonControl extends Controller {
	
	private final SimTime period;
	
	// keep track of replication
	private final int floor;
	private final Hallway hallway;
	
    // Input interface (physical message)
    private ReadableCarCallPayload localCarCall;
    
    private WriteableCanMailbox networkCarCall;
    private EfficientBooleanCanPayloadTranslator mCarCall;

    // Output interface (physical message)
    private WriteableCarLightPayload localCarLight;
    
    // States and state variables
    private enum State {
    	STATE_CAR_LIGHT_ON,
    	STATE_CAR_LIGHT_OFF
    }
    
    // State variables and Helpers
    private DoorClosedArray doorClosedArray;
    private AtFloorArray atFloorArray;
    
    private State state = State.STATE_CAR_LIGHT_OFF;

	public CarButtonControl(int floor, Hallway hallway, SimTime period, boolean verbose) {

		super("CarButtonControl" + ReplicationComputer.makeReplicationString(floor, hallway), verbose);
		
		this.period = period;
		this.floor = floor;
		this.hallway = hallway;
		
		log("Created CarButtonControl with period = ", period);
		
		// Initialize and register input messages.
		
		localCarCall = CarCallPayload.getReadablePayload(floor, hallway);
		physicalInterface.registerTimeTriggered(localCarCall);
		
		// Initialize and register output messages
		
		networkCarCall = CanMailbox.getWriteableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID + ReplicationComputer.computeReplicationId(floor, hallway));
		mCarCall = new EfficientBooleanCanPayloadTranslator(networkCarCall);
		canInterface.sendTimeTriggered(networkCarCall, period);
		
		localCarLight = CarLightPayload.getWriteablePayload(floor, hallway);
		physicalInterface.sendTimeTriggered(localCarLight, period);
		

		// Initialize state variable
		atFloorArray = new AtFloorArray(canInterface);
		doorClosedArray = new DoorClosedArray(hallway, canInterface);
		
		timer.start(period);
	}

	@Override
	public void timerExpired(Object callbackData) {
		
		State newState = state;
		switch(state) {
			case STATE_CAR_LIGHT_OFF:
				localCarLight.set(false);
				mCarCall.set(false);
				
				//#transition 'CBCT2'
				if (localCarCall.isPressed()) {
					newState = State.STATE_CAR_LIGHT_ON;
				} else {
					newState = state;
				}
				break;
				
			case STATE_CAR_LIGHT_ON:
				localCarLight.set(true);
				mCarCall.set(true);
				
				//#transition 'CBCT1'
				if (atFloorArray.isAtFloor(floor, hallway)
					&& doorClosedArray.getBothClosed() == false) {
					newState = State.STATE_CAR_LIGHT_OFF;
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
