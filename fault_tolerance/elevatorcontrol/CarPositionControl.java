/*
 * 18649 Fall 2014
 * Group 4 - Zhu Zhang (zhuzhang), Hua Liu (hual1), Howard Lee (hweekeul)
 */
package simulator.elevatorcontrol;

import jSimPack.SimTime;
import simulator.elevatormodules.CarLevelPositionCanPayloadTranslator;
import simulator.framework.Controller;
import simulator.framework.Elevator;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CarPositionIndicatorPayload;
import simulator.payloads.CarPositionIndicatorPayload.WriteableCarPositionIndicatorPayload;

/**
 * There is only one CarPositionControl instance in the car.
 * 
 * @author Zhu Zhang (zhuzhang)
 */
public class CarPositionControl extends Controller {
	
	/***************************************************************************
     *                      Variable Declarations
     **************************************************************************/
	/* Input Interfaces */
	// mCarLevelPosition input
	private ReadableCanMailbox networkCarLevelPosition;
	private CarLevelPositionCanPayloadTranslator mCarLevelPosition;
	
	/* Output Interfaces */
	// CarPositionIndicator physical output
	private WriteableCarPositionIndicatorPayload localCarPositionIndicator;
	
	/* Local States */
	private int currentFloor;
	
	// enumerate state
	private enum State {
		STATE_DISPLAY,
	}
	// state variable initialized to the initial state STATE_DISPLAY
    private State state = State.STATE_DISPLAY;
	
	private SimTime period;
	
	public CarPositionControl(SimTime period, boolean verbose) {
		super("CarPositionControl", verbose);
		
		this.period = period;
		
		// log Controller creation information
		log("Created CarPositionControl with period = ", period);
		
		// initialize variables
		/* Input Interfaces */	
		// mCarLevelPosition
		networkCarLevelPosition = CanMailbox.getReadableCanMailbox(MessageDictionary.CAR_LEVEL_POSITION_CAN_ID);
		mCarLevelPosition = new CarLevelPositionCanPayloadTranslator(networkCarLevelPosition);
		canInterface.registerTimeTriggered(networkCarLevelPosition);
	
		/* Output Interfaces */
		// CarPositionIndicator physical output
		localCarPositionIndicator = CarPositionIndicatorPayload.getWriteablePayload();
		physicalInterface.sendTimeTriggered(localCarPositionIndicator, period);
		
		// initialize local states
		currentFloor = 1;
		
		// start
		timer.start(period);
	}

	@Override
	public void timerExpired(Object callbackData) {
		State newState = state;
		
		// update current floor		
		for (int i = 1; i <= Elevator.numFloors; i++) {
			if (Math.abs(5000 * (i - 1) - mCarLevelPosition.getPosition()) < 200) {
				currentFloor = i;
				break;
			}
		}
		
		switch (state) {
		case STATE_DISPLAY:
			localCarPositionIndicator.set(currentFloor);
			
			//#transition 'CBCT1'
			if(currentFloor >= 1 && currentFloor <= 8) {
				newState = State.STATE_DISPLAY;
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
        timer.start(period);
	}

}
