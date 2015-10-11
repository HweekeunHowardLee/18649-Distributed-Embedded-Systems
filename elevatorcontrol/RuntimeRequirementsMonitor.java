package simulator.elevatorcontrol;

import simulator.elevatormodules.DriveObject;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.RuntimeMonitor;
import simulator.framework.Side;
import simulator.framework.Speed;
import simulator.payloads.AtFloorPayload.ReadableAtFloorPayload;
import simulator.payloads.CarLanternPayload.ReadableCarLanternPayload;
import simulator.payloads.DoorClosedPayload.ReadableDoorClosedPayload;
import simulator.payloads.DoorMotorPayload.ReadableDoorMotorPayload;
import simulator.payloads.DoorOpenPayload.ReadableDoorOpenPayload;
import simulator.payloads.DoorReversalPayload.ReadableDoorReversalPayload;
import simulator.payloads.DrivePayload.ReadableDrivePayload;
import simulator.payloads.DriveSpeedPayload.ReadableDriveSpeedPayload;


public class RuntimeRequirementsMonitor extends RuntimeMonitor {
	DoorStateMachine doorState = new DoorStateMachine();
	DriveStateMachine driveState = new DriveStateMachine();
	DoorNudgeStateMachine doorNudgeState = new DoorNudgeStateMachine();
    protected boolean fastSpeedReached = false;

    int rt6warnings = 0;
    int rt7warnings = 0;
    int rt8warnings = 0;
    int rt9warnings = 0;
    int rt10warnings = 0;

    protected int currentFloor = 1;
    protected int lastStoppedFloor = 1;


	@Override
	public void timerExpired(Object callbackData) {
		// do nothing
	}

	@Override
	protected String[] summarize() {
		String[] arr = new String[5];
        arr[0] = "R-T6 Warnings Count = " + rt6warnings;
        arr[1] = "R-T7 Warnings Count = " + rt7warnings;
        arr[2] = "R-T8 Warnings Count = " + rt8warnings;
        arr[3] = "R-T9 Warnigns Count = " + rt9warnings;
        arr[4] = "R-T10 Warnings Count = " + rt10warnings;
        return arr;
	}

    /**************************************************************************
     * high level event methods
     *
     * these are called by the logic in the message receiving methods and the
     * state machines
     **************************************************************************/
	// none currently

    /**************************************************************************
     * low level message receiving methods
     *
     * These mostly forward messages to the appropriate state machines
     **************************************************************************/
    @Override
    public void receive(ReadableDoorClosedPayload msg) {
    	// check RT-10
        doorNudgeState.receive(msg);
    }
    
    @Override
    public void receive(ReadableDoorOpenPayload msg) {
    	if(msg.isOpen()) {
    		// check RT-8.1
        	checkrt81();
        	// check RT-8.2
        	checkrt82();
    	}
    }

    @Override
    public void receive(ReadableDriveSpeedPayload msg) {
    	// check RT-9
    	checkFastSpeed(msg);
    }

    @Override
    public void receive(ReadableAtFloorPayload msg) {
    	// update current floor
        updateCurrentFloor(msg);
    }

    @Override
    public void receive(ReadableDoorMotorPayload msg) {
    	// check RT-10
    	doorNudgeState.receive(msg);
    	// check RT-7
    	doorState.receive(msg);
    }

    @Override
    public void receive(ReadableDoorReversalPayload msg) {
    	// check RT-10
    	doorNudgeState.receive(msg);
    }

    @Override
    public void receive(ReadableDrivePayload msg) {
    	// check RT-6
    	if(msg.speed() == Speed.LEVEL)
    		driveState.updateState();
    }
    
    @Override
    public void receive(ReadableCarLanternPayload msg) {
    	checkrt83();
    }

	private static enum DoorState {
        NoOpen,
        Open,
        NoPendingCall
    }

    private static enum DriveState {
        Moving,
        Stopped,
        NoPendingCall
    }

//    private static enum DriveCommandState {
//    	Slow,
//    	SlowViol,
//    	noViol
//    }

    private static enum DoorNudgeState {
    	NoReversal,
    	Reversal,
    	Nudge
    }
    
    /**
     * RT - 6
     * State Machine monitors drive state.
     * @author Zhu Zhang (zhuzhang)
     */
    private class DriveStateMachine {
    	DriveState state;

    	public DriveStateMachine() {
			state = DriveState.NoPendingCall;
		}

    	private void updateState() {
    		DriveState previousState = state;
            DriveState newState = previousState;

            if (isCarCall() || isHallCall() || lastStoppedFloor == currentFloor || state == DriveState.Stopped) {
            	newState = DriveState.Stopped;
            // only check if there is no pending call when reaching a new floor
            } else {
            	newState = DriveState.NoPendingCall;
            }

            if (newState != previousState) {
                switch (newState) {
                    case Moving:
                        break;
                    case Stopped:
                        break;
                    case NoPendingCall:
                    	rt6warnings++;
                        warning("R-T.6 Violated:  Car stopped at floor " + currentFloor + " when there were no pending calls.");
                        break;
                }
            }

            state = newState;
    	}

    	private boolean isCarCall() {
    		return carCalls[currentFloor-1][Hallway.BACK.ordinal()].isPressed() ||
    				carCalls[currentFloor-1][Hallway.FRONT.ordinal()].isPressed();
    	}

    	private boolean isHallCall() {
    		return hallCalls[currentFloor-1][Hallway.BACK.ordinal()][Direction.UP.ordinal()].pressed() ||
        			hallCalls[currentFloor-1][Hallway.BACK.ordinal()][Direction.DOWN.ordinal()].pressed() ||
        			hallCalls[currentFloor-1][Hallway.FRONT.ordinal()][Direction.UP.ordinal()].pressed() ||
        			hallCalls[currentFloor-1][Hallway.FRONT.ordinal()][Direction.DOWN.ordinal()].pressed();
    	}
    }

    /**
     * RT - 7
     * Utility class for keeping track of the door state.
     *
     * Also provides external methods that can be queried to determine the
     * current door state.
     * @author zhuzhang
     */
    private class DoorStateMachine {
        DoorState state[] = new DoorState[2];

        public DoorStateMachine() {
            state[Hallway.FRONT.ordinal()] = DoorState.NoPendingCall;
            state[Hallway.BACK.ordinal()] = DoorState.NoPendingCall;
        }

        public void receive(ReadableDoorMotorPayload msg) {
        	if(msg.command() == DoorCommand.OPEN)
            	updateState(msg.getHallway());
        }

        private void updateState(Hallway h) {
            DoorState previousState = state[h.ordinal()];
            DoorState newState = previousState;

            if (isCarCall(h) || isHallCall(h) || state[h.ordinal()] == newState) {
                newState = DoorState.Open;
            } else {
                newState = DoorState.NoPendingCall;
            }

            if (newState != previousState) {
                switch (newState) {
                    case NoOpen:
                        break;
                    case Open:
                        break;
                    case NoPendingCall:
                    	rt7warnings++;
                        warning("R-T.7 Violated:  Doors opened at floor " + currentFloor + " " + h + " when there were no pending calls.");
                        break;
                }
            }

            //set the newState
            state[h.ordinal()] = newState;
        }

        //door utility methods
        public boolean isCarCall(Hallway h) {
        	return carCalls[currentFloor-1][h.ordinal()].isPressed();
        }

        public boolean isHallCall(Hallway h) {
        	return hallCalls[currentFloor-1][h.ordinal()][Direction.UP.ordinal()].pressed() ||
        			hallCalls[currentFloor-1][h.ordinal()][Direction.DOWN.ordinal()].pressed();
        }
//
//        public boolean anyDoorOpen() {
//            return anyDoorOpen(Hallway.FRONT) || anyDoorOpen(Hallway.BACK);
//
//        }
//
//        public boolean anyDoorOpen(Hallway h) {
//            return !doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed()
//                    || !doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed();
//        }
    }
    
    /**
     * RT - 8.1
     */
    private void checkrt81() {
    	if(anyOtherCalls() && !isLanternOn(Direction.UP) && !isLanternOn(Direction.DOWN) && mDesiredFloor.getDirection() != Direction.STOP) {
    		rt8warnings++;
    		warning("R-T.8.1 Violated: Doors open and lanterns are off while there is pending calls on other floors.");
    	}
    }
    
    /**
     * RT - 8.2
     */
    private void checkrt82() {
    	if(isLanternBothOn()) {
    		rt8warnings++;
    		warning("R-T.8.2 Violated: One lantern is on, the direction indicated is changed while doors are open.");
    	}
    }
    
    /**
     * RT - 8.3
     */
    private void checkrt83() {
    	if((isLanternOn(Direction.UP) && mDesiredFloor.getFloor() < currentFloor) ||
    			isLanternOn(Direction.DOWN) && mDesiredFloor.getFloor() > currentFloor) {
    		rt8warnings++;
    		warning("R-T.8.3 Violated: One lantern is on, the car doesn't service calls on that direction first.");
    	}
    }
    
    /**
     * RT - 9
     * Warn if the drive was never commanded to fast when fast speed could be
     * used.
     * @param msg
     * @author Howard Lee
     */
    private void checkFastSpeed(ReadableDriveSpeedPayload msg) {
        if (msg.speed() == 0 && currentFloor != MessageDictionary.NONE) {
            //stopped at a floor
            if (lastStoppedFloor != currentFloor) {
                //we've stopped at a new floor
                if (fastSpeedAttainable(lastStoppedFloor, currentFloor)) {
                    //check and see if the drive was ever reached fast
                    if (!fastSpeedReached) {
                    	rt9warnings++;
                    	warning("R-T.9 Violated:  Car could be commanded to drive at Fast between " + lastStoppedFloor + " and " + currentFloor);
                    }
                }
                //now that the check is done, set the lastStoppedFloor to this floor
                lastStoppedFloor = currentFloor;
                //reset fastSpeedReached
                fastSpeedReached = false;
            }
        }
        if (msg.speed() > DriveObject.SlowSpeed) {
            //if the drive exceeds the Slow Speed, the drive must have been commanded to fast speed.
            fastSpeedReached = true;
        }
    }
    
    /**
     * RT - 10
     * State Machine used to monitor door nudging.
     * @author hual1
     */
    private class DoorNudgeStateMachine {
    	DoorNudgeState state[] = new DoorNudgeState[2];

    	public DoorNudgeStateMachine() {
    		state[Hallway.FRONT.ordinal()] = DoorNudgeState.NoReversal;
    		state[Hallway.BACK.ordinal()] = DoorNudgeState.NoReversal;
    	}

        public void receive(ReadableDoorClosedPayload msg) {
            updateState(msg.getHallway());
        }

        public void receive(ReadableDoorMotorPayload msg) {
            updateState(msg.getHallway());
        }

        public void receive(ReadableDoorReversalPayload msg) {
        	updateState(msg.getHallway());
        }

        private void updateState(Hallway h) {
        	DoorNudgeState previousState = state[h.ordinal()];
        	DoorNudgeState newState = previousState;

        	if (isDoorReversal(h)) {
        		newState = DoorNudgeState.Reversal;
        	} else if (isDoorClosed(h)) {
        		newState = DoorNudgeState.NoReversal;
        	} else if (isNudging(h)) {
        		newState = DoorNudgeState.Nudge;
        	}

        	if (newState != previousState) {
        		switch (newState) {
					case Reversal:
						break;
					case NoReversal:
						break;
					case Nudge:
						if (previousState != DoorNudgeState.Reversal) {
							rt10warnings++;
	                        warning("R-T.10 Violated:  Doors nudging at floor " + currentFloor + " " + h + " with no doorReversal triggered before.");
						}
						break;
				}

                //set the newState
                state[h.ordinal()] = newState;
        	}
        }

        private boolean isDoorReversal(Hallway h) {
        	return doorReversals[h.ordinal()][Side.LEFT.ordinal()].isReversing() ||
        			doorReversals[h.ordinal()][Side.RIGHT.ordinal()].isReversing();
        }

        private boolean isDoorClosed(Hallway h) {
        	return doorCloseds[h.ordinal()][Side.LEFT.ordinal()].isClosed() &&
        			doorCloseds[h.ordinal()][Side.RIGHT.ordinal()].isClosed();
        }

        private boolean isNudging(Hallway h) {
        	return (doorMotors[h.ordinal()][Side.LEFT.ordinal()].command() == DoorCommand.NUDGE) ||
        			(doorMotors[h.ordinal()][Side.RIGHT.ordinal()].command() == DoorCommand.NUDGE);
        }
    }


    /**************************************************************************
     * Utility methods
     **************************************************************************/
    private void updateCurrentFloor(ReadableAtFloorPayload lastAtFloor) {
        if (lastAtFloor.getFloor() == currentFloor) {
            //the atFloor message is for the currentfloor, so check both sides to see if they are true
            if (!atFloors[lastAtFloor.getFloor()-1][Hallway.BACK.ordinal()].value() && !atFloors[lastAtFloor.getFloor()-1][Hallway.FRONT.ordinal()].value()) {
                //both sides are false, so set to NONE
                currentFloor = MessageDictionary.NONE;
            }
            //otherwise at least one side is true, so leave the current floor as is
        } else {
            if (lastAtFloor.value()) {
            	currentFloor = lastAtFloor.getFloor();
            }
        }
    }

    /**
     * Computes whether fast speed is attainable.  In general, it is attainable
     * between any two floors.
     *
     * @param startFloor
     * @param endFloor
     * @return true if Fast speed can be commanded between the given floors, otherwise false
     */
    private boolean fastSpeedAttainable(int startFloor, int endFloor) {
        //fast speed is attainable between all floors
        if (startFloor == MessageDictionary.NONE || endFloor == MessageDictionary.NONE) {
            return false;
        }
        if (startFloor != endFloor) {
            return true;
        }
        return false;
    }

//    private void updateCarCalls(int floor, Hallway hallway, boolean on) {
//		carCallsStore[floor-1][hallway.ordinal()] = on;
//	}

//	private void updateHallCalls(int floor, Hallway hallway,
//			Direction direction, boolean on) {
//		hallCallsStore[floor-1][hallway.ordinal()][direction.ordinal()] = on;
//	}
    
    private boolean anyOtherCalls() {
    	for(int i = 1; i <= Elevator.numFloors; i++) {
    		if(i != currentFloor && isCalled(i)) {
    			return true;
    		}
    	}
    	return false;
    }
    
    private boolean isCalled(int floor) {
    	return carCalls[floor-1][Hallway.BACK.ordinal()].pressed() ||
    			carCalls[floor-1][Hallway.FRONT.ordinal()].pressed() ||
    			hallCalls[floor-1][Hallway.BACK.ordinal()][Direction.UP.ordinal()].pressed() ||
    			hallCalls[floor-1][Hallway.BACK.ordinal()][Direction.DOWN.ordinal()].pressed() ||
    			hallCalls[floor-1][Hallway.FRONT.ordinal()][Direction.UP.ordinal()].pressed() ||
    			hallCalls[floor-1][Hallway.FRONT.ordinal()][Direction.DOWN.ordinal()].pressed();
    }
    
    private boolean isLanternOn(Direction direction) {
    	return carLanterns[direction.ordinal()].lighted();
    }
    
    private boolean isLanternBothOn() {
    	return carLanterns[Direction.UP.ordinal()].lighted() &&
    			carLanterns[Direction.DOWN.ordinal()].lighted();
    }
}
