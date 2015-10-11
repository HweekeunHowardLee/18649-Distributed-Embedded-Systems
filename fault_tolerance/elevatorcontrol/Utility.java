/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package simulator.elevatorcontrol;

import java.util.HashMap;
import java.util.Set;
import java.util.TreeSet;

import simulator.elevatormodules.AtFloorCanPayloadTranslator;
import simulator.elevatormodules.DoorClosedCanPayloadTranslator;
import simulator.elevatormodules.DriveObject;
import simulator.elevatormodules.passengers.Passenger;
import simulator.payloads.CANNetwork;
import simulator.framework.Direction;
import simulator.framework.DoorCommand;
import simulator.framework.Elevator;
import simulator.framework.Hallway;
import simulator.framework.Harness;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;

/**
 * This class provides some example utility classes that might be useful in more
 * than one spot. It is okay to create new classes (or modify the ones given
 * below), but you may not use utility classes in such a way that they
 * constitute a communication channel between controllers.
 *
 * @author justinr2
 */
public class Utility {

	public static class DoorClosedArray {

		HashMap<Integer, DoorClosedCanPayloadTranslator>	translatorArray	= new HashMap<Integer, DoorClosedCanPayloadTranslator>();
		public final Hallway								hallway;

		public DoorClosedArray(Hallway hallway, CANNetwork.CanConnection conn) {
			this.hallway = hallway;
			for (Side s : Side.values()) {
				int index = ReplicationComputer
						.computeReplicationId(hallway, s);
				ReadableCanMailbox m = CanMailbox
						.getReadableCanMailbox(MessageDictionary.DOOR_CLOSED_SENSOR_BASE_CAN_ID
								+ index);
				DoorClosedCanPayloadTranslator t = new DoorClosedCanPayloadTranslator(
						m, hallway, s);
				conn.registerTimeTriggered(m);
				translatorArray.put(index, t);
			}
		}

		public boolean getBothClosed() {
			return translatorArray.get(
					ReplicationComputer
							.computeReplicationId(hallway, Side.LEFT))
					.getValue()
					&& translatorArray.get(
							ReplicationComputer.computeReplicationId(hallway,
									Side.RIGHT)).getValue();
		}
	}
	
	public static class DoorMotorArray {

		HashMap<Integer, DoorMotorCommandCanPayloadTranslator>	translatorArray	= new HashMap<Integer, DoorMotorCommandCanPayloadTranslator>();

		public DoorMotorArray(CANNetwork.CanConnection conn) {
			for(Hallway hallway : Hallway.values()) {
				for (Side s : Side.values()) {
					int index = ReplicationComputer
							.computeReplicationId(hallway, s);
					ReadableCanMailbox m = CanMailbox
							.getReadableCanMailbox(MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID
									+ index);
					DoorMotorCommandCanPayloadTranslator t = new DoorMotorCommandCanPayloadTranslator(
							m, hallway, s);
					conn.registerTimeTriggered(m);
					translatorArray.put(index, t);
				}
			}
		}

		public boolean getAllStopped() {
			return (translatorArray.get(
					ReplicationComputer
							.computeReplicationId(Hallway.BACK, Side.LEFT))
					.getCommand() == DoorCommand.STOP)
					&& (translatorArray.get(
							ReplicationComputer.computeReplicationId(Hallway.BACK,
									Side.RIGHT)).getCommand() == DoorCommand.STOP)
					&& (translatorArray.get(
							ReplicationComputer.computeReplicationId(Hallway.FRONT,
									Side.LEFT)).getCommand() == DoorCommand.STOP)
					&& (translatorArray.get(
							ReplicationComputer.computeReplicationId(Hallway.FRONT,
									Side.RIGHT)).getCommand() == DoorCommand.STOP);
		}
	}

	public static class AtFloorArray {

		public HashMap<Integer, AtFloorCanPayloadTranslator>	networkAtFloorsTranslators	= new HashMap<Integer, AtFloorCanPayloadTranslator>();
		public final int										numFloors					= Elevator.numFloors;

		public AtFloorArray(CANNetwork.CanConnection conn) {
			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floor,
							h);
					ReadableCanMailbox m = CanMailbox
							.getReadableCanMailbox(MessageDictionary.AT_FLOOR_BASE_CAN_ID
									+ index);
					AtFloorCanPayloadTranslator t = new AtFloorCanPayloadTranslator(
							m, floor, h);
					conn.registerTimeTriggered(m);
					networkAtFloorsTranslators.put(index, t);
				}
			}
		}

		public boolean isAtFloor(int floor, Hallway hallway) {
			return networkAtFloorsTranslators.get(
					ReplicationComputer.computeReplicationId(floor, hallway))
					.getValue();
		}

		public int getCurrentFloor() {
			int retval = MessageDictionary.NONE;
			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floor,
							h);
					AtFloorCanPayloadTranslator t = networkAtFloorsTranslators
							.get(index);
					if (t.getValue()) {
						if (retval == MessageDictionary.NONE) {
							// this is the first true atFloor
							retval = floor;
						}
						else if (retval != floor) {
							// found a second floor that is different from the
							// first one
							throw new RuntimeException(
									"AtFloor is true for more than one floor at "
											+ Harness.getTime());
						}
					}
				}
			}
			return retval;
		}
	}

	public static void main(String args[]) {
		TargetDesired td1 = new TargetDesired(1, Hallway.BOTH, Direction.STOP);
		TargetDesired td2 = new TargetDesired(2, Hallway.BACK, Direction.STOP);

		System.out.println("td1: " + td1);
		System.out.println("td2: " + td2);
		System.out.println("Equal: " + td1.equals(td2));
		System.out.println("hash1: " + td1.hashCode());
		System.out.println("hash2: " + td2.hashCode());
		System.out.println("td1.compareTo(td2): " + td1.compareTo(td2));
	}

	public static class TargetDesired implements Comparable<TargetDesired> {
		public int			floor;
		public Hallway		hallway;
		public Direction	direction;

		public TargetDesired(int floor, Hallway hallway, Direction direction) {
			this.floor = floor;
			this.hallway = hallway;
			this.direction = direction;
		}

		public TargetDesired(TargetDesired originTD) {
			this.floor = originTD.floor;
			this.hallway = originTD.hallway;
			this.direction = originTD.direction;
		}

		public TargetDesired() {
			this.floor = MessageDictionary.NONE;
			this.hallway = Hallway.NONE;
			this.direction = Direction.STOP;
		}

		public String toString() {
			return "[" + floor + "][" + hallway.name() + "]["
					+ direction.name() + "]";
		}

		@Override
		public int compareTo(TargetDesired o) {
			return hashCode() - o.hashCode();
		}

		@Override
		public boolean equals(Object obj) {
			TargetDesired o = (TargetDesired) obj;

			if (o.floor == floor && o.direction == direction
					&& o.hallway == hallway) {
				return true;
			}
			else {
				return false;
			}
		}

		@Override
		public int hashCode() {
			return floor * 1000 + hallway.ordinal() * 100 + direction.ordinal()
					* 10;
		}

	}

	public static class CarCallArray {

		public HashMap<Integer, CarCallCanPayloadTranslator>	networkCarCallsTranslators	= new HashMap<Integer, CarCallCanPayloadTranslator>();
		public final int										numFloors					= Elevator.numFloors;

		public CarCallArray(CANNetwork.CanConnection conn) {
			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;
				for (Hallway h : Hallway.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floor,
							h);
					ReadableCanMailbox m = CanMailbox
							.getReadableCanMailbox(MessageDictionary.CAR_CALL_BASE_CAN_ID
									+ index);
					CarCallCanPayloadTranslator t = new CarCallCanPayloadTranslator(
							m);
					conn.registerTimeTriggered(m);
					networkCarCallsTranslators.put(index, t);
				}
			}
		}

		public boolean isCarCall(int floor, Hallway hallway) {
			return networkCarCallsTranslators.get(
					ReplicationComputer.computeReplicationId(floor, hallway))
					.getValue();
		}

		public boolean isCarCall(int floor) {
			return isCarCall(floor, Hallway.FRONT)
					|| isCarCall(floor, Hallway.BACK);
		}

		public TDSet getCarCall() {
			TDSet result = new TDSet();

			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;

				int frontIndex = ReplicationComputer.computeReplicationId(
						floor, Hallway.FRONT);
				int backIndex = ReplicationComputer.computeReplicationId(floor,
						Hallway.BACK);

				TargetDesired thisTargetDesired = new TargetDesired();
				thisTargetDesired.floor = floor;
				thisTargetDesired.direction = Direction.STOP;

				if (networkCarCallsTranslators.get(frontIndex).getValue()
						&& networkCarCallsTranslators.get(backIndex).getValue()) {
					thisTargetDesired.hallway = Hallway.BOTH;
					result.tdSet.add(thisTargetDesired);
				}
				else if (networkCarCallsTranslators.get(frontIndex).getValue()) {
					thisTargetDesired.hallway = Hallway.FRONT;
					result.tdSet.add(thisTargetDesired);
				}
				else if (networkCarCallsTranslators.get(backIndex).getValue()) {
					thisTargetDesired.hallway = Hallway.BACK;
					result.tdSet.add(thisTargetDesired);
				}
			}

			return result;
		}

		public void printCarCallArray() {
			System.out.println("CarCall[f, b]:");
			for (int floor = 1; floor <= numFloors; floor++) {
				for (Hallway hallway : Hallway.replicationValues) {
					int index = ReplicationComputer.computeReplicationId(floor,
							hallway);
					if (networkCarCallsTranslators.get(index).getValue()) {
						System.out.println("CarCall[" + floor + ", " + hallway
								+ "]");
					}
				}
			}
		}

	}

	public static class HallCallArray {
		public HashMap<Integer, HallCallCanPayloadTranslator>	networkHallCallsTranslators	= new HashMap<Integer, HallCallCanPayloadTranslator>();
		public final int										numFloors					= Elevator.numFloors;

		public HallCallArray(CANNetwork.CanConnection conn) {
			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;
				for (Hallway hallway : Hallway.replicationValues) {
					for (Direction direction : Direction.replicationValues) {
						int index = ReplicationComputer.computeReplicationId(
								floor, hallway, direction);
						ReadableCanMailbox m = CanMailbox
								.getReadableCanMailbox(MessageDictionary.HALL_CALL_BASE_CAN_ID
										+ index);
						HallCallCanPayloadTranslator t = new HallCallCanPayloadTranslator(
								m);
						conn.registerTimeTriggered(m);
						// System.out.println("Initializing HallCall Map(" +
						// index + "): floor=" + floor + " hallway=" + hallway +
						// " direction=" + direction);
						networkHallCallsTranslators.put(index, t);
					}
				}
			}
		}

		public TDSet getHallCall() {
			TDSet result = new TDSet();

			for (int i = 0; i < numFloors; i++) {
				int floor = i + 1;
				for (Direction direction : Direction.replicationValues) {

					int frontIndex = ReplicationComputer.computeReplicationId(
							floor, Hallway.FRONT, direction);
					int backIndex = ReplicationComputer.computeReplicationId(
							floor, Hallway.BACK, direction);

					TargetDesired thisTargetDesired = new TargetDesired();
					thisTargetDesired.floor = floor;
					thisTargetDesired.direction = direction;

					if (networkHallCallsTranslators.get(frontIndex).getValue()
							&& networkHallCallsTranslators.get(backIndex)
									.getValue()) {
						thisTargetDesired.hallway = Hallway.BOTH;
						result.tdSet.add(thisTargetDesired);
						// System.out.println("Getting " + floor + ":" +
						// direction + ":BOTH");
					}
					else if (networkHallCallsTranslators.get(frontIndex)
							.getValue()) {
						thisTargetDesired.hallway = Hallway.FRONT;
						result.tdSet.add(thisTargetDesired);
						// System.out.println("Getting " + floor + ":" +
						// direction + ":FRONT");
					}
					else if (networkHallCallsTranslators.get(backIndex)
							.getValue()) {
						thisTargetDesired.hallway = Hallway.BACK;
						result.tdSet.add(thisTargetDesired);
						// System.out.println("Getting " + floor + ":" +
						// direction + ":BACK");
					}
				}
			}

			return result;
		}

		public boolean isHallCall(int floor, Hallway hallway,
				Direction direction) {
			return networkHallCallsTranslators.get(
					ReplicationComputer.computeReplicationId(floor, hallway,
							direction)).getValue();
		}

		public boolean isHallCall(int floor, Direction direction) {
			return isHallCall(floor, Hallway.BACK, direction)
					|| isHallCall(floor, Hallway.FRONT, direction);
		}

		public boolean isHallCall(int floor) {
			return isHallCall(floor, Direction.UP)
					|| isHallCall(floor, Direction.DOWN);
		}

		public void printHallCallArray() {
			System.out.println("HallCall[f, b, d]:");
			for (int floor = 1; floor <= numFloors; floor++) {
				for (Hallway hallway : Hallway.replicationValues) {
					for (Direction direction : Direction.replicationValues) {
						int index = ReplicationComputer.computeReplicationId(
								floor, hallway, direction);
						if (networkHallCallsTranslators.get(index).getValue()) {
							System.out.println("HallCall[" + floor + ", "
									+ hallway + ", " + direction + "]");
						}
					}
				}
			}
		}
	}

	public static class TDSet {
		public Set<TargetDesired>	tdSet;

		public TDSet() {
			tdSet = new TreeSet<TargetDesired>();
		}

		public TDSet filterBy(Direction d) {
			TDSet resultSet = new TDSet();

			for (TargetDesired td : tdSet) {
				if (td.direction == d)
					resultSet.tdSet.add(td);
			}

			return resultSet;
		}

		public TDSet filterBy(int loFloor, int hiFloor) {
			TDSet resultSet = new TDSet();

			for (TargetDesired td : tdSet) {
				if (td.floor >= loFloor && td.floor <= hiFloor)
					resultSet.tdSet.add(td);
			}

			return resultSet;
		}

		public TDSet excludeBy(int floor) {
			TDSet resultSet = new TDSet();

			for (TargetDesired td : tdSet) {
				if (td.floor != floor)
					resultSet.tdSet.add(td);
			}

			return resultSet;
		}

		public TDSet getBy(int floor) {
			TDSet resultSet = new TDSet();

			for (TargetDesired td : tdSet) {
				if (td.floor == floor)
					resultSet.tdSet.add(td);
			}

			return resultSet;
		}

		public String toString() {
			StringBuilder resultSB = new StringBuilder();

			for (TargetDesired td : tdSet) {
				resultSB.append(td.toString()).append('\n');
			}

			return resultSB.toString();
		}
	}

	public static TargetDesired getTargetForDoorOpened(
			CarCallArray carCallArray, HallCallArray hallCallArray,
			TargetDesired currentTD) {
//		System.out.println("---AT DOOR OPENED---");

		TargetDesired targetDesired = new TargetDesired(currentTD);

		if (currentTD.direction == Direction.STOP) {
			TDSet upCarCallTD = carCallArray.getCarCall().filterBy(
					currentTD.floor + 1, Elevator.numFloors);
			TDSet downCarCallTD = carCallArray.getCarCall().filterBy(1,
					currentTD.floor - 1);
			TDSet upHallCallTD = hallCallArray.getHallCall().filterBy(
					currentTD.floor + 1, Elevator.numFloors);
			TDSet downHallCallTD = hallCallArray.getHallCall().filterBy(1,
					currentTD.floor - 1);

			if (!upCarCallTD.tdSet.isEmpty() || !upHallCallTD.tdSet.isEmpty()) {
				targetDesired.direction = Direction.UP;
			}
			else if (!downCarCallTD.tdSet.isEmpty()
					|| !downHallCallTD.tdSet.isEmpty()) {
				targetDesired.direction = Direction.DOWN;
			}
		}

		return targetDesired;
	}

	public static TargetDesired getTargetForDoorClosed(
			CarCallArray carCallArray, HallCallArray hallCallArray,
			TargetDesired currentTD, int carLevelPosition, double driveSpeed,
			Direction movingDirection, int carWeight) {
		TargetDesired targetDesired = new TargetDesired(currentTD);

//		System.out.println("---AT DOOR CLOSED---");
//		System.out.println("currentTD: " + currentTD);
//		System.out.println("currentHallCall: ");
//		System.out.println(hallCallArray.getHallCall().toString());
//		System.out.println("currentCarCall: ");
//		System.out.println(carCallArray.getCarCall().toString());
		
		boolean ignoreHallCall = false;
		if (carWeight + Passenger.DEFAULT_WEIGHT >= Elevator.MaxCarCapacity) {
			ignoreHallCall = true;
		}

		if (currentTD.direction == Direction.STOP) {
//			System.out.println("Original TD.d is STOP, search for new TD.d.");

			// search calls based on currentTD.f
			// search result only affects targetDesired.d

			// for hall calls, just check if there are any valid hall call on
			// other floors.
			// call direction doesn't matter.
			TDSet upperHallCallTD = hallCallArray.getHallCall().filterBy(
					currentTD.floor + 1, Elevator.numFloors);
			TDSet lowerHallCallTD = hallCallArray.getHallCall().filterBy(1,
					currentTD.floor - 1);
			TDSet curHallCallTD = hallCallArray.getHallCall().getBy(
					currentTD.floor);
			
			if (ignoreHallCall) {
				upperHallCallTD.tdSet.clear();
				lowerHallCallTD.tdSet.clear();
				curHallCallTD.tdSet.clear();
			}

			// for car calls, do not care calls on currentTD.f for it doesn't
			// affect TD.d
			TDSet upperCarCallTD = carCallArray.getCarCall().filterBy(
					currentTD.floor + 1, Elevator.numFloors);
			TDSet lowerCarCallTD = carCallArray.getCarCall().filterBy(1,
					currentTD.floor - 1);

			if (!upperCarCallTD.tdSet.isEmpty()) {
				targetDesired.direction = Direction.UP;
//				System.out.println("TD.d changed to UP because of upper car call.");
			}
			else if (!lowerCarCallTD.tdSet.isEmpty()) {
				targetDesired.direction = Direction.DOWN;
//				System.out.println("TD.d changed to DOWN because of lower car call.");
			}
			else if (!curHallCallTD.tdSet.isEmpty()) {
				for (TargetDesired td : curHallCallTD.tdSet) {
					targetDesired.direction = td.direction;
					// only check the first one is enough.
					break;
				}
			}
			else if (!upperHallCallTD.tdSet.isEmpty()) {
				targetDesired.direction = Direction.UP;
//				System.out.println("TD.d changed to UP because of upper hall call.");
			}
			else if (!lowerHallCallTD.tdSet.isEmpty()) {
				targetDesired.direction = Direction.DOWN;
//				System.out.println("TD.d changed to DOWN because of lower hall call.");

			}
		}
		else {
//			System.out.println("Original TD.d is " + currentTD.direction + ", optimizing...");
			int validPosition = carLevelPosition;
			int validCurrentFloor;
			switch (movingDirection) {
				case UP:

					if (driveSpeed > DriveObject.SlowSpeed) {
						validPosition += computeNeededDistance(driveSpeed);
					}
					validPosition += MARGIN_DIST;

					validCurrentFloor = validPosition / 5000 + 1;
					break;

				case DOWN:

					if (driveSpeed > DriveObject.SlowSpeed) {
						// validPosition -= COMMIT_POINT_DIST;
						validPosition -= computeNeededDistance(driveSpeed);
					}
					validPosition -= MARGIN_DIST;
					validCurrentFloor = validPosition / 5000 + 2;
					break;
				case STOP:
//					System.out.println("position: " + validPosition);
					// failsafe.
					validCurrentFloor = validPosition / 5000 + 1;

					// mitigate 10cm carLevelPosition sensor error when parking.
					for (int i = 1; i <= Elevator.numFloors; i++) {

						if (Math.abs(5000 * (i - 1) - validPosition) < 200) {
							validCurrentFloor = i;
							break;
						}
					}
//					System.out.println("floor: " + validCurrentFloor);

					break;
				default:
					validCurrentFloor = validPosition / 5000 + 1;
					break;
			}

//			System.out.println("Valid current floor: " + validCurrentFloor);

			// -----------------------------------------------
			// Search closest calls
			// -----------------------------------------------

			// if targetDirection is being held by valid call, skip this step.
			// i.e., proceed only if there are no calls on currentTD, which
			// means this "currentTD" is already served and the car is ready
			// to go.

			// This branch should be executed when elevator door is just closed.
			if (movingDirection == Direction.STOP
					&&
					carCallArray.getCarCall().getBy(currentTD.floor).tdSet
							.isEmpty()
					&& hallCallArray.getHallCall().getBy(currentTD.floor)
							.filterBy(currentTD.direction).tdSet.isEmpty()) {
				/*
				 * carCallArray.getCarCall().getBy(validCurrentFloor).tdSet.isEmpty
				 * () &&
				 * hallCallArray.getHallCall().getBy(validCurrentFloor).filterBy
				 * (currentTD.direction).tdSet.isEmpty()) {
				 */
//				System.out.println("Searching closest calls...");

				TDSet carCallTD = carCallArray.getCarCall();
				TDSet hallCallTD = hallCallArray.getHallCall();
				
				if (ignoreHallCall) hallCallTD.tdSet.clear();

				// "moving direction" is the equivalent of "desired direction"
				// at this time.
				// therefore we should search along "desired direction" for next
				// call.

				if (currentTD.direction == Direction.UP) {
					carCallTD = carCallTD.filterBy(currentTD.floor + 1,
							Elevator.numFloors);
					hallCallTD = hallCallTD.filterBy(currentTD.floor + 1,
							Elevator.numFloors);
				}
				else {
					carCallTD = carCallTD.filterBy(1, currentTD.floor - 1);
					hallCallTD = hallCallTD.filterBy(1, currentTD.floor - 1);
				}

				if (carCallTD.tdSet.isEmpty() && hallCallTD.tdSet.isEmpty()) {
//					System.out.println("No valid call. Keep searching.");
					targetDesired.direction = Direction.STOP;
				}
				else {
//					System.out.println("Found valid call. Filtering.");

					TDSet forwardHallCallTD = hallCallTD
							.filterBy(currentTD.direction);
					TDSet backwardHallCallTD = hallCallTD
							.filterBy(flipDirection(currentTD.direction));

					if (!carCallTD.tdSet.isEmpty()
							&& !forwardHallCallTD.tdSet.isEmpty()) {
//						System.out.println("Both car call and hall call found.");
						int carCallDist = Integer.MAX_VALUE;
						TargetDesired nearestCarCall = new TargetDesired();
						for (TargetDesired td : carCallTD.tdSet) {
							if (carCallDist > Math.abs(td.floor
									- validCurrentFloor)) {
								carCallDist = Math.abs(td.floor
										- validCurrentFloor);
								nearestCarCall = td;
							}
						}

						int hallCallDist = Integer.MAX_VALUE;
						TargetDesired nearestHallCall = new TargetDesired();
						for (TargetDesired td : forwardHallCallTD.tdSet) {
							if (hallCallDist > Math.abs(td.floor
									- validCurrentFloor)) {
								hallCallDist = Math.abs(td.floor
										- validCurrentFloor);
								nearestHallCall = td;
							}
						}

						if (hallCallDist < carCallDist) {
							targetDesired.floor = nearestHallCall.floor;
							targetDesired.hallway = nearestHallCall.hallway;
							targetDesired.direction = nearestHallCall.direction;
//							System.out.println("Using nearest hall call: " + nearestHallCall);
						}
						else {
							targetDesired.floor = nearestCarCall.floor;
							targetDesired.hallway = nearestCarCall.hallway;
							targetDesired.direction = Direction.STOP;
//							System.out.println("Using nearest car call: " + nearestCarCall);
						}

					}
					else if (!carCallTD.tdSet.isEmpty()) {
//						System.out.println("Car call found.");
						int carCallDist = Integer.MAX_VALUE;
						TargetDesired nearestCarCall = new TargetDesired();
						for (TargetDesired td : carCallTD.tdSet) {
							if (carCallDist > Math.abs(td.floor
									- validCurrentFloor)) {
								carCallDist = Math.abs(td.floor
										- validCurrentFloor);
								nearestCarCall = td;
							}
						}

						targetDesired.floor = nearestCarCall.floor;
						targetDesired.hallway = nearestCarCall.hallway;
						targetDesired.direction = Direction.STOP;
//						System.out.println("Using nearest car call: " + nearestCarCall);
					}
					else if (!forwardHallCallTD.tdSet.isEmpty()) {
//						System.out.println("Hall call found.");
						int hallCallDist = Integer.MAX_VALUE;
						TargetDesired nearestHallCall = new TargetDesired();
						for (TargetDesired td : forwardHallCallTD.tdSet) {
							if (hallCallDist > Math.abs(td.floor
									- validCurrentFloor)) {
								hallCallDist = Math.abs(td.floor
										- validCurrentFloor);
								nearestHallCall = td;
							}
						}

						targetDesired.floor = nearestHallCall.floor;
						targetDesired.hallway = nearestHallCall.hallway;
						targetDesired.direction = nearestHallCall.direction;
//						System.out.println("Using nearest hall call: " + nearestHallCall);
					}
					else {
//						System.out.println("No car call or forward hall call found. Searching backward hall call.");

						int hallCallDist = 0;
						TargetDesired furthestHallCall = new TargetDesired();
						for (TargetDesired td : backwardHallCallTD.tdSet) {
							if (hallCallDist < Math.abs(td.floor
									- validCurrentFloor)) {
								hallCallDist = Math.abs(td.floor
										- validCurrentFloor);
								furthestHallCall = td;
							}
						}

						targetDesired.floor = furthestHallCall.floor;
						targetDesired.hallway = furthestHallCall.hallway;
						targetDesired.direction = furthestHallCall.direction;
//						System.out.println("Using furthest hall call: " + furthestHallCall);
					}
				}
			}
			else {

//				System.out.println("Search for better calls.");

				// -----------------------------------------------
				// Search for better calls
				// -----------------------------------------------

				// CurrentFloor            TargetFloor
				//     |                        |
				//     V                        V
				// ----|------------------------|-----------------
				//     |<----- near ----------->|<----- far ----->

				// find valid calls between validCurrentFloor and TD.f

				if (movingDirection == Direction.STOP) {
					movingDirection = currentTD.direction;
				}

				TDSet nearCarCallTD = carCallArray.getCarCall();
				TDSet nearHallCallTD = hallCallArray.getHallCall();
				if (movingDirection == Direction.UP) {
					nearCarCallTD = nearCarCallTD.filterBy(
							validCurrentFloor + 1, currentTD.floor - 1);
					nearHallCallTD = nearHallCallTD.filterBy(
							validCurrentFloor + 1, currentTD.floor - 1)
							.filterBy(movingDirection);
				}
				else {
					nearCarCallTD = nearCarCallTD.filterBy(currentTD.floor + 1,
							validCurrentFloor - 1);
					nearHallCallTD = nearHallCallTD.filterBy(
							currentTD.floor + 1, validCurrentFloor - 1)
							.filterBy(movingDirection);
				}

				TDSet farHallCallTD = hallCallArray.getHallCall();
				// far section is considered only when movingDirection !=
				// currentTD.d
				if (movingDirection != currentTD.direction) {
					if (movingDirection == Direction.UP) {
						farHallCallTD = farHallCallTD.filterBy(Direction.DOWN)
								.filterBy(currentTD.floor + 1,
										Elevator.numFloors);
					}
					else {
						farHallCallTD = farHallCallTD.filterBy(Direction.UP)
								.filterBy(1, currentTD.floor - 1);
					}
				}
				else {
					farHallCallTD.tdSet.clear();
				}
				
				if (ignoreHallCall) {
					nearHallCallTD.tdSet.clear();
					farHallCallTD.tdSet.clear();
				}

				if (!nearCarCallTD.tdSet.isEmpty()
						&& !nearHallCallTD.tdSet.isEmpty()) {
					// found hall calls and car calls in near section
//					System.out.println("found hall calls and car calls in near section");
					int carCallDist = Integer.MAX_VALUE;
					TargetDesired nearestCarCall = new TargetDesired();
					for (TargetDesired td : nearCarCallTD.tdSet) {
						if (carCallDist > Math
								.abs(td.floor - validCurrentFloor)) {
							carCallDist = Math
									.abs(td.floor - validCurrentFloor);
							nearestCarCall = td;
						}
					}

					int hallCallDist = Integer.MAX_VALUE;
					TargetDesired nearestHallCall = new TargetDesired();
					for (TargetDesired td : nearHallCallTD.tdSet) {
						if (hallCallDist > Math.abs(td.floor
								- validCurrentFloor)) {
							hallCallDist = Math.abs(td.floor
									- validCurrentFloor);
							nearestHallCall = td;
						}
					}

					if (hallCallDist < carCallDist) {
						targetDesired.floor = nearestHallCall.floor;
						targetDesired.hallway = nearestHallCall.hallway;
					}
					else {
						targetDesired.floor = nearestCarCall.floor;
						targetDesired.hallway = nearestCarCall.hallway;
					}

					targetDesired.direction = movingDirection;
				}
				else if (!nearCarCallTD.tdSet.isEmpty()) {
					// found car call in near section only.
//					System.out.println("found car call in near section only");

					int carCallDist = Integer.MAX_VALUE;
					TargetDesired nearestCarCall = new TargetDesired();
					for (TargetDesired td : nearCarCallTD.tdSet) {
						if (carCallDist > Math
								.abs(td.floor - validCurrentFloor)) {
							carCallDist = Math
									.abs(td.floor - validCurrentFloor);
							nearestCarCall = td;
						}
					}

					targetDesired.floor = nearestCarCall.floor;
					targetDesired.hallway = nearestCarCall.hallway;
					targetDesired.direction = movingDirection;
				}
				else if (!nearHallCallTD.tdSet.isEmpty()) {
					// found hall call in near section only.
//					System.out.println("found hall call in near section only");

					int hallCallDist = Integer.MAX_VALUE;
					TargetDesired nearestHallCall = new TargetDesired();
					for (TargetDesired td : nearHallCallTD.tdSet) {
						if (hallCallDist > Math.abs(td.floor
								- validCurrentFloor)) {
							hallCallDist = Math.abs(td.floor
									- validCurrentFloor);
							nearestHallCall = td;
						}
					}

					targetDesired.floor = nearestHallCall.floor;
					targetDesired.hallway = nearestHallCall.hallway;
					targetDesired.direction = movingDirection;
				}
				else if (!farHallCallTD.tdSet.isEmpty()) {
					// no calls in near section, search far section.
//					System.out.println("no calls in near section, search far section");
					int hallCallDist = 0;
					TargetDesired furthestHallCall = new TargetDesired();
					for (TargetDesired td : farHallCallTD.tdSet) {
						if (hallCallDist < Math.abs(td.floor
								- validCurrentFloor)) {
							hallCallDist = Math.abs(td.floor
									- validCurrentFloor);
							furthestHallCall = td;
						}
					}

					targetDesired.floor = furthestHallCall.floor;
					targetDesired.hallway = furthestHallCall.hallway;
					targetDesired.direction = furthestHallCall.direction;
				}
				else {
//					System.out.println("no calls found. DF unchanged.");
				}

			}
		}

		return targetDesired;
	}

	// static final int MARGIN_DIST = 100;
	static final int	MARGIN_DIST	= 0;

	public static int computeNeededDistance(double speed) {
		return (int) (Math.pow(speed, 2) * 1000 / 2 + 100 * speed + 200);
	}

	public static Direction flipDirection(Direction d) {
		switch (d) {
			case DOWN:
				return Direction.UP;
			case UP:
				return Direction.DOWN;
			default:
				return d;
		}
	}
}
