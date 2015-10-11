package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.framework.DoorCommand;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.framework.Side;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DoorMotorCommandCanPayloadTranslator extends CanPayloadTranslator {
	private final Hallway hallway;
	private final Side side;

	public DoorMotorCommandCanPayloadTranslator(ReadableCanMailbox payload,
			Hallway hallway, Side side) {
		super(payload, 1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
		this.hallway = hallway;
		this.side = side;
	}
	
	public DoorMotorCommandCanPayloadTranslator(WriteableCanMailbox payload,
			Hallway hallway, Side side) {
		super(payload, 1, MessageDictionary.DOOR_MOTOR_COMMAND_BASE_CAN_ID + ReplicationComputer.computeReplicationId(hallway, side));
		this.hallway = hallway;
		this.side = side;
	}
	
	public void set(DoorCommand command) {
		BitSet b = getMessagePayload();
		addIntToBitset(b, command.ordinal(), 0, 8);
        setMessagePayload(b, getByteSize());
	}
	
	public DoorCommand getCommand() {
		int val = getIntFromBitset(getMessagePayload(), 0, 8);
        for (DoorCommand c : DoorCommand.values()) {
            if (c.ordinal() == val) {
                return c;
            }
        }
        throw new RuntimeException("Unrecognized DoorMotorCommand Value " + val);
	}

	@Override
	public String payloadToString() {
		return "DoorMotorCommand[" + hallway + "][" + side + "] = " + getCommand();
	}

}
