package simulator.elevatorcontrol;

import java.util.BitSet;

import simulator.framework.Direction;
import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;
import simulator.payloads.translators.CanPayloadTranslator;

public class DriveSpeedCanPayloadTranslator extends CanPayloadTranslator {
	private static final int PRECISION = 100;

	public DriveSpeedCanPayloadTranslator(ReadableCanMailbox payload) {
		super(payload, 3, MessageDictionary.DRIVE_SPEED_CAN_ID);
	}
	
	public DriveSpeedCanPayloadTranslator(WriteableCanMailbox payload) {
		super(payload, 3, MessageDictionary.DRIVE_SPEED_CAN_ID);
	}
	
	public void set(double speed, Direction dir) {
        setSpeed(speed);
        setDirection(dir);
    }
	
	public void set(Direction dir, double speed) {
        setSpeed(speed);
        setDirection(dir);
    }
    
    public void setSpeed(double speed) {
        BitSet b = getMessagePayload();
        short s = (short) (speed * PRECISION);
        addIntToBitset(b, s, 0, 16);
        setMessagePayload(b, getByteSize());
    }

    public double getSpeed() {
        int val = getIntFromBitset(getMessagePayload(), 0, 16);
        return (double) val / PRECISION;
    }

    public void setDirection(Direction dir) {
        BitSet b = getMessagePayload();
        addIntToBitset(b, dir.ordinal(), 16, 4);
        setMessagePayload(b, getByteSize());
    }

    public Direction getDirection() {
        int val = getIntFromBitset(getMessagePayload(), 16, 4);
        for (Direction d : Direction.values()) {
            if (d.ordinal() == val) {
                return d;
            }
        }
        throw new RuntimeException("Unrecognized Direction Value " + val);
    }
    
	@Override
	public String payloadToString() {
		return "DriveSpeed: speed = " + getSpeed() + " direction = " + getDirection();
	}

}
