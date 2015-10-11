package simulator.elevatorcontrol;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

public class HallCallCanPayloadTranslator extends EfficientBooleanCanPayloadTranslator {

	public HallCallCanPayloadTranslator(ReadableCanMailbox payload) {
		super(payload);
	}

	public HallCallCanPayloadTranslator(WriteableCanMailbox payload) {
		super(payload);
	}
}
