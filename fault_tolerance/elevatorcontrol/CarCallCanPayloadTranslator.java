package simulator.elevatorcontrol;

import simulator.payloads.CanMailbox.ReadableCanMailbox;
import simulator.payloads.CanMailbox.WriteableCanMailbox;

// TODO: Should be used in Dispatcher, but not necessary in Project 6.
public class CarCallCanPayloadTranslator extends EfficientBooleanCanPayloadTranslator {

	public CarCallCanPayloadTranslator(WriteableCanMailbox payload) {
		super(payload);
	}
	
	public CarCallCanPayloadTranslator(ReadableCanMailbox payload) {
		super(payload);
	}
}