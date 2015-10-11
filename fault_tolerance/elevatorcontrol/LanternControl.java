/* 
 * 18649 Fall 2014
 * Group 4 - Zhu Zhang (zhuzhang), Hua Liu (hual1), Howard Lee (hweekeul)
 */

package simulator.elevatorcontrol;

//Time and framework
import jSimPack.SimTime;
import simulator.framework.Controller;
import simulator.framework.Direction;
import simulator.framework.Hallway;
import simulator.framework.ReplicationComputer;
import simulator.elevatorcontrol.MessageDictionary;

//Can Mailbox
import simulator.payloads.CanMailbox;
import simulator.payloads.CanMailbox.ReadableCanMailbox;

//translators for network message
import simulator.elevatorcontrol.DesiredFloorCanPayloadTranslator;

//physical state message for CarLantern[d].
import simulator.payloads.CarLanternPayload;
import simulator.payloads.CarLanternPayload.WriteableCarLanternPayload;

/*
 * The LanternControl sends out signal such as mLanternControl to control the behavior of the Lanterns.
 * 
 * 
 * @author Howard Lee
 */
public class LanternControl extends Controller {
    /*****************************************************************************************
     * Declarations
     * **************************************************************************************/
    
    //local physical state
    private WriteableCarLanternPayload CarLantern;

    //network interference
    //received message of whether it arrived at certain floor.
    //translator for the at floor message
    private Utility.AtFloorArray mAtFloor;
    
    // received desired floor message
    private ReadableCanMailbox networkDesiredFloor;
    // translator for the desired floor message
    private DesiredFloorCanPayloadTranslator mDesiredFloor;

    private Utility.DoorClosedArray mDoorClosedFront;
    private Utility.DoorClosedArray mDoorClosedBack;

   //these variables keep track of which instance this is
   private final SimTime period;
   private final Direction direction;
   
   //enumerate states
   private enum State {
	STATE_LANTERN_OFF,
	STATE_LANTERN_ON
   }
   //state variable initialized to the initial state all lanterns off.
   private State state = State.STATE_LANTERN_OFF;

   /*
    * The arguments listed in the .cf configuration file should match the order and type given here.
    *
    * For your elevator controllers, you should make sure that the constructor matches the method
    * signatures in ControllerBuilder.makeAll().
    */

   public LanternControl(Direction direction, SimTime period, boolean verbose){
       //call to the controller superclass constructor is required
       super("LanternControl" + ReplicationComputer.makeReplicationString(direction), verbose);

       //stored the constructor arguments in internal state
       this.period = period;
       this.direction = direction;
       this.verbose = verbose;
       /*
        * The log() method is inherited from the Controller class.  It takes an array of objects which
        * will be converted to strings and concatenated only if the log message is actually written.
        *
        * For performance reasons, call with comma- separated lists, e.g.:
        *     log("object=", object);
        * Do NOT call with concatenated objects like:
        *     log("object=" + object);
        *
        */
       log("Created LanternControl with period = ", this.period);

       //initialize physical state
       //create a payload object using factory method in DoorMotorPayload.
      CarLantern = CarLanternPayload.getWriteablePayload(this.direction);
      // register the payload with the physical interface, periodically sending message.
      physicalInterface.sendTimeTriggered(CarLantern, this.period);


      //initialize network interface
      //initialize network interface for 'AtFloor'
      mAtFloor = new Utility.AtFloorArray(canInterface);      
      //initialize network interface for 'desired floor'
      networkDesiredFloor = CanMailbox.getReadableCanMailbox(MessageDictionary.DESIRED_FLOOR_CAN_ID);
      mDesiredFloor = new DesiredFloorCanPayloadTranslator(networkDesiredFloor);
      canInterface.registerTimeTriggered(networkDesiredFloor);
      //initialize network interface for 'doorClosed'
      mDoorClosedFront = new Utility.DoorClosedArray(Hallway.FRONT, canInterface);
      mDoorClosedBack = new Utility.DoorClosedArray(Hallway.BACK, canInterface);

      timer.start(period);
} 

public void timerExpired(Object callbackData){
    State newState = state;
    switch(state) {
	case STATE_LANTERN_OFF:
		//Sets Lantern off;
		CarLantern.set(false);

		//Transition to STATE_LANTERN_ON; 
		//If the car is at the floor and the door is not closed and the desired direction is up.
		//#transition 'LCT1'
		if(mAtFloor.getCurrentFloor() != MessageDictionary.NONE &&
		  (mDoorClosedFront.getBothClosed() == false || mDoorClosedBack.getBothClosed() == false) && direction == mDesiredFloor.getDirection())
			newState = State.STATE_LANTERN_ON;
		break;
	case STATE_LANTERN_ON:
		//Sets Lantern to be true.
		CarLantern.set(true);
		
		//Transition to STATE_LANTERN_OFF;
		//If the car is not at the floor or all doors are closed
		//#transition 'LCT2'
		if(mAtFloor.getCurrentFloor() == MessageDictionary.NONE
			|| (mDoorClosedFront.getBothClosed() == true && mDoorClosedBack.getBothClosed() == true) || direction != mDesiredFloor.getDirection())
			newState = State.STATE_LANTERN_OFF;
		break;
	default:
		throw new RuntimeException("State " + state + " was not recognized.");
    }
    //log the results of this iteration
    if (state == newState){
    	log("remains in state: ",state);
    } else {
    	log("Transition:",state,"->",newState);
    }
    
    //update the state variable
    state = newState;
    
    //report the current state
    setState(STATE_KEY,newState.toString());
    
    timer.start(period);
    
}
}