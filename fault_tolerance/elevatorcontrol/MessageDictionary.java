/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package simulator.elevatorcontrol;

import jSimPack.SimTime;
import jSimPack.SimTime.SimTimeUnit;

/**
 * This class defines constants for CAN IDs that are used throughout the simulator.
 *
 * The default values will work for early projects.  Later on, you will modify these
 * values when you create a network schedule.
 *
 * @author justinr2
 */
public class MessageDictionary {

    //controller periods
    public final static int NONE = -1;
    public final static SimTime HALL_BUTTON_CONTROL_PERIOD = new SimTime(100, SimTimeUnit.MILLISECOND);
    public final static SimTime CAR_BUTTON_CONTROL_PERIOD = new SimTime(100, SimTimeUnit.MILLISECOND);
    public final static SimTime LANTERN_CONTROL_PERIOD = new SimTime(200, SimTimeUnit.MILLISECOND);
    public final static SimTime CAR_POSITION_CONTROL_PERIOD = new SimTime(50, SimTimeUnit.MILLISECOND);
    public final static SimTime DISPATCHER_PERIOD = new SimTime(50, SimTimeUnit.MILLISECOND);
    public final static SimTime DOOR_CONTROL_PERIOD = new SimTime(10, SimTimeUnit.MILLISECOND);
    public final static SimTime DRIVE_CONTROL_PERIOD = new SimTime(10, SimTimeUnit.MILLISECOND);
    
    //controller message IDs
    public final static int DRIVE_SPEED_CAN_ID =                0x0BE8B500;
    public final static int DRIVE_COMMAND_CAN_ID =              0x0F0AB500;
    public final static int DESIRED_DWELL_BASE_CAN_ID =         0x0E40B600;
    public final static int DESIRED_FLOOR_CAN_ID =              0x0DDCB600;
    public final static int CAR_POSITION_CAN_ID =               0x0F0DB700;
    public final static int DOOR_MOTOR_COMMAND_BASE_CAN_ID =    0x0CB11500;
    public final static int HALL_CALL_BASE_CAN_ID =             0x0F08B900;
    public final static int HALL_LIGHT_BASE_CAN_ID =            0x0F10B900;
    public final static int CAR_CALL_BASE_CAN_ID =              0x0F6DBA00;
    public final static int CAR_LIGHT_BASE_CAN_ID =             0x0F12BA00;
    public final static int CAR_LANTERN_BASE_CAN_ID =           0x0F13BB00;
    
    //module message IDs
    public final static int AT_FLOOR_BASE_CAN_ID =              0x0D782800;
    public final static int CAR_LEVEL_POSITION_CAN_ID =         0x0EA43C00;
    public final static int CAR_WEIGHT_CAN_ID =                 0x0F707800;
    public final static int CAR_WEIGHT_ALARM_CAN_ID =           0x0E408C00;
    public final static int DOOR_OPEN_SENSOR_BASE_CAN_ID =      0x0F6FA000;
    public final static int DOOR_CLOSED_SENSOR_BASE_CAN_ID =    0x0F6C5000;
    public final static int DOOR_REVERSAL_SENSOR_BASE_CAN_ID =  0x0C4C6400;
    public final static int HOISTWAY_LIMIT_BASE_CAN_ID =        0x0F6EB400;
    public final static int EMERGENCY_BRAKE_CAN_ID =            0x0D141E00;
    public final static int LEVELING_BASE_CAN_ID =              0x0CB01400;
    
}
