package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name.
 * This provides flexibility changing wiring, makes checking the wiring easier and significantly
 * reduces the number of magic numbers floating around.
 */
public class RobotMap {
  // TODO: Set port numbers correctly.
  // Motors
  public static final int FRONT_RIGHT_WHEEL = 9;
  public static final int FRONT_LEFT_WHEEL = 8;
  public static final int REAR_RIGHT_WHEEL = 7;
  public static final int REAR_LEFT_WHEEL = 6;
  public static final int BELT_MOTOR = 5;

  // Pneumatics
  public static final int LEFT_SOLENOID_FORWARD = -1;
  public static final int LEFT_SOLENOID_BACKWARD = -1;
  public static final int RIGHT_SOLENOID_FORWARD = -1;
  public static final int RIGHT_SOLENOID_BACKWARD = -1;

  // Controllers
  public static final int XBOX_CONTROLLER = 0;
}
