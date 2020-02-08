package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //TODO: Update port numbers
  // Cameras
  public static final int FRONT_CAMERA = 1;
  public static final int BACK_CAMERA = 0;

  // Motors
  public static final int FRONT_RIGHT_WHEEL = 1;
  public static final int FRONT_LEFT_WHEEL = 2;
  public static final int MIDDLE_RIGHT_WHEEL = -1;
  public static final int MIDDLE_LEFT_WHEEL = -1;
  public static final int REAR_RIGHT_WHEEL = 3;
  public static final int REAR_LEFT_WHEEL = 4;
  public static final int BELT_MOTOR = 5;
  public static final int WINCH_MOTOR = -1;
  public static final int CONTROL_PANEL_MOTOR = -1;

  // Pneumatics
  public static final int LEFT_SOLENOID_FORWARD = 0;
  public static final int LEFT_SOLENOID_BACKWARD = 1;
  public static final int MIDDLE_SOLENOID_FORWARD = 4;
  public static final int MIDDLE_SOLENOID_BACKWARD = 5;
  public static final int RIGHT_SOLENOID_FORWARD = 2;
  public static final int RIGHT_SOLENOID_BACKWARD = 3;
  public static final int CONTROL_PANEL_FORWARD = -1;
  public static final int CONTROL_PANEL_BACKWARD = -1;

  // Controllers
  public static final int DRIVER_GAMEPAD = 0;
  public static final int OPERATOR_GAMEPAD = 1;

  // Controller Buttons
  public static final int A_BUTTON = 1;
  public static final int B_BUTTON = 2;
  public static final int X_BUTTON = 3;
  public static final int Y_BUTTON = 4;
  public static final int LEFT_BUMPER = 5;
  public static final int RIGHT_BUMPER = 6;
  public static final int BACK_BUTTON = 7;
  public static final int START_BUTTON = 8;
  public static final int LEFT_STICK_PUSH = 9;
  public static final int RIGHT_STICK_PUSH = 10;

  // Controller Axis
  public static final int LEFT_STICK_X = 0;
  public static final int LEFT_STICK_Y = 1;
  public static final int LEFT_TRIGGER = 2;
  public static final int RIGHT_TRIGGER = 3;
  public static final int RIGHT_STICK_X = 4;
  public static final int RIGHT_STICK_Y = 5;
}
