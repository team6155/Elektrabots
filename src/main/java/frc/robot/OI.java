package frc.robot;

import frc.robot.controllers.Gamepad;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  private final Gamepad controller;

  /**
   * Constructor for the OI class.
   */
  public OI() {
    controller = new Gamepad(RobotMap.GAMEPAD);
  }

  /**
   * Get the xbox controller.
   * 
   * @return The xbox controller.
   */
  public Gamepad getController() {
    return controller;
  }
}
