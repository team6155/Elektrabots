package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.controllers.Gamepad;

/**
 * Drive the robot according the the input from the xbox controller.
 */
public class Drive extends InstantCommand {
  Gamepad controller;

  /**
   * Constructor for Drive command.
   * 
   * @param controller Xbox Controller used to control the robot.
   */
  public Drive(Gamepad controller) {
    super();
    requires(Robot.driveTrain);
    this.controller = controller;
  }

  /**
   * The initialize method is called the first time this Command is run after
   * being started.
   * <p>
   * Call the drivetrain subsystem's drive method.
   */
  @Override
  protected void initialize() {
    double forwardsSpeed = normalizeInput(controller.getLeftStickY());
    double rotationSpeed = normalizeInput(controller.getRightStickY());

    Robot.driveTrain.drive(forwardsSpeed, rotationSpeed);
  }

  /**
   * Normalize the input from the controller to give finer control at low speeds
   * and to discard tiny movements of the stick.
   * <p>
   * Square the input while keeping the positive or negative sign. If the squared
   * input is < 0.02, discard it.
   * 
   * @param stickInput Operator input from the xbox controller joystick.
   * @return Normalized speed.
   */
  private double normalizeInput(double stickInput) {
    int sign = stickInput >= 0 ? 1 : -1;

    stickInput *= stickInput;
    if (stickInput < 0.02)
      stickInput = 0;
    stickInput *= sign;

    return stickInput;
  }
}
