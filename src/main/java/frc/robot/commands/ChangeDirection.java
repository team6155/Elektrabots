package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Change the direction the robot faces from the perspective of the driver.
 */
public class ChangeDirection extends InstantCommand {

  /**
   * Constructor for the ChangeDirection command.
   */
  public ChangeDirection() {
    super();
    requires(Robot.driveTrain);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.driveTrain.changeDirection();
  }

}
