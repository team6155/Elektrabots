package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Bring the robot to a complete stop.
 */
public class Stop extends InstantCommand {

  /**
   * Constructor for the Stop command.
   */
  public Stop() {
    super();
    requires(Robot.driveTrain);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.driveTrain.drive(0, 0, 0);
  }
}
