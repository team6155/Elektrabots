package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Change the direction the robot faces from the perspective of the driver.
 */
public class ChangeDirection extends InstantCommand {

  //TODO: Reactivate camera
  /**
   * Constructor for the ChangeDirection command.
   */
  public ChangeDirection() {
    super();
    requires(Robot.driveTrain);
    //requires(Robot.camera);
  }

  /**
   * The initialize method is called the first time this Command is run after
   * being started.
   * <p>
   * Call the drivetrain subsystem's changeDirection method.
   * Call the camera subsystem's changeDirection method.
   */
  @Override
  protected void initialize() {
    Robot.driveTrain.changeDirection();
    //Robot.camera.changeDirection();
  }

}
