/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Command that extends the pneumatic pusher on the robot.
 */
public class ExtendPusher extends InstantCommand {
  /**
   * Constructor for ExtendPusher Command.
   */
  public ExtendPusher() {
    super();
    requires(Robot.pneumatics);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  /**
   * The initialize method is called the first time this Command is run after
   * being started.
   * <p>
   * Call the pneumatics subsystem's extend method.
   */
  @Override
  protected void initialize() {
    Robot.pneumatics.extend();
  }

}
