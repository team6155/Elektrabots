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
 * Command that retracts the pneumatic pusher on the robot.
 */
public class RetractPusher extends InstantCommand {
  /**
   * Constructor for RetractPusher Command.
   */
  public RetractPusher() {
    super();
    requires(Robot.pneumatics);
  }

  /**
   * The initialize method is called the first time this Command is run after
   * being started.
   * <p>
   * Call the pneumatics subsystem's retract method.
   */
  @Override
  protected void initialize() {
    Robot.pneumatics.retract();
  }

}
