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
