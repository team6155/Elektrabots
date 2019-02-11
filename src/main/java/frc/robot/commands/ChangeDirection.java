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
