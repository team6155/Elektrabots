/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class UpdateSensors extends InstantCommand {
  /**
   * Add your docs here.
   */
  public UpdateSensors() {
    super();
    requires(Robot.driveTrain);
    requires(Robot.controlPanel);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    SmartDashboard.putNumber("Gyro Angle", Robot.driveTrain.readGyro());
    SmartDashboard.putString("Color", Robot.controlPanel.readColor().toString());
  }
}
