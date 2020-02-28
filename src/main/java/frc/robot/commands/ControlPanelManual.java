/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.controllers.Gamepad;

public class ControlPanelManual extends InstantCommand {
  double speed;
  
  public ControlPanelManual(double speed) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.speed = speed;
    requires(Robot.controlPanel);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.controlPanel.runMotor(speed);
  }
}
