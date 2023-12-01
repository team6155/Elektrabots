// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wheels;

public class Move extends CommandBase {//TODO; Initialize Variables
  public Wheels wheels;
  public XboxController controller;

  /** Creates a new Move. */
  public Move() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO:initialize speed variable
    double speed = controller.getLeftY(); 
    wheels.frontLeftWheel.set(speed);
    wheels.frontRightWheel.set(speed);
    wheels.rearLeftWheel.set(speed);
    wheels.rearRightWheel.set(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
