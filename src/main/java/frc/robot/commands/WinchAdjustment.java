// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class WinchAdjustment extends Command {
  private final Supplier<Double> leftSpeed ;
  private final Supplier<Double> rightSpeed ;
  private final Arm arm;
  private final double limit = .5;
  /** Creates a new Arm2. */
  public WinchAdjustment(Arm arm, Supplier<Double> leftSpeed, Supplier<Double> rightSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed; 
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.run(leftSpeed.get()*limit, rightSpeed.get()*limit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
