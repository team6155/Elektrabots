// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class Arm2 extends Command {
  private final Supplier<Double> speed1 ;
  private final Supplier<Double> speed2 ;
  private final Arm2 arm2;
  /** Creates a new Arm2. */
  public Arm2(Arm2 arm2, Supplier<Double> speed1, Supplier<Double> speed2) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm2 = arm2;
    this.speed1 = speed1;
    this.speed2 = speed2; 
    addRequirements(arm2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
