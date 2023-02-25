// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GrabberWrist;

public class ControlWrist extends CommandBase {
  private final GrabberWrist WRIST;
  private final Supplier<Double> SPEED;

  /** Creates a new ControlWrist. */
  public ControlWrist(GrabberWrist wrist, Supplier<Double> speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
    WRIST = wrist;
    SPEED = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    WRIST.run(SPEED.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    WRIST.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
