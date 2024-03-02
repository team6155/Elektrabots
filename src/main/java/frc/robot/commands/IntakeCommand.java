// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
  Intake intake;
  private final Supplier<Double> motorSpeed; 
  private final  double limit = .5;

  /** Creates a new Intake. */
  public IntakeCommand(Intake intake, Supplier<Double> motorSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    this.motorSpeed = motorSpeed; 
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO: Write execute function
    intake.run(motorSpeed.get()*limit);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.run(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
