// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

/** The command responsible for driving the robot autonomously. */
public class AutonomousDrive extends CommandBase {
  private final int DRIVE_TIME = 1;
  private final double SPEED = 1;

  private final Drivetrain DRIVETRAIN;
  private final Timer TIMER;

  /**
   * Creates a new Autonomous command.
   * @param drivetrain The robot's drivetrain subsystem.
   */
  public AutonomousDrive(Drivetrain drivetrain, Timer timer) {
    TIMER = timer;
    DRIVETRAIN = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TIMER.reset();
    TIMER.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DRIVETRAIN.drive(SPEED, 0, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DRIVETRAIN.drive(0, 0, 0);
    TIMER.stop();
    TIMER.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return TIMER.get() > DRIVE_TIME;
  }
}