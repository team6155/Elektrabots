// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

/** The command responsible for shooting the ball in autonomous mode. */
public class AutonomousShoot extends CommandBase {
  private final int SHOOTING_TIME = 3;
  private final double SPEED = 1;

  private final Conveyor CONVEYOR;
  private final Timer TIMER;

  /**
   * Creates a new AutonomousShoot command.
   * @param drivetrain The robot's drivetrain subsystem.
   */
  public AutonomousShoot(Conveyor conveyor, Timer timer) {
    TIMER = timer;
    CONVEYOR = conveyor;
    addRequirements(conveyor);
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
    CONVEYOR.runBeltMotor(SPEED);
    CONVEYOR.runShootingMotor(SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CONVEYOR.runBeltMotor(0);
    CONVEYOR.runShootingMotor(0);
    TIMER.stop();
    TIMER.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return TIMER.get() < SHOOTING_TIME;
  }
}