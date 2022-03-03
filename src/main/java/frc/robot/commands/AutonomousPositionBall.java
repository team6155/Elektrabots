// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

/** The command responsible for preparing the ball to be shot in autonomous mode. */
public class AutonomousPositionBall extends CommandBase {
  private final int POSITIONING_TIME = 1;
  private final double SPEED = 1;

  private final Conveyor CONVEYOR;
  private final Timer TIMER;

  /**
   * Creates a new AutonomousPositionBall command.
   * @param conveyor The robot's conveyor subsystem.
   */
  public AutonomousPositionBall(Conveyor conveyor) {
    TIMER = new Timer();
    CONVEYOR = conveyor;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TIMER.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    CONVEYOR.runIntakeMotor(SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CONVEYOR.runIntakeMotor(0);
    TIMER.stop();
    TIMER.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return TIMER.get() < POSITIONING_TIME;
  }
}