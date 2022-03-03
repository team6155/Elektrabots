// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

/** The command responsible for shooting the ball in autonomous mode. */
public class AutonomousShoot extends CommandBase {
  Conveyor conveyor;
  Timer timer;
  int shootingTime = 1;
  double speed = 1;

  /**
   * Creates a new AutonomousShoot command.
   * @param drivetrain The robot's drivetrain subsystem.
   */
  public AutonomousShoot(Conveyor conveyor) {
    timer = new Timer();
    this.conveyor = conveyor;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyor.runShootingMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.runShootingMotor(0);
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() < shootingTime;
  }
}