// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

/** Command for switching the direction the robot considers forwards. */
public class ChangeRobotDirection extends InstantCommand {
  private final Drivetrain DRIVETRAIN;

  /**
   * Creates a new ChangeRobotDirection command.
   * @param drivetrain The robot's drivetrain subsystem.
   */
  public ChangeRobotDirection(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    DRIVETRAIN = drivetrain;
    addRequirements(drivetrain);
  }

  /**
   * The initial subroutine of a command. Called once when the command is initially scheduled.
   * <p>
   * Call the subsystem's method for changing direction.
   */
  @Override
  public void initialize() {
    DRIVETRAIN.changeDirection();
  }
}
