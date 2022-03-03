// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Drivetrain;


/**
 * This command group contains all of the commands used in autonomous mode. They will execute in sequential order.
 * <p>
 * The robot will first shoot the preloaded ball and then drive forwards.
 */
public class Autonomous extends SequentialCommandGroup {

  /** Creates a new Autonomous command group. */
  public Autonomous(Drivetrain drivetrain, Conveyor conveyor) {
    addCommands(new AutonomousPositionBall(conveyor), new AutonomousShoot(conveyor), new AutonomousDrive(drivetrain));
  }
}
