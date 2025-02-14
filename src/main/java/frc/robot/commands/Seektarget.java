// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class Seektarget extends Command {
  private Vision vision; 
  private SwerveDrive drive;
  /** Creates a new Seektarget. */
  public Seektarget(Vision vision, SwerveDrive drive) {
    this.vision = vision;
    this.drive = drive;
    addRequirements(vision, drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //look for target
    drive.Drive(0, 0, 1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.Drive(0, 0, 0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return vision.foundAprilTag();
  }
}
