// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drivetarget extends Command {
  private Vision vision;
  private SwerveDrive drive;
  private double proportionateConstantA = .035;
  private double proportionateConstantR = .1 ;
  /** Creates a new Drivetarget. */
  public Drivetarget(Vision vision, SwerveDrive drive) {
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
    double rotSpeed = LimelightHelpers.getTX("")*proportionateConstantA;
    double ySpeed = -LimelightHelpers.getTY("")*proportionateConstantR; //assuming april tag is above the camera
    drive.Drive(0,ySpeed, rotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.Drive(0, 0, 0);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return LimelightHelpers.getTX("") ==0 && LimelightHelpers.getTY("")== 0;
  }
}
