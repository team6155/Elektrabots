// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveDrive extends SubsystemBase {
  private SwerveModule frwheel;
  private SwerveModule flwheel;
  private SwerveModule brwheel;
  private SwerveModule blwheel;

  public SwerveDrive() {
    frwheel = new SwerveModule(
        SwerveModuleConstants.FrmotorTurn, 
        SwerveModuleConstants.FrmotorDrive,
        SwerveModuleConstants.kFrontRightChassisAngularOffset
    );
    flwheel = new SwerveModule(
        SwerveModuleConstants.FlmotorTurn, 
        SwerveModuleConstants.FlmotorDrive,
        SwerveModuleConstants.kFrontLeftChassisAngularOffset
    );
    brwheel = new SwerveModule(
        SwerveModuleConstants.BrmotorTurn,
        SwerveModuleConstants.BrmotorDrive,
        SwerveModuleConstants.kBackRightChassisAngularOffset
    );
    blwheel = new SwerveModule(
        SwerveModuleConstants.BlmotorTurn, 
        SwerveModuleConstants.BlmotorDrive,
        SwerveModuleConstants.kBackLeftChassisAngularOffset
    );
  }

  public void setModuleStates(SwerveModuleState[] states){
    flwheel.run(states[0]);
    frwheel.run(states[1]);
    blwheel.run(states[2]);
    brwheel.run(states[3]);
  }

  public void Drive(double xSpeed, double ySpeed, double rotation){
    // TODO: convert speeds from decimal to meters per second and radians per second
    xSpeed*= SwerveModuleConstants.kMaxSpeedMetersPerSecond;
    ySpeed*= SwerveModuleConstants.kMaxSpeedMetersPerSecond;
    rotation*= SwerveModuleConstants.kMaxAngularSpeed;
    var states = SwerveModuleConstants.driveKinematics.toSwerveModuleStates(
      new ChassisSpeeds(ySpeed, xSpeed, rotation)
    );
    setModuleStates(states);
  }

  public void brake(){
    flwheel.run(new SwerveModuleState(0,Rotation2d.fromDegrees(45)));
    frwheel.run(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    blwheel.run( new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    brwheel.run(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
