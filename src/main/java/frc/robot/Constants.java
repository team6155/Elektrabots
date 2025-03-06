// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort= 1;
    public static final double deadbandValue = .01;
  }
  public static class HangerConstants{
    public static final int motorID = 9;
    public static final double lowerLimit= .333;
    public static final double upperLimit = Math.PI;
    public static final double threshold = .1;
  }
  public static class IntakeConstants{
    public static final int motorID = 10;
    public static final double lowerLimit = .11;
    public static final double upperLimit = Math.PI;
    public static final double threshold = .1;
  }
  public static class ElevatorConstants{
    public static final int motorID = 11;
    public static final double lowerLimit= 0.9;
    public static final double upperLimit = 2;
    public static final double threshold = .1;
    public static final double[] stages = {0, .7, 1.3, 2};
    public static final int topLimitSwitch = 0;
    public static final int bottomLimitSwitch = 1;
  }
  public static class SwerveModuleConstants{
    public static final int FlmotorTurn = 1;
    public static final int FlmotorDrive = 2;
    public static final int FrmotorTurn = 7;
    public static final int FrmotorDrive = 8;
    public static final int BlmotorTurn = 3;
    public static final int BlmotorDrive= 4;
    public static final int BrmotorTurn = 5;
    public static final int BrmotorDrive = 6;
    public static final double drivetrainWidth = 0.5969; //meters
    public static final double drivetrainLength = 0.7239; //meters
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
      new Translation2d(drivetrainWidth/2, drivetrainLength/2),
      new Translation2d(drivetrainWidth/2,-drivetrainLength/2),
      new Translation2d(-drivetrainWidth/2,drivetrainLength/2),
      new Translation2d(-drivetrainWidth/2,-drivetrainLength/2)
    );
    public static final int kDrivingMotorPinionTeeth = 13;
    public static final double kFreeSpeedRpm =5676;
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60 ;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = (45.0*22) / (kDrivingMotorPinionTeeth*15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters);
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;

    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
  }
  public static class VisionConstants{
    public static final double limelightHeight = 0; //distance from center of the lense to the floor
    public static final double limelightAngle = 0; //degrees rotated from perfectly vertical
    public static final int coralCamera = 0;
    public static final int hangerCamera = 1;
  }
}

