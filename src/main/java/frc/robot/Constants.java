// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
  }
  public static class HangerConstants{
    public static final int motorID = -1 ;
  }
  public static class IntakeConstants{
    public static final int motorID = -1;
  }
  public static class ElevatorConstants{
    public static final int motorID = -1;
  }
  public static class SwerveModuleConstants{
    //TODO: set canIDS 
    public static final int FlmotorTurn = -1;
    public static final int FlmotorDrive = -1;
    public static final int FrmotorTurn = -1;
    public static final int FrmotorDrive = -1;
    public static final int BlmotorTurn = -1;
    public static final int BlmotorDrive= -1;
    public static final int BrmotorTurn = -1;
    public static final int BrmotorDrive = -1;
    //TODO: get wheel measurements
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
      null, null, null, null
    );
    public static final double drivetrainWidth = 0.5969; //meters
    public static final double drivetrainLength = 0.7239; //meters
    public static final int kDrivingMotorPinionTeeth = 13;
    public static final double kFreeSpeedRpm =5676;
    public static final double kDrivingMotorFreeSpeedRps = kFreeSpeedRpm / 60 ;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    public static final double kDrivingMotorReduction = (45.0*22) / (kDrivingMotorPinionTeeth*15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters);
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
  }

}
