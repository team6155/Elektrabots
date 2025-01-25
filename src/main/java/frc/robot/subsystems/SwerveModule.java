// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Constants.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {
  private SparkMax motorDrive;
  private SparkMax motorTurn;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  
  public SwerveModule(int TurnmotorID, int DrivemotorID) {
    motorTurn = new SparkMax(TurnmotorID, MotorType.kBrushless);
    motorDrive = new SparkMax(DrivemotorID, MotorType.kBrushless);
    driveController = motorDrive.getClosedLoopController();
    turnController = motorTurn.getClosedLoopController();
  }
  
  public void run(SwerveModuleState state){
    driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    turnController.setReference(state.angle.getRadians(), ControlType.kPosition);
  }
}