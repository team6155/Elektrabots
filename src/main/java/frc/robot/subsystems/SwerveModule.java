// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.configs;
import frc.robot.Constants.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {
  private SparkMax motorDrive;
  private SparkMax motorTurn;
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;
  private final SparkClosedLoopController driveController;
  private final SparkClosedLoopController turnController;
  private final double chassisAngularOffset;
  
  public SwerveModule(int TurnmotorID, int DrivemotorID, double chassisAngularOffset) {
    motorTurn = new SparkMax(TurnmotorID, MotorType.kBrushless);
    motorDrive = new SparkMax(DrivemotorID, MotorType.kBrushless);
    drivingEncoder = motorDrive.getEncoder();
    turningEncoder = motorTurn.getAbsoluteEncoder();
    driveController = motorDrive.getClosedLoopController();
    turnController = motorTurn.getClosedLoopController();
    motorDrive.configure(configs.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorTurn.configure(configs.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.chassisAngularOffset = chassisAngularOffset ;
    drivingEncoder.setPosition(0);
  }
  
  public void run(SwerveModuleState state){
    state.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    state.optimize(new Rotation2d(turningEncoder.getPosition()));
    driveController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    turnController.setReference(state.angle.getRadians(), ControlType.kPosition);
  }
}