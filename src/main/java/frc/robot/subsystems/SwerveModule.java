// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Constants.SwerveModuleConstants;

/** Add your docs here. */
public class SwerveModule {
    private SparkMax motorTurn;
    private SparkMax motorDrive;

    public SwerveModule(int TurnmotorID, int DrivemotorID) {
    motorTurn = new SparkMax(TurnmotorID, MotorType.kBrushless);
    motorDrive = new SparkMax(DrivemotorID, MotorType.kBrushless);  
    }

    public void run(double speed){
    motorTurn.set(speed);
    motorDrive.set(speed);
  }
}