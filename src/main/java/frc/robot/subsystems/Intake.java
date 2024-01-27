// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.MotorCommutation;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  MotorController motor;

  /** Creates a new Intake. */
  public Intake() {
    motor = new CANSparkMax(IntakeConstants.MOTOR_CHANNEL, MotorType.kBrushless);
  }

public void run(double speed){
  motor.set(speed);

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
