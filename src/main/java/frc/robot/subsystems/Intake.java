// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  MotorController motor1;
  MotorController motor2;
  MotorController motor3;


  /** Creates a new Intake. */
  public Intake() {
    motor1 = new CANSparkMax(IntakeConstants.MOTOR_CHANNEL_1, MotorType.kBrushless);
    motor2 = new CANSparkMax(IntakeConstants.MOTOR_CHANNEL_1, MotorType.kBrushless);
    motor3 = new CANSparkMax(IntakeConstants.MOTOR_CHANNEL_1, MotorType.kBrushless);
  }

public void run(double speed){
  motor.set(speed);

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
