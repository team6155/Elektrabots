// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.configs;

public class Intake extends SubsystemBase {
  private SparkMax motor;
  private AbsoluteEncoder encoder;
  private SparkClosedLoopController PIDcontroller;


  public Intake() {
    motor = new SparkMax(IntakeConstants.motorID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    PIDcontroller = motor.getClosedLoopController();
    motor.configure(configs.intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void run(double speed){
    if(speed > 0){
      if (encoder.getPosition() > (IntakeConstants.upperLimit - ((IntakeConstants.upperLimit - IntakeConstants.lowerLimit)*IntakeConstants.threshold))){
        speed*=(IntakeConstants.upperLimit-encoder.getPosition())/((IntakeConstants.upperLimit - IntakeConstants.lowerLimit)*IntakeConstants.threshold);
      }
    }
    if(speed<0){
      if (encoder.getPosition() < (IntakeConstants.lowerLimit + ((IntakeConstants.upperLimit-IntakeConstants.lowerLimit)*IntakeConstants.threshold))){
        speed*=(encoder.getPosition()-IntakeConstants.lowerLimit)/((IntakeConstants.upperLimit - IntakeConstants.lowerLimit)*IntakeConstants.threshold) ;
      }
    }
    PIDcontroller.setReference(speed, ControlType.kVelocity);
  }

  public void set(double angle){
    angle = MathUtil.clamp(angle, IntakeConstants.lowerLimit, IntakeConstants.upperLimit);
    PIDcontroller.setReference(angle, ControlType.kPosition);
  }

  public double getAngle(){
    return encoder.getPosition();
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Angle", encoder.getPosition());
    // This method will be called once per scheduler run
  }
}
