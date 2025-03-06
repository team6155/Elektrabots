// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HangerConstants;
import frc.robot.configs;

public class Hanger extends SubsystemBase {
  private SparkMax motor;
  private AbsoluteEncoder encoder;
  private SparkClosedLoopController PIDcontroller;
  
  public Hanger() {
    motor = new SparkMax(HangerConstants.motorID, MotorType.kBrushless);
    encoder = motor.getAbsoluteEncoder();
    PIDcontroller = motor.getClosedLoopController();
    motor.configure(configs.hangerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void run(double speed){
    if(speed > 0){
      if (encoder.getPosition() > (HangerConstants.upperLimit - ((HangerConstants.upperLimit - HangerConstants.lowerLimit)*HangerConstants.threshold))){
        speed*=(HangerConstants.upperLimit-encoder.getPosition())/((HangerConstants.upperLimit - HangerConstants.lowerLimit)*HangerConstants.threshold);
      }
    }
    if(speed<0){
      if (encoder.getPosition() < (HangerConstants.lowerLimit + ((HangerConstants.upperLimit-HangerConstants.lowerLimit)*HangerConstants.threshold))){
        speed*=(encoder.getPosition()-HangerConstants.lowerLimit)/((HangerConstants.upperLimit - HangerConstants.lowerLimit)*HangerConstants.threshold) ;
      }
    }
    PIDcontroller.setReference(speed, ControlType.kVelocity);
  }

  public void run(double downSpeed, double upSpeed){
    run(downSpeed-upSpeed);
  }

  public void set(double angle){
    angle = MathUtil.clamp(angle, HangerConstants.lowerLimit, HangerConstants.upperLimit);
    PIDcontroller.setReference(angle, ControlType.kPosition);
  }

  public double getAngle(){
    return encoder.getPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hanger Angle", encoder.getPosition());
    // This method will be called once per scheduler run
  }
}
