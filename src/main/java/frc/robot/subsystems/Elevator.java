// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private SparkMax motor;
  private RelativeEncoder encoder;
  private SparkClosedLoopController PIDcontroller;
  private int curStage = 0;
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;

  /** Creates a new Elevator. */
  public Elevator() {
    motor =  new SparkMax(ElevatorConstants.motorID, MotorType.kBrushless);
    encoder = motor.getEncoder();
    PIDcontroller = motor.getClosedLoopController();
    motor.configure(configs.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    topLimitSwitch = new DigitalInput(ElevatorConstants.topLimitSwitch);
    bottomLimitSwitch = new DigitalInput(ElevatorConstants.bottomLimitSwitch);
    encoder.setPosition(.90);
  }

  public void run (double speed){
    if(speed > 0){
      if (getHeight() > (ElevatorConstants.upperLimit - ((ElevatorConstants.upperLimit - ElevatorConstants.lowerLimit)*ElevatorConstants.threshold))){
        speed*=(ElevatorConstants.upperLimit-getHeight())/((ElevatorConstants.upperLimit - ElevatorConstants.lowerLimit)*ElevatorConstants.threshold);
      }
    }
    if(speed<0){
      if (getHeight() < (ElevatorConstants.lowerLimit + ((ElevatorConstants.upperLimit-ElevatorConstants.lowerLimit)*ElevatorConstants.threshold))){
        speed*=(getHeight()-ElevatorConstants.lowerLimit)/((ElevatorConstants.upperLimit - ElevatorConstants.lowerLimit)*ElevatorConstants.threshold) ;
      }
    }
    PIDcontroller.setReference(speed, ControlType.kVelocity);
  }

  public void set(double height){
    height = MathUtil.clamp(height, ElevatorConstants.lowerLimit, ElevatorConstants.upperLimit);
    PIDcontroller.setReference(height, ControlType.kPosition);
  }
  public double getHeight(){
    return encoder.getPosition();
  }

  public int changeStage(boolean increasing){
    if (increasing){
      curStage = Math.min(curStage + 1, 3);
    }
    else{
      curStage = Math.max(curStage - 1, 0);
    }
    return curStage;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Height", encoder.getPosition());
    // This method will be called once per scheduler run
  }
}
