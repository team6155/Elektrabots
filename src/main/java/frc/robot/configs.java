// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.subsystems.SwerveModule;

import com.revrobotics.spark.config.SparkMaxConfig;

/** Add your docs here. */
public class configs {
    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig(); 
    public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();
    public static final SparkMaxConfig hangerConfig = new SparkMaxConfig();
    static{
        double drivingFactor = SwerveModuleConstants.kWheelDiameterMeters * Math.PI
                / SwerveModuleConstants.kDrivingMotorReduction;
        double turningFactor = 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / SwerveModuleConstants.kDriveWheelFreeSpeedRps;
        //TODO: set factors
        double elevatorFactor = 0; //meters
        double intakeFactor = 0; //radians
        double hangerFactor = 0; //radians

        drivingConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        drivingConfig.encoder
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor / 60.0);
        drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.04, 0, 0)
            .velocityFF(drivingVelocityFeedForward)
            .outputRange(-1, 1);

        turningConfig
            .idleMode(IdleMode.kBrake) // stopping if not recieving inputs
            .smartCurrentLimit(20);
        turningConfig.absoluteEncoder
            .inverted(true)
            .positionConversionFactor(turningFactor) // radians
            .velocityConversionFactor(turningFactor / 60.0); // radians per seco
        turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1, 0, 0)
            .outputRange(-1, 1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);

        elevatorConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);//TODO; check limit
        elevatorConfig.encoder
            .positionConversionFactor(elevatorFactor)
            .velocityConversionFactor(elevatorFactor / 60.0);
        elevatorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.04, 0, 0)
            .outputRange(-1, 1);

        intakeConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        intakeConfig.encoder
            .positionConversionFactor(intakeFactor)
            .velocityConversionFactor(intakeFactor/ 60.0);
        intakeConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(.04,0,0)
            .outputRange(-1, 1);

        hangerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50);
        hangerConfig.encoder
            .positionConversionFactor(hangerFactor)
            .velocityConversionFactor(hangerFactor/ 60.0);
        hangerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(.04,0,0)
            .outputRange(-1, 1);
    }
}
