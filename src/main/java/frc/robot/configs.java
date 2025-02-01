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
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig(); 
    static{
        double drivingFactor = SwerveModuleConstants.kWheelDiameterMeters * Math.PI
                / SwerveModuleConstants.kDrivingMotorReduction;
        double turningFactor = 2 * Math.PI;
        double drivingVelocityFeedForward = 1 / SwerveModuleConstants.kDriveWheelFreeSpeedRps;
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
            .idleMode(IdleMode.kBrake)
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
    }
}
