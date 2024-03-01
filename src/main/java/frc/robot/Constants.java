// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants extends Object {
    // Computer USB ports
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    //TODO: update channel number 
    public static final class IntakeConstants {
        public static final int MOTOR_CHANNEL = 12;
    }

    public static final class ShooterConstants {
        public static final int MOTOR_CHANNEL_RIGHT = 10;
        public static final int MOTOR_CHANNEL_LEFT = 11;
    }
    public static final class ArmConstants{
        public static final int LEFT_MOTOR_CHANNEL = 13;
        public static final int RIGHT_MOTOR_CHANNEL = 14;

    }

    public static final class InputConstants {
        public static final double DEADBAND = 0.05;
    }

    public static final class DriveConstants {
        public static final int FRONT_LEFT_DRIVING_MOTOR_PORT = 1;
        public static final int FRONT_RIGHT_DRIVING_MOTOR_PORT = 7;
        public static final int REAR_LEFT_DRIVING_MOTOR_PORT = 3;
        public static final int REAR_RIGHT_DRIVING_MOTOR_PORT = 5;

        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 2;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 8;
        public static final int REAR_LEFT_TURNING_MOTOR_PORT = 4;
        public static final int REAR_RIGHT_TURNING_MOTOR_PORT = 6;
        
        public static final double FRONT_LEFT_ANGULAR_OFFSET = -Math.PI / 2;
        public static final double FRONT_RIGHT_ANGULAR_OFFSET = 0;
        public static final double REAR_LEFT_ANGULAR_OFFSET = Math.PI;
        public static final double REAR_RIGHT_ANGULAR_OFFSET = Math.PI / 2;

        //TODO: Update dimensions to new chassis
        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = .61;
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = .737;
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, TRACK_WIDTH_METERS / 2),
                new Translation2d(-WHEEL_BASE_METERS / 2, -TRACK_WIDTH_METERS / 2)   
            );
        
        public static boolean GYRO_REVERSED = false;
        public static final double SPEED_RATIO = .5;
        public static final Rotation2d STARTING_ROTATION = new Rotation2d(Math.PI);
        public static final double DIRECTION_SLEW_RATE = 1.2;
        public static final double MAGNITUDE_SLEW_RATE = 1.8;
        public static final double ROTATIONAL_SLEW_RATE = 2.0;
        public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;
    }

    public static abstract class SwerveModuleConstants {
        public static final double DRIVING_P_VALUE = 0.04;
        public static final double DRIVING_I_VALUE = 0;
        public static final double DRIVING_D_VALUE = 0;

        public static final double TURNING_P_VALUE = 1;
        public static final double TURNING_I_VALUE = 0;
        public static final double TURNING_D_VALUE = 0;

        public static final double WHEEL_DIAMETER_METERS = .0762;
        public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;

        // The RevRobotics MAXSwerve Module we're using can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // Make sure this is accurate to our configuration.
        // The other 3 teeth numbers are fixed.
        public static final double DRIVING_MOTOR_PINION_TEETH = 13;
        public static final double DRIVING_MOTOR_BEVEL_GEAR_TEETH = 45;
        public static final double DRIVING_MOTOR_SPUR_GEAR_TEETH = 22;
        public static final double DRIVING_MOTOR_BEVEL_PINION_TEETH = 15;
        public static final double DRIVING_MOTOR_REDUCTION = (DRIVING_MOTOR_BEVEL_GEAR_TEETH * DRIVING_MOTOR_SPUR_GEAR_TEETH)
            / (DRIVING_MOTOR_PINION_TEETH * DRIVING_MOTOR_BEVEL_PINION_TEETH);
        /** Convert the encoder's units into meters. */
        public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
            / DRIVING_MOTOR_REDUCTION;

        public static final double DRIVING_MOTOR_ROTATIONS_PER_SECOND = 5676 / 60;
        public static final double DRIVING_MOTOR_METERS_PER_SECOND = DRIVING_MOTOR_ROTATIONS_PER_SECOND * WHEEL_CIRCUMFERENCE_METERS
            / DRIVING_MOTOR_REDUCTION;
        
        public static final double DRIVING_FF_VALUE = 1 / DRIVING_MOTOR_METERS_PER_SECOND;
        public static final double TURNING_FF_VALUE = 0;
        
        /** Convert the encoder's units into radians. */
        public static final double TURNING_ENCODER_POSITION_FACTOR = 2 * Math.PI;
        public static final boolean TURNING_ENCODER_INVERTED = true;
        
        public static final int DRIVING_MOTOR_CURRENT_LIMIT = 50;
        public static final int TURNING_MOTOR_CURRENT_LIMIT = 20;
    }
}
