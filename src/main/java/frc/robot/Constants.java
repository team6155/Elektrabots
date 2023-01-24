// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
public final class Constants {
    // Computer USB ports
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final class InputConstants {
        public static final double DEADBAND = 0.05;
        public static final double ACCELERATION_RATE_LIMIT = 3;
    }

    // TODO: Correct motor channel numbers.
    public static final class DriveConstants {
        public static final int FRONT_LEFT_DRIVING_MOTOR_PORT = -1;
        public static final int FRONT_RIGHT_DRIVING_MOTOR_PORT = -1;
        public static final int REAR_LEFT_DRIVING_MOTOR_PORT = -1;
        public static final int REAR_RIGHT_DRIVING_MOTOR_PORT = -1;

        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = -1;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = -1;
        public static final int REAR_LEFT_TURNING_MOTOR_PORT = -1;
        public static final int REAR_RIGHT_TURNING_MOTOR_PORT = -1;
        
        public static final boolean FRONT_LEFT_DRIVING_MOTOR_REVERSED = false;
        public static final boolean FRONT_RIGHT_DRIVING_MOTOR_REVERSED = false;
        public static final boolean REAR_LEFT_DRIVING_MOTOR_REVERSED = false;
        public static final boolean REAR_RIGHT_DRIVING_MOTOR_REVERSED = false;
        
        public static final boolean FRONT_LEFT_TURNING_MOTOR_REVERSED = false;
        public static final boolean FRONT_RIGHT_TURNING_MOTOR_REVERSED = false;
        public static final boolean REAR_LEFT_TURNING_MOTOR_REVERSED = false;
        public static final boolean REAR_RIGHT_TURNING_MOTOR_REVERSED = false;

        public static final int[] FRONT_LEFT_DRIVING_ENCODER_PORTS = new int[] {-1, -1};
        public static final int[] FRONT_RIGHT_DRIVING_ENCODER_PORTS = new int[] {-1, -1};
        public static final int[] REAR_LEFT_DRIVING_ENCODER_PORTS = new int[] {-1, -1};
        public static final int[] REAR_RIGHT_DRIVING_ENCODER_PORTS = new int[] {-1, -1};

        public static final int[] FRONT_LEFT_TURNING_ENCODER_PORTS = new int[] {-1, -1};
        public static final int[] FRONT_RIGHT_TURNING_ENCODER_PORTS = new int[] {-1, -1};
        public static final int[] REAR_LEFT_TURNING_ENCODER_PORTS = new int[] {-1, -1};
        public static final int[] REAR_RIGHT_TURNING_ENCODER_PORTS = new int[] {-1, -1};

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH = -1;
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE = -1;
        public static final SwerveDriveKinematics DRIVE_KINEMATICS =
            new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)   
            );
        
        public static boolean GYRO_REVERSED = false;

        // TODO: Figure out these values before using this code in the robot!
        public static final double VOLTS = -1;
        public static final double VOLT_SECONDS_PER_METER = -1;
        public static final double VOLT_SECONDS_SQUARED_PER_METER = -1;

        public static final double MAX_SPEED_METERS_PER_SECOND = 1;
    }

    // TODO: Measure constants on robot.
    public static final class SwerveModuleConstants {
        public static final double DRIVING_CONTROLLER_P_VALUE = 0.5;
        public static final double TURNING_CONTROLLER_P_VALUE = 0.5;

        public static final Constraints ROTATION_CONSTRAINTS = new Constraints(2 * Math.PI, 2 * Math.PI);

        public static final int ENCODER_CYCLES_PER_REVOLUTION = -1;
        public static final double WHEEL_DIAMETER_METERS = -1;

        public static final double DRIVING_ENCODER_DISTANCE_PER_PULSE = 
            (WHEEL_DIAMETER_METERS * Math.PI) / (double) ENCODER_CYCLES_PER_REVOLUTION;

        public static final double TURNING_ENCODER_DISTANCE_PER_PULSE =
            (2 * Math.PI) / (double) ENCODER_CYCLES_PER_REVOLUTION;
        
    }
}
