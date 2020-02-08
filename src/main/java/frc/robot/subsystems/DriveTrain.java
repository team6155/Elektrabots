package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

/**
 * Subsystem controlling the robot's wheels.
 * <p>
 * The robot has six wheels on a differential drivetrain.
 */
public class DriveTrain extends Subsystem {
    private SpeedController frontRightWheel;
    private SpeedController frontLeftWheel;
    private SpeedController middleRightWheel;
    private SpeedController middleLeftWheel;
    private SpeedController rearRightWheel;
    private SpeedController rearLeftWheel;
    private DifferentialDrive wheels;
    private int direction;

    /**
     * Constructor for DriveTrain class
     */
    public DriveTrain() {
        frontRightWheel = new PWMVictorSPX(RobotMap.FRONT_RIGHT_WHEEL);
        frontLeftWheel = new PWMVictorSPX(RobotMap.FRONT_LEFT_WHEEL);
        rearRightWheel = new PWMVictorSPX(RobotMap.REAR_RIGHT_WHEEL);
        rearLeftWheel = new PWMVictorSPX(RobotMap.REAR_LEFT_WHEEL);
        middleRightWheel = new PWMVictorSPX(RobotMap.MIDDLE_RIGHT_WHEEL);
        middleLeftWheel = new PWMVictorSPX(RobotMap.MIDDLE_LEFT_WHEEL);
        SpeedControllerGroup rightsideWeels = new SpeedControllerGroup(frontRightWheel, middleRightWheel, rearRightWheel);
        SpeedControllerGroup leftsideWheels = new SpeedControllerGroup(frontLeftWheel, middleLeftWheel, rearLeftWheel);
        wheels = new DifferentialDrive(rightsideWeels, leftsideWheels);
        direction = 1;
    }

    /**
     * Drive the robot according to the given inputs.
     * 
     * @param forwardsSpeed The robot's speed forwards or backwards [-1.0..1.0].
     *                      Back is negative and forwards is positive.
     * @param rotationSpeed The robot's rotational speed [-1.0..1.0].
     *                      Counter-clockwise is negative and clockwise is positive.
     */
    public void drive(double forwardsSpeed, double rotationSpeed) {
        // Each input is multiplied by direction so that the user can switch the
        // direction of the robot.
        wheels.arcadeDrive(direction * -forwardsSpeed, rotationSpeed);
    }

    /**
     * Change the forwards direction of the robot.
     * 
     * @return The new direction of the robot.
     */
    public int changeDirection() {
        // If the direction is currently positive, set it to negative. Otherwise, set it
        // to positive.
        direction = direction == 1 ? -1 : 1;
        return direction;
    }

    /**
     * Spin one of the wheels for a second to make sure it is functioning properly.
     * 
     * @param wheel
     */
    public void testWheel(int wheel) {
        SpeedController motor;
        if (wheel == RobotMap.FRONT_LEFT_WHEEL)
            motor = frontLeftWheel;
        else if (wheel == RobotMap.FRONT_RIGHT_WHEEL)
            motor = frontRightWheel;
        else if (wheel == RobotMap.REAR_LEFT_WHEEL)
            motor = rearLeftWheel;
        else
            motor = rearRightWheel;
        motor.set(.5);
        Timer.delay(1);
        motor.stopMotor();
    }

    /**
     * Initialize the default command for a subsystem. By default subsystems have no
     * default command, but if they do, the default command is set with this method.
     * It is called on all Subsystems by CommandBase in the users program after all
     * the Subsystems are created.
     * <p>
     * When the subsystem is not currently in use, it will call the command set in
     * this method.
     */
    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive(Robot.oi.DRIVER_CONTROLLER));
    }
}
