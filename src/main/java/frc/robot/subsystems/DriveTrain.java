package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

/**
 * Subsystem controlling the robot's wheels.
 */
public class DriveTrain extends Subsystem {
    private SpeedController frontRightWheel;
    private SpeedController frontLeftWheel;
    private SpeedController rearRightWheel;
    private SpeedController rearLeftWheel;
    private MecanumDrive wheels;
    private int direction;

    /**
     * Constructor for DriveTrain class
     */
    public DriveTrain() {
        frontRightWheel = new PWMVictorSPX(RobotMap.FRONT_RIGHT_WHEEL);
        frontLeftWheel = new PWMVictorSPX(RobotMap.FRONT_LEFT_WHEEL);
        rearRightWheel = new PWMVictorSPX(RobotMap.REAR_RIGHT_WHEEL);
        rearLeftWheel = new PWMVictorSPX(RobotMap.REAR_LEFT_WHEEL);
        wheels = new MecanumDrive(frontLeftWheel, rearLeftWheel, frontRightWheel, rearRightWheel);
        direction = 1;
    }

    /**
     * Drive the robot according to the given inputs.
     * 
     * @param forwardsSpeed The robot's speed forwards or backwards [-1.0..1.0]. Back is negative
     *                      and forwards is positive.
     * @param sidewaysSpeed The robot's sideways speed [-1.0..1.0]. Left is negative and right is
     *                      positive.
     * @param rotationSpeed The robot's rotational speed [1.0..1.0]. Counter-clockwise is negative
     *                      and clockwise is positive.
     */
    public void drive(double forwardsSpeed, double sidewaysSpeed, double rotationSpeed) {
        wheels.driveCartesian(direction * sidewaysSpeed, direction * -forwardsSpeed,
                direction * rotationSpeed);
    }

    /**
     * Change the forwards direction of the robot.
     * 
     * @return The new direction of the robot.
     */
    public int changeDirection() {
        direction = direction == 1 ? -1 : 1;
        return direction;
    }

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
        Timer timer = new Timer();
        timer.delay(1);
        motor.stopMotor();
    }

    // Set the default command of the subsystem.
    // This command will run whenever the subsystem is not being used by another command.
    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive(Robot.oi.getController()));
    }
}
