package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
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
        frontRightWheel = new PWMVictorSPX(RobotMap.frontRightWheel);
        frontLeftWheel = new PWMVictorSPX(RobotMap.frontLeftWheel);
        rearRightWheel = new PWMVictorSPX(RobotMap.rearRightWheel);
        rearLeftWheel = new PWMVictorSPX(RobotMap.rearLeftWheel);
        wheels = new MecanumDrive(frontLeftWheel, rearLeftWheel, frontRightWheel, rearRightWheel);
        direction = 1;
    }

    /**
     * Drive the robot according to the given inputs.
     * 
     * @param forwardsSpeed The robot's speed forwards or backwards [-1.0..1.0]. Back is
     *                      negative and forwards is positive.
     * @param sidewaysSpeed The robot's sideways speed [-1.0..1.0]. Left is negative and
     *                      right is positive.
     * @param rotationSpeed The robot's rotational speed [1.0..1.0]. Counter-clockwise is
     *                      negative and clockwise is positive.
     */
    public void drive(double forwardsSpeed, double sidewaysSpeed, double rotationSpeed) {
        wheels.driveCartesian(forwardsSpeed, sidewaysSpeed, rotationSpeed);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive(Robot.m_oi.getController()));
    }
}