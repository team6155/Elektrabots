package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;

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

    public void drive(double forwardsSpeed, double sidewaysSpeed, double rotation) {
        wheels.driveCartesian(forwardsSpeed, sidewaysSpeed, rotation);
    }

    @Override
    protected void initDefaultCommand() {

    }
}