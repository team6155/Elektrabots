package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Drive;

/**
 * Subsystem controlling the robot's wheels.
 */
public abstract class DriveTrain extends Subsystem {
    protected RobotDriveBase wheels;
    protected int direction;
    protected Gyro gyro;

    public class Direction {
        public static final int FORWARD = 1;
        public static final int BACKWARD = -1;
    }

    /**
     * Constructor for DriveTrain class
     */
    public DriveTrain() {
        direction = Direction.FORWARD;
        gyro = new ADXRS450_Gyro(RobotMap.GYRO);
    }

    public int getDirection() {
        return direction;
    }

    public double readGyro() {
        return gyro.getAngle();
    }

    /**
     * Drive the robot according to the given inputs.
     * 
     * @param forwardsSpeed The robot's speed forwards or backwards [-1.0..1.0].
     *                      Back is negative and forwards is positive.
     * @param sidewaysSpeed The robot's speed left or right [-1.0..1.0].
     *                      Left is negative and right is positive.
     * @param rotationSpeed The robot's rotational speed [-1.0..1.0].
     *                      Counter-clockwise is negative and clockwise is positive.
     */
    public abstract void drive(double forwardsSpeed, double sidewaysSpeed, double rotationSpeed);

    /**
     * Drive the robot according to the given inputs.
     * 
     * @param forwardsSpeed The robot's speed forwards or backwards [-1.0..1.0].
     *                      Back is negative and forwards is positive.
     * @param rotationSpeed The robot's rotational speed [-1.0..1.0].
     *                      Counter-clockwise is negative and clockwise is positive.
     */
    public abstract void drive(double forwardsSpeed, double RotationSpeed);

    /**
     * Change the forwards direction of the robot.
     * 
     * @return The new direction of the robot.
     */
    public int changeDirection() {
        if (direction == Direction.FORWARD) {
            direction = Direction.BACKWARD;
        }
        else {
            direction = Direction.FORWARD;
        }
        return direction;
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
