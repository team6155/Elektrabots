package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Object for controller allowing command-based usage.
 */
public class Gamepad extends Joystick {
    public final Button A_BUTTON;
    public final Button B_BUTTON;
    public final Button X_BUTTON;
    public final Button Y_BUTTON;
    public final Button LEFT_BUMPER;
    public final Button RIGHT_BUMPER;
    public final Button BACK_BUTTON;
    public final Button START_BUTTON;
    public final Button LEFT_TRIGGER;
    public final Button RIGHT_TRIGGER;

    public Gamepad(int port) {
        super(port);
        A_BUTTON = new JoystickButton(this, RobotMap.A_BUTTON);
        B_BUTTON = new JoystickButton(this, RobotMap.B_BUTTON);
        X_BUTTON = new JoystickButton(this, RobotMap.X_BUTTON);
        Y_BUTTON = new JoystickButton(this, RobotMap.Y_BUTTON);
        LEFT_BUMPER = new JoystickButton(this, RobotMap.LEFT_BUMPER);
        RIGHT_BUMPER = new JoystickButton(this, RobotMap.RIGHT_BUMPER);
        BACK_BUTTON = new JoystickButton(this, RobotMap.BACK_BUTTON);
        START_BUTTON = new JoystickButton(this, RobotMap.START_BUTTON);
        LEFT_TRIGGER = new GamepadTrigger(this, RobotMap.LEFT_TRIGGER);
        RIGHT_TRIGGER = new GamepadTrigger(this, RobotMap.RIGHT_TRIGGER);
    }

    public double getLeftTrigger() {
        return getRawAxis(RobotMap.LEFT_TRIGGER);
    }

    public double getRightTrigger() {
        return getRawAxis(RobotMap.RIGHT_TRIGGER);
    }

    public double getLeftStickX() {
        return getRawAxis(RobotMap.LEFT_STICK_X);
    }

    public double getLeftStickY() {
        return getRawAxis(RobotMap.LEFT_STICK_Y);
    }

    public double getRightStickX() {
        return getRawAxis(RobotMap.RIGHT_STICK_X);
    }

    public double getRightStickY() {
        return getRawAxis(RobotMap.RIGHT_STICK_Y);
    }
}
