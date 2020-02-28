package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Object for controller allowing command-based usage.
 */
public class Gamepad extends Joystick {
    private class Direction {
        private static final int UP = 0;
        private static final int UP_RIGHT = 45;
        private static final int RIGHT = 90;
        private static final int DOWN_RIGHT = 135;
        private static final int DOWN = 180;
        private static final int DOWN_LEFT = 225;
        private static final int LEFT = 270;
        private static final int UP_LEFT = 315;
    }
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
    public final Button D_PAD_UP;
    public final Button D_PAD_UP_RIGHT;
    public final Button D_PAD_RIGHT;
    public final Button D_PAD_DOWN_RIGHT;
    public final Button D_PAD_DOWN;
    public final Button D_PAD_DOWN_LEFT;
    public final Button D_PAD_LEFT;
    public final Button D_PAD_UP_LEFT;

    /**
     * Constructor for the Gamepad class.
     * 
     * @param port The computer port that the Gamepad is plugged into.
     */
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
        D_PAD_UP = new DirectionalPad(this, Direction.UP);
        D_PAD_UP_RIGHT = new DirectionalPad(this, Direction.UP_RIGHT);
        D_PAD_RIGHT = new DirectionalPad(this, Direction.RIGHT);
        D_PAD_DOWN_RIGHT = new DirectionalPad(this, Direction.DOWN_RIGHT);
        D_PAD_DOWN = new DirectionalPad(this, Direction.DOWN);
        D_PAD_DOWN_LEFT = new DirectionalPad(this, Direction.DOWN_LEFT);
        D_PAD_LEFT = new DirectionalPad(this, Direction.LEFT);
        D_PAD_UP_LEFT = new DirectionalPad(this, Direction.UP_LEFT);
    }

    /**
     * Get the value of the left trigger on the controller.
     * 
     * @return The value of the left trigger.
     */
    public double getLeftTrigger() {
        return getRawAxis(RobotMap.LEFT_TRIGGER);
    }

    /**
     * Get the value of the right trigger on the controller.
     * 
     * @return The value of the right trigger.
     */
    public double getRightTrigger() {
        return getRawAxis(RobotMap.RIGHT_TRIGGER);
    }

    /**
     * Get the value of the left stick's horizontal direction on the controller.
     * 
     * @return The value of the left stick's horizontal direction.
     */
    public double getLeftStickX() {
        return getRawAxis(RobotMap.LEFT_STICK_X);
    }

    /**
     * Get the value of the left stick's vertical direction on the controller.
     * 
     * @return The value of the left stick's vertical direction.
     */
    public double getLeftStickY() {
        return getRawAxis(RobotMap.LEFT_STICK_Y);
    }

    /**
     * Get the value of the right stick's horizontal direction on the controller.
     * 
     * @return The value of the right stick's horizontal direction.
     */
    public double getRightStickX() {
        return getRawAxis(RobotMap.RIGHT_STICK_X);
    }

    /**
     * Get the value of the right stick's vertical direction on the controller.
     * 
     * @return The value of the right stick's vertical direction.
     */
    public double getRightStickY() {
        return getRawAxis(RobotMap.RIGHT_STICK_Y);
    }
}
