package frc.robot.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;

/**
 * Class handling the triggers on the gamepad.
 */
public class GamepadTrigger extends Button {
    private final Joystick JOYSTICK;
    private final int AXIS;

    /**
     * Constructor for the GamepadTrigger class.
     * 
     * @param joystick The gamepad that this trigger belongs to.
     * @param axis What port on the gamepad corresponds to this trigger.
     */
    public GamepadTrigger(Joystick joystick, int axis) {
        this.JOYSTICK = joystick;
        this.AXIS = axis;
    }

    /**
     * Get the value of this trigger.
     * 
     * @return The current value of this trigger.
     */
    public boolean get() {
        return JOYSTICK.getRawAxis(AXIS) > 0;
    }
}
