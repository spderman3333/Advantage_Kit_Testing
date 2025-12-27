package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Generic Controller interface by team 2813.
 *
 * <p>This interface is abstracting implementation differences by common controller types
 * (e.g., PS4 and XBox controllers). It supports the least-common-denominator interface
 * between these implementations.
 */
public interface Controller {
    /** Face pad (not POV/DPAD) button: Top */
    public Trigger faceTop();

    /** Face pad (not POV/DPAD) button: Right */
    public Trigger faceRight();

    /** Face pad (not POV/DPAD) button: Down */
    public Trigger faceDown();

    /** Face pad (not POV/DPAD) button: Left */
    public Trigger faceLeft();

    /** DPAD controls: Up */
    public Trigger povUp();

    /** DPAD controls: Right */
    public Trigger povRight();

    /** DPAD controls: Down */
    public Trigger povDown();

    /** DPAD controls: Left */
    public Trigger povLeft();

    /** Shoulder top of controller: Left */
    public Trigger shoulderLeft();

    /** Shoulder top of controller: Right */
    public Trigger shoulderRight();

    /** Trigger under shoulder: Left */
    public Trigger triggerLeft();

    /** Trigger under shoulder: Right */
    public Trigger triggerRight();

    /** Setting menu buttons: Left */
    public Trigger leftMenuButtons();

    /** Setting menu buttons: Right */
    public Trigger rightMenuButtons();

    /** Joystick buttons: Left */
    public Trigger joystickLeftPress();

    /** Joystick buttons: Right */
    public Trigger joystickRightPress();

    /** Left Joystick, X-Axis */
    public double leftXAxis();

    /** Left Joystick, Y-Axis */
    public double leftYAxis();

    /** Right Joystick, X-Axis */
    public double rightXAxis();

    /** Right Joystick, Y-Axis */
    public double rightYAxis();
}
