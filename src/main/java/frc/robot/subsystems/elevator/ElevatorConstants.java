package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import frc.robot.subsystems.arm.ArmConstants;

public class ElevatorConstants {
    // Gotten from design and fab.
    public static final double MOTOR_TO_ELEVATOR_GEARING = 8.06666667;

    // NOTE: Takes arm weight into account.
    public static final Mass ELEVATOR_CARRIAGE_WEIGHT = Pounds.of(5.454).plus(ArmConstants.ARM_WEIGHT);

    // Correct calculation for Inches of travel per 1 Motor Rotation:
    // (2 * pi * Spool Radius) / Gearing
    public static final double ELEVATOR_HEIGHT_CHANGE_PER_MOTOR_ROTATION = 0.349727;

    // Guess what, I asked design again.
    public static final Distance ELEVATOR_SPOOL_RADIUS = Inches.of(0.449);

    public static final Distance ELEVATOR_MIN_HEIGHT = Inches.of(0.0);
    public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(56.0);

    public static final double ELEVATOR_kG = 0.0; // 0.29
    public static final double ELEVATOR_kS = 0.0; // 0.11
    public static final double ELEVATOR_kV = 0.0; // 0.1
    public static final double ELEVATOR_kA = 0.0;
    public static final double ELEVATOR_kP = 0.1;
    public static final double ELEVATOR_kI = 0.0;
    public static final double ELEVATOR_kD = 0.0;
}
