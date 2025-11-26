package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class ElevatorConstants {
    // Gotten from design and fab.
    public static final double MOTOR_TO_ELEVATOR_GEARING = 8.06666667;

    // NOTE: Does not take arm into account.
    public static final Mass ELEVATOR_CARRIAGE_WEIGHT = Pounds.of(5.454);

    // Correct calculation for Inches of travel per 1 Motor Rotation:
    // (2 * pi * Spool Radius) / Gearing
    public static final double ELEVATOR_HEIGHT_CHANGE_PER_MOTOR_ROTATION = 0.32560277;

    // Guess what, I asked design again.
    public static final Distance ELEVATOR_SPOOL_RADIUS = Inches.of(0.449);

    public static final Distance ELEVATOR_MIN_HEIGHT = Inches.of(0.0);
    public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(56.0);

    //  public static final double kG = 0.29; // 0.3
    //  public static final double kS = 0.11; // 0
    //  public static final double kV = 0.1; // 0
    //  public static final double kA = 0.0; // 0
    public static final double kP = 0.4; // 0.4
    public static final double kI = 0.0; // 0
    public static final double kD = 0.0; // 0
}
