package frc.robot.subsystems.groundintake;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;

public class GroundIntakePivotConstants {
    public static final Angle PIVOT_UP_POSITION = Units.Rotations.of(0.15869140625);
    public static final Angle PIVOT_DOWN_POSITION = Units.Rotations.of(-15.623046875);
    public static final double ACCEPTABLE_ERROR = 0.1;
    public static final double kP_PIVOT = 1;
    public static final double kI_PIVOT = 0;
    public static final double kD_PIVOT = 0;
    public static final double MOTOR_TO_PIVOT_GEARING = 21d/11;
    public static final Mass GROUND_INTAKE_WEIGHT = Units.Pounds.of(11.257);
    public static final Distance GROUND_INTAKE_LENGTH = Units.Inches.of(13.751);

    public static final Angle MIN_PIVOT_ANGLE = Units.Degrees.of(0); // Down
    public static final Angle MAX_PIVOT_ANGLE = Units.Degrees.of(138.047); // Up
}
