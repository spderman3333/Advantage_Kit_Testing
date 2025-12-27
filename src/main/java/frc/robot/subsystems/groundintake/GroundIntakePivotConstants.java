package frc.robot.subsystems.groundintake;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public class GroundIntakePivotConstants {
    public static final Angle PIVOT_UP_POSITION = Units.Rotations.of(0.15869140625);
    public static final Angle PIVOT_DOWN_POSITION = Units.Rotations.of(-15.623046875);
    public static final double ACCEPTABLE_ERROR = 0.1;
    public static final double kP_PIVOT = 1;
    public static final double kI_PIVOT = 0;
    public static final double kD_PIVOT = 0;
}
