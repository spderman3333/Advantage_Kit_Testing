package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;

public class ArmConstants {

    // 78 motor rotations : 1 arm rotation
    public static final int MOTOR_TO_ARM_GEARING = 78;

    // NOTE: This weight has been given by design, thanks design.
    public static final Mass ARM_WEIGHT = Pounds.of(4.119);

    // NOTE: More or less accurate, should be fine for now.
    public static final Distance ARM_LENGTH = Inches.of(23.2);

    // Any more than these and the wires snap I believe.
    // NOTE: Check to see if these degrees need to be sharpened for, maybe to 180°.
    public static final Angle MAX_ARM_ROTATION = Degrees.of(360);
    public static final Angle MIN_ARM_ROTATION = Degrees.of(-360);

    public static final Voltage INTAKE_VOLTAGE = Volts.of(6);
    public static final Voltage OUTTAKE_VOLTAGE = Volts.of(-6);

    //  public static final double ARM_PIVOT_kG = 0.0;
    //  public static final double ARM_PIVOT_kS = 0.0;
    //  public static final double ARM_PIVOT_kV = 0.0;
    //  public static final double ARM_PIVOT_kA = 0.0;
    public static final double ARM_PIVOT_kP = 0.4;
    public static final double ARM_PIVOT_kI = 0.0;
    public static final double ARM_PIVOT_kD = 0.0;
}
