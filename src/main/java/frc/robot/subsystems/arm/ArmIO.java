package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {

    @AutoLog
    class ArmIOInputs {
        public double armPositionDegrees; // Note: mapped to a unit circle where east is 0 deg.

        // Pivot motor inputs
        public double pivotMotorRotations;
        public double pivotMotorVelocityRotsPerSecond;
        public double pivotMotorCurrent;
        public double pivotMotorVoltage;

        // Intake motor inputs
        public double intakeMotorVelocityRotsPerSecond;
        public double intakeMotorVoltage;

        // Encoder inputs
        public double encoderPositionDegrees;
    }

    /**
     * Used to update Advantage kit autologged input data, as well as any other necessary states (like in sim)
     * @param inputs The "struct" (data class) to handle hardware inputs.
     */
    default void updateState(ArmIOInputs inputs) {}

    /**
     * Uses positional control for the arm pivot motor, using its internal PID values.
     * @param setpoint Position for the motor to go to.
     */
    default void setPivotMotorSetpoint(Angle setpoint) {}

    /**
     * @param voltage Voltage to apply to the arm pivot motor.
     */
    default void setPivotMotorVoltage(Voltage voltage) {}

    /**
     * @return The angle of the arm pivot motor.
     */
    default Angle getPivotMotorPosition() {
        return Rotations.of(0);
    }

    /**
     * The angle of the arm is related to the angle of the motor, but a calculation with the gear ratio needs to occur.
     * <p>Mapped to a unit circle, where east is 0°</p>
     * @return The angle of the arm.
     */
    default Angle getArmPosition() {
        return Degrees.of(0);
    }

    /**
     * @param voltage Voltage to apply to the arm intake motor.
     */
    default void setIntakeMotorVoltage(Voltage voltage) {}
}
