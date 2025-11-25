package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs {
        public double carriagePositionInches = 0.0;
        public double motorRotations = 0.0;
        public double motorSpeedRotsPerSecond = 0.0;
        public double motorCurrent = 0.0;
        public double motorVoltage = 0.0;
    }

    /**
     * Used to update Advantage kit autologged input data, as well as any other necessary states (like in sim)
     * @param inputs The "struct" (data class) to handle hardware inputs.
     */
    default void updateState(ElevatorIOInputs inputs) {}

    /**
     * Uses positional control for the motor, using its internal PID values.
     * @param position position for the motor to go to.
     */
    default void setMotorSetpoint(Angle position) {}

    /**
     * @param voltage Voltage to apply to the motor.
     */
    default void setMotorVoltage(Voltage voltage) {}

    /**
     * @return The angle of the motor.
     */
    default Angle getMotorPosition() {
        return Rotations.of(0);
    }
}
