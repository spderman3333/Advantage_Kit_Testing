package frc.robot.subsystems.groundintake;

import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakePivotIO {

    @AutoLog
    class GroundIntakePivotIOInputs {
        public double pivotMotorRotations;
        public double pivotMotorVelocityRotsPerSecond;
        public double pivotMotorCurrent;
        public double pivotMotorVoltage;
    }

    default void updateState(GroundIntakePivotIOInputs inputs) {}

    default void setPivotMotorSetpoint(Angle setpoint) {}

    default void setPivotMotorVoltage(double voltage) {}

    default Angle getPivotMotorPosition() {
        return Units.Rotations.of(0);
    }

    default Angle getGroundIntakePivotPosition() {
        return Units.Degrees.of(0);
    }


}
