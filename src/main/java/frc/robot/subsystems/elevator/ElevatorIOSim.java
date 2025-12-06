package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

    // Physics sim for the elevator.
    private final ElevatorSim elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60(1),
            ElevatorConstants.MOTOR_TO_ELEVATOR_GEARING,
            ElevatorConstants.ELEVATOR_CARRIAGE_WEIGHT.in(Kilograms),
            ElevatorConstants.ELEVATOR_SPOOL_RADIUS.in(Meter),
            ElevatorConstants.ELEVATOR_MIN_HEIGHT.in(Meter),
            ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meter),
            true,
            0.00);

    private TalonFX motor;
    private TalonFXSimState motorSim;
    private TalonFXConfiguration motorConfig;

    // Used for actually moving the motor to a given position with PID applied to a voltage input.
    PositionVoltage positionControl = new PositionVoltage(0);

    public ElevatorIOSim() {
        motor = new TalonFX(Constants.ELEVATOR_ID);
        motor.setNeutralMode(NeutralModeValue.Brake);

        motorSim = motor.getSimState();

        motorConfig = new TalonFXConfiguration();

        motorConfig.withSlot0(new Slot0Configs()
                .withKP(ElevatorConstants.ELEVATOR_kP)
                .withKI(ElevatorConstants.ELEVATOR_kI)
                .withKD(ElevatorConstants.ELEVATOR_kD));
        motorConfig.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        motor.getConfigurator().apply(motorConfig);
    }

    @Override
    public void updateState(ElevatorIOInputs inputs) {
        updateSim();

        inputs.carriagePositionInches =
                Meters.of(elevatorSim.getPositionMeters()).in(Inches);
        inputs.motorCurrent = motor.getStatorCurrent().getValueAsDouble();
        inputs.motorRotations = motor.getPosition().getValueAsDouble();
        inputs.motorVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.motorVelocityRotsPerSecond = motor.getVelocity().getValueAsDouble();
    }

    private void updateSim() {
        motorSim.setSupplyVoltage(Volts.of(12));

        // Apply the voltage to the sim elevator that we apply to the sim motor.
        elevatorSim.setInputVoltage(motorSim.getMotorVoltage());

        // Logs to "Real Outputs" NT
        Logger.recordOutput("Simulated Elevator/motorSim/Voltage", motorSim.getMotorVoltage());

        elevatorSim.update(0.02); // Same update cycle as an actual robot, 20 ms.

        motorSim.setRawRotorPosition(getMotorRotations(elevatorSim.getPositionMeters()));

        // angular velocity = linear velocity / radius, taken also from 5414
        motorSim.setRotorVelocity(
                ((elevatorSim.getVelocityMetersPerSecond() / ElevatorConstants.ELEVATOR_SPOOL_RADIUS.in(Meters))
                                // radians/sec to rotations/sec
                                / (2.0 * Math.PI))
                        * ElevatorConstants.MOTOR_TO_ELEVATOR_GEARING);
    }

    @Override
    public void setMotorSetpoint(Angle setpoint) {
        motor.setControl(positionControl.withPosition(setpoint));
    }

    @Override
    public void setMotorVoltage(Voltage voltage) {
        motor.setVoltage(voltage.in(Volts));
    }

    @Override
    public Angle getMotorPosition() {
        return motor.getPosition().getValue();
    }

    @Override
    public Distance getCarriagePosition() {
        return Meters.of(elevatorSim.getPositionMeters());
    }

    /**
     * Source: 5414 Pearadox
     * Converts the elevators position (meters) to motor rotations based on the elevator spool radius and motor gearing.
     * @param elevatorPosition
     * @return
     */
    private static double getMotorRotations(double elevatorPosition) {
        // angular displacement in radians = linear displacement / radius
        return Units.radiansToRotations(elevatorPosition / ElevatorConstants.ELEVATOR_SPOOL_RADIUS.in(Meters))
                // multiply by gear ratio
                * ElevatorConstants.MOTOR_TO_ELEVATOR_GEARING;
    }
}
