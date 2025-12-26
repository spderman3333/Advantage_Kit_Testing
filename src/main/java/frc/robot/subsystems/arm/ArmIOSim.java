package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ArmConstants.MOTOR_TO_ARM_GEARING,
            SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_WEIGHT.in(Kilograms)),
            ARM_LENGTH.in(Meters),
            MIN_ARM_ROTATION.in(Radians),
            MAX_ARM_ROTATION.in(Radians),
            true,
            Degrees.of(90).in(Radians));

    // CANCoder configuration
    CANcoder armEncoder;
    CANcoderSimState armEncoderSimState;

    // Pivot motors & configurations
    TalonFX pivotMotor;
    TalonFXSimState pivotMotorSimState;
    TalonFXConfiguration pivotMotorConfig;

    PositionVoltage pivotPositionControl = new PositionVoltage(0);

    // Intake/Outtake motors & configurations
    TalonFX intakeMotor;
    TalonFXSimState intakeMotorSimState;
    TalonFXConfiguration intakeMotorConfig;

    public ArmIOSim() {
        // Set up arm encoder
        armEncoder = new CANcoder(ARM_ENCODER_ID);
        armEncoderSimState = armEncoder.getSimState();

        armEncoder.setPosition(Radians.of(armSim.getAngleRads()));

        // Set up arm pivot motor.
        pivotMotor = new TalonFX(ARM_PIVOT_ID);
        pivotMotorSimState = pivotMotor.getSimState();

        // Start motor config.
        pivotMotorConfig = new TalonFXConfiguration();

        // PID
        pivotMotorConfig.withSlot0(
                new Slot0Configs().withKP(ARM_PIVOT_kP).withKI(ARM_PIVOT_kI).withKD(ARM_PIVOT_kD));

        // Motor Rotation Direction - Positive: arm moves clockwise
        pivotMotorConfig.withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        // End motor config.
        pivotMotor.getConfigurator().apply(pivotMotorConfig);

        // Set up arm intake motor.
        intakeMotor = new TalonFX(ARM_INTAKE_ID);
        intakeMotorSimState = intakeMotor.getSimState();

        // Start Motor Config
        intakeMotorConfig = new TalonFXConfiguration();

        // Motor Rotation Direction - Positive: intake. Negative: outtake.
        intakeMotorConfig.withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        // End Motor Config
        intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    @Override
    public void updateState(ArmIOInputs inputs) {
        updateSim();

        inputs.armPositionDegrees = getArmPosition().in(Degrees);

        inputs.pivotMotorRotations = pivotMotor.getPosition().getValueAsDouble();
        inputs.pivotMotorVelocityRotsPerSecond = pivotMotor.getVelocity().getValueAsDouble();
        inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();

        inputs.intakeMotorVelocityRotsPerSecond = intakeMotor.getVelocity().getValueAsDouble();
        inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();

        inputs.encoderPositionDegrees = armEncoder.getPosition().getValue().in(Degrees);
    }

    private void updateSim() {
        pivotMotorSimState.setSupplyVoltage(Volts.of(12));
        intakeMotorSimState.setSupplyVoltage(Volts.of(12));
        armEncoderSimState.setSupplyVoltage(Volts.of(12));

        armEncoder.setPosition(Radians.of(armSim.getAngleRads()));

        armSim.setInputVoltage(pivotMotorSimState.getMotorVoltage());
        armSim.update(0.02);

        Logger.recordOutput("Simulated Arm Pivot/pivotMotorSimState/Voltage", pivotMotorSimState.getMotorVoltage());

        Logger.recordOutput("Simulated Arm Intake/intakeMotorSimState/Voltage", intakeMotorSimState.getMotorVoltage());

        // TODO: Make the conversions nicer.
        pivotMotorSimState.setRawRotorPosition(Units.radiansToRotations(armSim.getAngleRads()) * MOTOR_TO_ARM_GEARING);
        pivotMotorSimState.setRotorVelocity(armSim.getVelocityRadPerSec() / (2.0 * Math.PI) * MOTOR_TO_ARM_GEARING);
    }

    @Override
    public void setPivotMotorSetpoint(Angle setpoint) {
        pivotMotor.setControl(pivotPositionControl.withPosition(setpoint));
    }

    @Override
    public void setPivotMotorVoltage(Voltage voltage) {
        pivotMotor.setVoltage(voltage.magnitude());
    }

    @Override
    public void setIntakeMotorVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage.magnitude());
    }

    @Override
    public Angle getPivotMotorPosition() {
        return pivotMotor.getPosition().getValue();
    }

    @Override
    public Angle getArmPosition() {
        return Radians.of(armSim.getAngleRads());
    }
}
