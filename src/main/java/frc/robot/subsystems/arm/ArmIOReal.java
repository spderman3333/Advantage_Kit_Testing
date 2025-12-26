package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

// TODO: Test this class on LePrawn, ensure the arm won't rotate more than once in any direction.
public class ArmIOReal implements ArmIO {

    // Pivot motors & configurations
    TalonFX pivotMotor;
    TalonFXConfiguration pivotMotorConfig;

    CANcoder armEncoder;
    CANcoderConfiguration armEncoderConfig;

    PositionVoltage pivotPositionControl = new PositionVoltage(0);

    // Intake/Outtake motors & configurations
    TalonFX intakeMotor;
    TalonFXConfiguration intakeMotorConfig;

    public ArmIOReal() {
        // Set up the arm encoder (records the arm position)
        armEncoder = new CANcoder(ARM_ENCODER_ID);
        armEncoderConfig = new CANcoderConfiguration();
        armEncoderConfig.withMagnetSensor(
                new MagnetSensorConfigs().withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

        // Set up arm pivot motor.
        pivotMotor = new TalonFX(ARM_PIVOT_ID);

        // Set the current position of the arm to what the position of the absolute encoder.
        // Negates the encoder position because the encoder and motor are on different sides of the arm carriage.
        pivotMotor.setPosition(armEncoder.getAbsolutePosition().getValue().unaryMinus());

        // Start motor config.
        pivotMotorConfig = new TalonFXConfiguration();

        // PID
        pivotMotorConfig.withSlot0(
                new Slot0Configs().withKP(ARM_PIVOT_kP).withKI(ARM_PIVOT_kI).withKD(ARM_PIVOT_kD));

        // Motor Rotation Direction - Positive: arm moves clockwise.
        pivotMotorConfig.withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        // End motor config.
        pivotMotor.getConfigurator().apply(pivotMotorConfig);

        // Set up arm intake motor.
        intakeMotor = new TalonFX(ARM_INTAKE_ID);

        // Start motor config.
        intakeMotorConfig = new TalonFXConfiguration();

        // Motor Rotation Direction - Positive: intake. Negative: outtake.
        intakeMotorConfig.withMotorOutput(
                new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

        // End motor config.
        intakeMotor.getConfigurator().apply(intakeMotorConfig);
    }

    @Override
    public void updateState(ArmIOInputs inputs) {
        inputs.armPositionDegrees = pivotMotor.getPosition().getValue().in(Degrees) / MOTOR_TO_ARM_GEARING;

        inputs.pivotMotorRotations = pivotMotor.getPosition().getValueAsDouble();
        inputs.pivotMotorVelocityRotsPerSecond = pivotMotor.getVelocity().getValueAsDouble();
        inputs.pivotMotorCurrent = pivotMotor.getStatorCurrent().getValueAsDouble();
        inputs.pivotMotorVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();

        inputs.intakeMotorVelocityRotsPerSecond = intakeMotor.getVelocity().getValueAsDouble();
        inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();

        inputs.encoderPositionDegrees = armEncoder.getPosition().getValue().in(Degrees);
    }

    @Override
    public void setPivotMotorSetpoint(Angle setpoint) {
        // "0" in sim is facing to the east, while in real robot, it is north. No me gusta, but thus must be done.

        pivotMotor.setControl(pivotPositionControl.withPosition(setpoint));
    }

    @Override
    public void setPivotMotorVoltage(Voltage voltage) {
        pivotMotor.setVoltage(voltage.magnitude());
    }

    @Override
    public Angle getPivotMotorPosition() {
        return pivotMotor.getPosition().getValue();
    }

    // TODO & NOTE: Test on bot, if it doesn't work move to the Arm.java (this and sim impl) and fix.
    @Override
    public Angle getArmPosition() {
        return pivotMotor.getPosition().getValue().div(MOTOR_TO_ARM_GEARING);
    }

    @Override
    public void setIntakeMotorVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage.magnitude());
    }
}
