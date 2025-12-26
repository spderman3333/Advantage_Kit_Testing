package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SimulationVisualizer;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged replayedInputs = new ArmIOInputsAutoLogged();

    private ArmPositions currentArmPosition = ArmPositions.NORTH;

    public Arm(ArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateState(replayedInputs);
        Logger.processInputs("Arm", replayedInputs);
    }

    @Override
    public void simulationPeriodic() {
        SimulationVisualizer.getInstance().updateArmRotation(io.getArmPosition());
    }

    public void setArmPosition(ArmPositions armPosition) {
        // WARNING: Make sure you use getMotorPosition() not getArmPosition()!
        io.setPivotMotorSetpoint(armPosition.getMotorPosition());
    }

    public Command setArmPositionCommand(ArmPositions armPosition) {
        return new InstantCommand(() -> setArmPosition(armPosition));
    }

    public void setIntakeMotorVoltage(Voltage voltage) {
        io.setIntakeMotorVoltage(voltage);
    }

    public Command intakeItemCommand() {
        return new InstantCommand(() -> setIntakeMotorVoltage(INTAKE_VOLTAGE));
    }

    public Command outtakeItemCommand() {
        return new InstantCommand(() -> setIntakeMotorVoltage(OUTTAKE_VOLTAGE));
    }

    public Command stopIntakeMotorCommand() {
        return new InstantCommand(() -> setIntakeMotorVoltage(Volts.of(0)));
    }

    public enum ArmPositions {
        // According to a unit circle.
        // These are the arm positions, not motor positions
        EAST(Degrees.of(0)),
        NORTH(Degrees.of(90)),
        WEST(Degrees.of(180)),
        SOUTH(Degrees.of(270));

        public final Angle position;

        ArmPositions(Angle position) {
            this.position = position;
        }

        /**
         * @return The position of the arm.
         */
        public Angle getArmPosition() {
            return position;
        }

        /**
         * @return The position the motor should go to in order to achieve the above armPosition.
         */
        public Angle getMotorPosition() {
            return position.times(MOTOR_TO_ARM_GEARING);
        }
    }
}
