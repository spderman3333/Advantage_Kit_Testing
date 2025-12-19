package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SimulationVisualizer;
import org.littletonrobotics.junction.Logger;

/**
 * Class that holds control logic and public interface for the elevator.
 */
public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged replayedInputs = new ElevatorIOInputsAutoLogged();

    private ElevatorHeight currentElevatorSetpoint = ElevatorHeight.DOWN;

    /**
     * @param io The hardware implementation for the elevator, either sim or real.
     */
    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void stopElevator() {
        io.setMotorVoltage(Volts.of(0));
    }

    @Override
    public void periodic() {
        io.updateState(replayedInputs);
        // Must be called every periodic after updating hardware state.
        Logger.processInputs("Elevator", replayedInputs);
        Logger.recordOutput("Elevator", currentElevatorSetpoint.getPosition().in(Inches));
    }

    @Override
    public void simulationPeriodic() {
        SimulationVisualizer.getInstance().updateElevatorHeight(io.getCarriagePosition());
    }

    public void setElevatorPosition(ElevatorHeight height) {
        currentElevatorSetpoint = height;
        io.setMotorSetpoint(height.getPositionAngle());
    }

    public Command setElevatorPositionCommand(ElevatorHeight height) {
        return new InstantCommand(() -> setElevatorPosition(height));
    }

    public enum ElevatorHeight {
        // Positions taken from offseason bot code, inturn taken from onshape.
        UP(Inches.of(56.0)),
        MIDDLE(Inches.of(5.0)),
        DOWN(Inches.of(0.0));

        public final Distance position;

        ElevatorHeight(Distance position) {
            this.position = position;
        }

        public Distance getPosition() {
            return position;
        }

        public Angle getPositionAngle() {
            // NOTE: yes this code is bs, but i wanna see it work first.
            return Radians.of(getPosition().in(Inches) / (ElevatorConstants.ELEVATOR_HEIGHT_CHANGE_PER_MOTOR_ROTATION));
        }
    }
}
