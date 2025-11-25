package frc.robot.subsystems.elevator;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

/**
 * Class that holds control logic and public interface for the elevator.
 */
public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged replayedInputs = new ElevatorIOInputsAutoLogged();

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


    }

    public void lowerElevator() {
      io.setMotorSetpoint(ElevatorHeight.DOWN.getPositionAngle());
    }

    public void raiseElevator() {
      io.setMotorSetpoint(ElevatorHeight.UP.getPositionAngle());
    }

    enum ElevatorHeight {
        // Positions taken from offseason bot code, inturn taken from onshape.
        UP(Inches.of(0.0)),
        DOWN(Inches.of(56.0));

        public final Distance position;

        ElevatorHeight(Distance position) {
            this.position = position;
        }

        public Distance getPosition() {
            return position;
        }

        public Angle getPositionAngle() {
          // NOTE: yes this code is bs, but i wanna see it work first.
          return Rotations.of(getPosition().in(Inches)/(ElevatorConstants.ELEVATOR_HEIGHT_CHANGE_PER_MOTOR_ROTATION));
        }
    }
}
