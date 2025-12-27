package frc.robot.subsystems.groundintake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

public class GroundIntakePivotIOReal implements GroundIntakePivotIO {

    TalonFX groundIntakePivotMotor;
    TalonFXConfiguration groundIntakeMotorPivotConfig;

    private PositionVoltage positionVoltage = new PositionVoltage(0);

    public GroundIntakePivotIOReal() {

        groundIntakePivotMotor = new TalonFX(Constants.GROUNDINTAKEPIVOTE_ID);
    }

    @Override
    public void updateState(GroundIntakePivotIOInputs inputs){
        
    }

    @Override
    public void setPivotMotorSetpoint(Angle setpoint) {

    }

    @Override
    public void setPivotMotorVoltage(double voltage) {
        
    }

    @Override
    public Angle getPivotMotorPosition() {

    }

    @Override
    public Angle getGroundIntakePivotPosition() {

    }
}
