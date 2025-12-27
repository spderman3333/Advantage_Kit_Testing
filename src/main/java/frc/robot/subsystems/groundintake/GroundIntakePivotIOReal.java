package frc.robot.subsystems.groundintake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class GroundIntakePivotIOReal implements GroundIntakePivotIO {

    TalonFX groundIntakePivotMotor;
    TalonFXConfiguration groundIntakeMotorPivotConfig;

    private PositionVoltage positionVoltage = new PositionVoltage(0);

    public GroundIntakePivotIOReal() {

        groundIntakePivotMotor = new TalonFX(Constants.GROUNDINTAKEPIVOTE_ID);

    }

}