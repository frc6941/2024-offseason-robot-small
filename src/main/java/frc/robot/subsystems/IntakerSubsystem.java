package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drivers.BeamBreak;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.IntakerConstants.INTAKE_MOTOR_ID;
import static frc.robot.Constants.IntakerConstants.motorOutputConfigs;

public class IntakerSubsystem extends SubsystemBase{
    private final TalonFX intakeMotor = new TalonFX(Constants.IntakerConstants.INTAKE_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    private final BeamBreak intakerBeamBreak = new BeamBreak(Constants.BeamBreakConstants.INTAKER_BEAMBREAK_ID);
    private boolean lastRecordedState;
    private boolean noteState = false;

    public IntakerSubsystem() {
        boolean isIntakerBeamBreakOn = intakerBeamBreak.get();
        if (!lastRecordedState && isIntakerBeamBreakOn) {
            noteState = true;
        }
        lastRecordedState = isIntakerBeamBreakOn;

        intakeMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(0.2)
                .withKI(0)
                .withKD(0.001)
                .withKA(0.0037512677)
                .withKV(0.115)
                .withKS(0.28475008));
    }

    public void setIndexRPM(double velocityRPM) {
        var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        intakeMotor.setControl(new VelocityVoltage(
                Units.radiansToRotations(velocityRadPerSec),
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
    }

    public boolean intakerHasNote() {
        return noteState;
    }
    
}
