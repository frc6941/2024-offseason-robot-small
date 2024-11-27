package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.drivers.BeamBreak;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX shooterHighMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_MOTORH_ID, Constants.RobotConstants.CAN_BUS_NAME);
    private final TalonFX shooterLowMotor = new TalonFX(Constants.ShooterConstants.SHOOTER_MOTORL_ID, Constants.RobotConstants.CAN_BUS_NAME);
    private final BeamBreak intakerBeamBreak = new BeamBreak(Constants.BeamBreakConstants.INTAKER_BEAMBREAK_ID);
    private final BeamBreak shooterBeamBreak = new BeamBreak(Constants.BeamBreakConstants.SHOOTER_BEAMBREAK_ID);
    private boolean lastRecordedState;
    private boolean noteState = false;

    public ShooterSubsystem() {
        shooterHighMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(0.25)
                .withKI(0.0)
                .withKD(0.001)
                .withKA(0.0037512677)
                .withKV(0.115)
                .withKS(0.28475008));
        shooterLowMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(0.25)
                .withKI(0.0)
                .withKD(0.001)
                .withKA(0.0037512677)
                .withKV(0.115)
                .withKS(0.28475008));

        boolean isIntakerBeamBreakOn = intakerBeamBreak.get();
        boolean isShooterBeamBreakOn = shooterBeamBreak.get();
        shooterLowMotor.setInverted(true);
        shooterHighMotor.setInverted(true);
        if (!lastRecordedState && isIntakerBeamBreakOn) {
            noteState = true;
        }
        lastRecordedState = isShooterBeamBreakOn;

    }

    public void setShooterRPM(double highMotorVelocityRPM, double lowMotorVelocityRPM) {
        var highMotorVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(highMotorVelocityRPM);
        var lowMotorVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(lowMotorVelocityRPM);
        shooterHighMotor.setControl(new VelocityVoltage(
                Units.radiansToRotations(highMotorVelocityRadPerSec),
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
        shooterLowMotor.setControl(new VelocityVoltage(
                Units.radiansToRotations(lowMotorVelocityRadPerSec),
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
    }

    public void setShooterDirectVoltage(double voltage) {
        shooterHighMotor.setVoltage(voltage);
        shooterLowMotor.setVoltage(voltage);
    }


    public boolean isShootComplete() {
        return !shooterBeamBreak.get();
    }

}

