package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;
import frc.robot.drivers.BeamBreak;

public class Shooter extends SubsystemBase {
    private final TalonFX shooterHighMotor = new TalonFX(RobotConstants.ShooterConstants.SHOOTER_MOTORH_ID, RobotConstants.CAN_BUS_NAME);
    private final TalonFX shooterLowMotor = new TalonFX(RobotConstants.ShooterConstants.SHOOTER_MOTORL_ID, RobotConstants.CAN_BUS_NAME);
    private final BeamBreak shooterBeamBreak = new BeamBreak(RobotConstants.BeamBreakConstants.SHOOTER_BEAMBREAK_ID);
    private boolean lastRecordedState;
    private boolean noteState = false;
    private double highMotorVelocityRPM = 0, lowMotorVelocityRPM = 0;

    public Shooter() {
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


        boolean isShooterBeamBreakOn = shooterBeamBreak.get();
        shooterLowMotor.setInverted(true);
        shooterHighMotor.setInverted(true);
        lastRecordedState = isShooterBeamBreakOn;

    }

    public void setShooterRPM(double highMotorVelocityRPM, double lowMotorVelocityRPM) {
        this.highMotorVelocityRPM = highMotorVelocityRPM;
        this.lowMotorVelocityRPM = lowMotorVelocityRPM;
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

    public boolean ShooterVelocityReady() {
        boolean velocityReadyHigh = Math.abs(highMotorVelocityRPM - shooterHighMotor.getVelocity().getValueAsDouble() * 60) < Units.radiansPerSecondToRotationsPerMinute(9);//1
        boolean velocityReadyLow = Math.abs(lowMotorVelocityRPM - shooterLowMotor.getVelocity().getValueAsDouble() * 60) < Units.radiansPerSecondToRotationsPerMinute(9);//1
        boolean velocityReady = velocityReadyHigh && velocityReadyLow;
        SmartDashboard.putBoolean("velocityReady", velocityReady);
        return velocityReady;
    }


}

