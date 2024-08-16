package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class shooter extends SubsystemBase {
    private final TalonFX shooterMotorH;
    private final TalonFX shooterMotorL;

    public shooter(int idH, int idL, String canBusName) {
        shooterMotorH = new TalonFX(idH, canBusName);
        shooterMotorL = new TalonFX(idL, canBusName);
    }

    @Override
    public void periodic() {
        Slot0Configs configs = new Slot0Configs()
                .withKP(Constants.ShooterConstants.shooterGainsClass.SHOOTER_KP.get())
                .withKI(Constants.ShooterConstants.shooterGainsClass.SHOOTER_KI.get())
                .withKD(Constants.ShooterConstants.shooterGainsClass.SHOOTER_KD.get())
                .withKA(Constants.ShooterConstants.shooterGainsClass.SHOOTER_KA.get())
                .withKV(Constants.ShooterConstants.shooterGainsClass.SHOOTER_KV.get())
                .withKS(Constants.ShooterConstants.shooterGainsClass.SHOOTER_KS.get());
        shooterMotorH.getConfigurator().apply(configs);
        shooterMotorL.getConfigurator().apply(configs);
    }

    public void setShooterDutyCycle(double dutyCycleL, double dutyCycleH) {
        shooterMotorL.set(-dutyCycleL);
        shooterMotorH.set(-dutyCycleH);
    }

    public void setShooterVelocity(double RPM_L, double RPM_H) {
        SmartDashboard.putNumber("Shooter/SHOOTER-L ACT",shooterMotorL.getVelocity().getValue());
        SmartDashboard.putNumber("Shooter/SHOOTER-L REQ", Units.rotationsPerMinuteToRadiansPerSecond(RPM_L));
        shooterMotorL.setControl(new VelocityVoltage(
                Units.radiansToRotations(Units.rotationsPerMinuteToRadiansPerSecond(RPM_L)),
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));

        SmartDashboard.putNumber("Shooter/SHOOTER-H ACT",shooterMotorH.getVelocity().getValue());
        SmartDashboard.putNumber("Shooter/SHOOTER-H REQ", Units.rotationsPerMinuteToRadiansPerSecond(RPM_H));
        shooterMotorH.setControl(new VelocityVoltage(
                Units.radiansToRotations(Units.rotationsPerMinuteToRadiansPerSecond(RPM_H)),
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
    }
}
