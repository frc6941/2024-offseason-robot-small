package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frcteam6941.swerve.SwerveModuleBase;

public class shooter extends SubsystemBase {
    private TalonFX shootermotorH;
    private TalonFX shootermotorL;

    public shooter(int idH, int idL, String canBusName) {
        shootermotorH = new TalonFX(idH, canBusName);
        shootermotorL = new TalonFX(idL, canBusName);

        // Slot0Configs configs = new Slot0Configs()
        // .withKP(0)
        // .withKI(0)
        // .withKD(0);

        // shootermotorL.getConfigurator().apply(configs);
        // shootermotorH.getConfigurator().apply(configs);
    }

    @Override
    public void periodic() {
    }

    public void setshooter(double VoltageL, double VoltageH) {
        shootermotorL.set(-VoltageL);
        shootermotorH.set(-VoltageH);
        // shootermotorL.setControl(new VelocityVoltage(-VoltageL));
        // shootermotorH.setControl(new VelocityVoltage(-VoltageH));
        // SmartDashboard.putNumber("SHOOTERH ACT",
        // shootermotorH.getVelocity().getValue());
        // SmartDashboard.putNumber("SHOOTERH REQ", -VoltageH);
    }
}
