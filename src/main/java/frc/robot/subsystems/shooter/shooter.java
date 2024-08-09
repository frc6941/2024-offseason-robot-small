package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
    private TalonFX shootermotorH;
    private TalonFX shootermotorL;

    public shooter(int idH, int idL, String canBusName) {
        shootermotorH = new TalonFX(idH, canBusName);
        shootermotorL = new TalonFX(idL, canBusName);
    }

    @Override
    public void periodic() {
    }

    public void setshooter(double VoltageL, double VoltageH) {
        shootermotorL.set(-VoltageL);
        shootermotorH.set(-VoltageH);
    }

    public void setshooteramp(double VoltageL, double VoltageH) {
        shootermotorL.set(-VoltageL);
        shootermotorH.set(-VoltageH);
    }
}
