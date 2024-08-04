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

    public void setshooter(double Voltage) {
        shootermotorH.set(-Voltage * 0.9);
        shootermotorL.set(-Voltage);
    }

    public void setshooteramp(double Voltage) {
        shootermotorH.set(-Voltage * 0.8);
        shootermotorL.set(-Voltage);
    }
}
