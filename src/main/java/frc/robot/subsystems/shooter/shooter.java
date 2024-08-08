package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.a.TunableNumber;

public class shooter extends SubsystemBase {
    private TalonFX shootermotorH;
    private TalonFX shootermotorL;
    public static final TunableNumber H = new TunableNumber("H", 1);
    public static final TunableNumber L = new TunableNumber("L", 0.4);

    public shooter(int idH, int idL, String canBusName) {
        shootermotorH = new TalonFX(idH, canBusName);
        shootermotorL = new TalonFX(idL, canBusName);
    }

    @Override
    public void periodic() {
    }

    public void setshooter(double Voltage) {
        shootermotorH.set(-Voltage * 0.95);
        shootermotorL.set(-Voltage * 1);
    }

    public void setshooteramp(double Voltage) {
        shootermotorH.set(-Voltage * 0.70);
        shootermotorL.set(-Voltage);
    }
}
