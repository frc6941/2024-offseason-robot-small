package frc.robot.subsystems.intaker;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intaker extends SubsystemBase {

    private TalonFX intakermotor;

    public intaker(int id, String canBusName) {
        intakermotor = new TalonFX(id, canBusName);
    }

    @Override
    public void periodic() {
    }

    public void setintaker(double Voltage) {
        intakermotor.set(Voltage);
    }

}
