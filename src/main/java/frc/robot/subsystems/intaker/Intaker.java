package frc.robot.subsystems.intaker;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intaker extends SubsystemBase {

    private TalonFX intakermotor;

    public Intaker(int id, String canBusName) {
        intakermotor = new TalonFX(id, canBusName);
    }

    @Override
    public void periodic() {
    }

    public void setIntakerDutyCycle(double dutyCycle) {
        intakermotor.set(dutyCycle);
    }

}
