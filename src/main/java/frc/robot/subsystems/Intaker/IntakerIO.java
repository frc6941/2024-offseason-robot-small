package frc.robot.subsystems.Intaker;

import edu.wpi.first.units.*;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface IntakerIO {
    default void updateInputs(IntakerIOInputs inputs) {
    }

    default void setVelocity(Measure<Velocity<Angle>> velocity) {
    }

    default void setVoltage(Measure<Voltage> voltage) {
    }


    @AutoLog
    class IntakerIOInputs {
        public boolean intakerConnected = true;
        public Measure<Voltage> voltage = Volts.zero();
        public Measure<Velocity<Angle>> intakerSpeed = RotationsPerSecond.zero();
        public Measure<Current> intakerSupplyCurrent = Amps.zero();
        public boolean higherbeamBreakState = false;
        public boolean lowerBeamBreakState = false;
    }
}
