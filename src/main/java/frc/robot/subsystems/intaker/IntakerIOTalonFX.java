package frc.robot.subsystems.intaker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import frc.robot.RobotConstants;
import frc.robot.drivers.BeamBreak;

import static edu.wpi.first.units.Units.*;

public class IntakerIOTalonFX implements IntakerIO {
    private final TalonFX intakerMotor = new TalonFX(
            RobotConstants.IntakerConstants.INTAKER_MOTOR_ID,
            RobotConstants.CAN_BUS_NAME);
    private final BeamBreak higherintakerBeamBreak =
            new BeamBreak(RobotConstants.BeamBreakConstants.HIGHER_INTAKER_BEAMBREAK_ID);
    private final BeamBreak lowerIntakerBeamBreak =
            new BeamBreak(RobotConstants.BeamBreakConstants.LOWER_INTAKER_BEAMBREAK_ID);

    public IntakerIOTalonFX() {
        intakerMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(0.2)
                .withKI(0)
                .withKD(0.001)
                .withKA(0.0037512677)
                .withKV(0.115)
                .withKS(0.28475008));

    }

    @Override
    public void updateInputs(IntakerIOInputs inputs) {
        inputs.intakerConnected = BaseStatusSignal.refreshAll(
                intakerMotor.getVelocity(),
                intakerMotor.getMotorVoltage(),
                intakerMotor.getSupplyCurrent()
        ).isOK();
        inputs.higherbeamBreakState = higherintakerBeamBreak.get();
        inputs.lowerBeamBreakState = lowerIntakerBeamBreak.get();
        inputs.intakerSpeed = RotationsPerSecond.of(intakerMotor.getVelocity().getValueAsDouble());
        inputs.voltage = Volts.of(intakerMotor.getMotorVoltage().getValueAsDouble());
        inputs.intakerSupplyCurrent = Amps.of(intakerMotor.getSupplyCurrent().getValueAsDouble());
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocityRPS) {
        intakerMotor.setControl(new VelocityVoltage(
                velocityRPS.magnitude(),
                0.0,
                true,
                0,
                0,
                false,
                false,
                false
        ));
    }

    @Override
    public void setVoltage(Measure<Voltage> voltage) {
        intakerMotor.setVoltage(voltage.magnitude());
    }
}
