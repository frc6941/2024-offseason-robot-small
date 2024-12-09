package frc.robot.subsystems.Intaker;

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
    private final TalonFX intakeMotor = new TalonFX(
            RobotConstants.IntakerConstants.INTAKE_MOTOR_ID,
            RobotConstants.CAN_BUS_NAME);
    private final BeamBreak intakerBeamBreak =
            new BeamBreak(RobotConstants.BeamBreakConstants.INTAKER_BEAMBREAK_ID);

    public IntakerIOTalonFX() {
        intakeMotor.getConfigurator().apply(new Slot0Configs()
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
                intakeMotor.getVelocity(),
                intakeMotor.getMotorVoltage(),
                intakeMotor.getSupplyCurrent()
        ).isOK();
        inputs.beamBreakState = intakerBeamBreak.get();
        inputs.intakerSpeed = RotationsPerSecond.of(intakeMotor.getVelocity().getValueAsDouble());
        inputs.voltage = Volts.of(intakeMotor.getMotorVoltage().getValueAsDouble());
        inputs.intakeSupplyCurrent = Amps.of(intakeMotor.getSupplyCurrent().getValueAsDouble());
    }

    @Override
    public void setVelocity(Measure<Velocity<Angle>> velocityRPM) {
        intakeMotor.setControl(new VelocityVoltage(
                velocityRPM.magnitude() / 60,
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
        intakeMotor.setVoltage(voltage.magnitude());
    }
}
