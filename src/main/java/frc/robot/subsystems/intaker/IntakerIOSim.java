package frc.robot.subsystems.intaker;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;

import static edu.wpi.first.units.Units.*;

public class IntakerIOSim implements IntakerIO{
private final DCMotorSim intakeMotorSim;
private double appliedVoltage;
private PIDController intakePID;
public IntakerIOSim() {
    intakeMotorSim = new DCMotorSim(DCMotor.getKrakenX60(1),1,0.1);
    intakePID = new PIDController(1.5,0,0);
    appliedVoltage = 0.0;
}

@Override
public void updateInputs(IntakerIOInputs inputs) {
    intakeMotorSim.update(RobotConstants.LOOPER_DT);
    inputs.higherbeamBreakState = false;
    inputs.intakerSpeed = RotationsPerSecond.of(intakeMotorSim.getAngularVelocityRPM()/60);
    inputs.intakerConnected = true;
    inputs.intakerSupplyCurrent = Amps.of(intakeMotorSim.getCurrentDrawAmps());
    inputs.voltage = Volts.of(appliedVoltage);
}

@Override
public void setVoltage(Measure<Voltage> voltage) {
    this.appliedVoltage = voltage.magnitude();
    intakeMotorSim.setInputVoltage(appliedVoltage);
}

@Override
    public void setVelocity(Measure<Velocity<Angle>> velocity) {
    appliedVoltage = intakePID.calculate(intakeMotorSim.getAngularVelocityRPM(), velocity.in(RotationsPerSecond)*60);
}

}

