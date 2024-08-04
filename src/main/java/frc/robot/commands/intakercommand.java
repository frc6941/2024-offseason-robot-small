package frc.robot.commands;

import frc.robot.subsystems.intaker.*;
import frc.robot.subsystems.shooter.*;
import frc.robot.drivers.BeamBreak;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class intakercommand extends Command {
    private BeamBreak intakerBeamBreak = new BeamBreak(3);
    private final intaker intaker;
    private final double Voltage;
    private final shooter shooter;
    // private final BooleanSupplier isIntakerBeamBreakOn;

    public intakercommand(intaker intaker, shooter shooter, double Voltage) {
        this.intaker = intaker;
        this.Voltage = Voltage;
        this.shooter = shooter;
        addRequirements(intaker, shooter);
    }

    @Override
    public void execute() {
        intaker.setintaker(Voltage);
        shooter.setshooter(0.5);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setintaker(0);
    }

    @Override
    public boolean isFinished() {
        return intakerBeamBreak.get();
    }
}
