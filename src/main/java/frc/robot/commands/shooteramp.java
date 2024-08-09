package frc.robot.commands;

import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.intaker.intaker;
// import frc.robot.drivers.BeamBreak;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class shooteramp extends Command {
    private shooter shooter;
    private double VoltageL;
    private double VoltageH;
    private intaker intaker;
    // private BeamBreak intakerBeamBreak = new BeamBreak(3);

    public shooteramp(shooter shooter, intaker intaker, double VoltageL, double VoltageH) {
        this.shooter = shooter;
        this.VoltageL = VoltageL;
        this.VoltageH = VoltageH;
        this.intaker = intaker;
        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        intaker.setintaker(1.0);
        shooter.setshooter(VoltageL , VoltageH);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setshooter(0,0);
        intaker.setintaker(0);
    }
}

