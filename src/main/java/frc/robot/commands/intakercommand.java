package frc.robot.commands;

import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.intaker.*;
import frc.robot.subsystems.shooter.*;
import edu.wpi.first.wpilibj2.command.Command;

public class intakercommand extends Command {

    private final intaker intaker;
    private final double Voltage;
    private final shooter shooter;
    private final BeamBreak intakerbeambreak;

    public intakercommand(intaker intaker, shooter shooter, double Voltage, BeamBreak intakerbeambreak) {
        this.intaker = intaker;
        this.Voltage = Voltage;
        this.shooter = shooter;
        this.intakerbeambreak = intakerbeambreak;
        addRequirements(intaker, shooter);
    }

    @Override
    public void execute() {
        intaker.setintaker(Voltage);
        shooter.setshooter(0.6);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setintaker(0);
    }

    @Override
    public boolean isFinished() {
        return intakerbeambreak.get();
    }
}
