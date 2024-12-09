package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intaker.Intaker;
import frc.robot.subsystems.Shooter;

public class DeliverNoteCommand extends Command {

    private final Intaker intaker;
    private final Shooter shooter;

    public DeliverNoteCommand(Intaker intaker, Shooter shooter) {
        this.intaker = intaker;
        this.shooter = shooter;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intaker.setIntakeRPM(6000);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setIntakeRPM(0);
    }

    @Override
    public boolean isFinished() {
        return !intaker.intakerHasNote() && shooter.isShootComplete();
    }
}