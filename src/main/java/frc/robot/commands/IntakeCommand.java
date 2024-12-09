package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intaker.Intaker;

public class IntakeCommand extends Command {

    private final Intaker intaker;

    public IntakeCommand(Intaker intaker) {
        this.intaker = intaker;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intaker.setIntakeRPM(1500);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setIntakeRPM(0);
    }

    @Override
    public boolean isFinished() {
        return intaker.intakerHasNote();
    }
}