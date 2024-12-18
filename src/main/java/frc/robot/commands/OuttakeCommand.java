package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intaker.IntakerSubsystem;

public class OuttakeCommand extends Command {

    private final IntakerSubsystem intaker;

    public OuttakeCommand(IntakerSubsystem intaker) {
        this.intaker = intaker;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intaker.setWantedState(IntakerSubsystem.WantedState.OUTTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setWantedState(IntakerSubsystem.WantedState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}