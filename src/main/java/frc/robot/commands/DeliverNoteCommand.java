package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intaker.Intaker;
import frc.robot.subsystems.Intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter;

public class DeliverNoteCommand extends Command {

    private final IntakerSubsystem intaker;



    public DeliverNoteCommand(IntakerSubsystem intaker) {
        this.intaker = intaker;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intaker.setWantedState(IntakerSubsystem.WantedState.FEED);
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