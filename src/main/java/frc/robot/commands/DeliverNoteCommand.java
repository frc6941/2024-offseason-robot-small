package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class DeliverNoteCommand extends Command {

    private final IntakerSubsystem intakerSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    public DeliverNoteCommand(IntakerSubsystem intakerSubsystem, ShooterSubsystem shooterSubsystem) {
        this.intakerSubsystem = intakerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakerSubsystem.setIntakeRPM(4500);
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem.setIntakeRPM(0);
    }

    @Override
    public boolean isFinished() {
        return !intakerSubsystem.intakerHasNote()&&shooterSubsystem.isShootComplete();
    }
}