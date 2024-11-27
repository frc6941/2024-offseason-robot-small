package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakerSubsystem;

import static edu.wpi.first.units.Units.*;

public class IntakeCommand extends Command {

    private final IntakerSubsystem intakerSubsystem;

    public IntakeCommand(IntakerSubsystem intakerSubsystem) {
        this.intakerSubsystem = intakerSubsystem;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        intakerSubsystem.setIntakeRPM(1000);
    }

    @Override
    public void end(boolean interrupted) {
        intakerSubsystem.setIntakeDirectVoltage(Volts.of(0));
    }

    @Override
    public boolean isFinished() {
        return intakerSubsystem.intakerHasNote();
    }
}