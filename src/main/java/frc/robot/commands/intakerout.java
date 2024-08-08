package frc.robot.commands;

import frc.robot.subsystems.intaker.*;
import edu.wpi.first.wpilibj2.command.Command;

public class intakerout extends Command {
    private final intaker intaker;

    public intakerout(intaker intaker) {
        this.intaker = intaker;
        addRequirements(intaker);
    }

    @Override
    public void execute() {
        intaker.setintaker(-0.7);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setintaker(0);
    }

}
