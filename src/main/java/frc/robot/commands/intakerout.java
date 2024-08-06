package frc.robot.commands;

import frc.robot.subsystems.intaker.*;
import frc.robot.subsystems.shooter.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
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
