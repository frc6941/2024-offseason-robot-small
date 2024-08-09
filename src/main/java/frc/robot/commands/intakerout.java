package frc.robot.commands;

import frc.robot.subsystems.intaker.*;
import edu.wpi.first.wpilibj2.command.Command;

public class intakerout extends Command {
    private final intaker intaker;
    private final shooter shooter;

    public intakerout(intaker intaker , shooter shooter) {
        this.intaker = intaker;
        this.shooter = shooter;
        addRequirements(intaker , shooter);
    }

    @Override
    public void execute() {
        intaker.setintaker(-0.7);
        shooter.setshooter(0);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setintaker(0);
    }

}
