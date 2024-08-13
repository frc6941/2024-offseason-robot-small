package frc.robot.commands;

import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.intaker.intaker;

import edu.wpi.first.wpilibj2.command.Command;

public class passcommand extends Command {
    private shooter shooter;
    private intaker intaker;

    public passcommand(shooter shooter, intaker intaker) {
        this.shooter = shooter;
        this.intaker = intaker;
        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        shooter.setshooter(0.95, 0.85);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intaker.setintaker(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setshooter(0, 0);
        intaker.setintaker(0);
    }
}
