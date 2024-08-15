package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.intaker.intaker;

import edu.wpi.first.wpilibj2.command.Command;

public class shootercommand extends Command {
    private shooter shooter;
    private intaker intaker;

    public shootercommand(shooter shooter, intaker intaker) {
        this.shooter = shooter;
        this.intaker = intaker;
        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        shooter.setshooter(Constants.ShooterConstants.SHOOTERL_SPEAKER_SPEED,
                Constants.ShooterConstants.SHOOTERH_SPEAKER_SPEED);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intaker.setintaker(Constants.IntakerConstants.INTAKER_SHOOT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setshooter(0, 0);
        intaker.setintaker(0);
    }
}
