package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.intaker.intaker;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {
    private shooter shooter;
    private intaker intaker;

    public ShooterCommand(shooter shooter, intaker intaker) {
        this.shooter = shooter;
        this.intaker = intaker;
        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        shooter.setShooterDutyCycle(Constants.ShooterConstants.SHOOTERL_SPEAKER_SPEED,
                Constants.ShooterConstants.SHOOTERH_SPEAKER_SPEED);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intaker.setIntakerDutyCycle(Constants.IntakerConstants.INTAKER_SHOOT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterDutyCycle(0, 0);
        intaker.setIntakerDutyCycle(0);
    }
}
