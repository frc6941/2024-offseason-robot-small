package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.intaker.intaker;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterCommand extends Command {
    private final shooter shooter;
    private final intaker intaker;

    public ShooterCommand(shooter shooter, intaker intaker) {
        this.shooter = shooter;
        this.intaker = intaker;

        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        shooter.setShooterVelocity(Constants.ShooterConstants.SHOOTERL_SPEAKER_VELOCITY,
                Constants.ShooterConstants.SHOOTERH_SPEAKER_VELOCITY);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intaker.setIntakerDutyCycle(Constants.IntakerConstants.INTAKER_SHOOT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(0.0);
        intaker.setIntakerDutyCycle(0);
    }

}
