package frc.robot.commands;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intaker.Intaker;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;

public class PassCommand extends Command {
    private Shooter shooter;
    private Intaker intaker;

    public PassCommand(Shooter shooter, Intaker intaker) {
        this.shooter = shooter;
        this.intaker = intaker;
        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        shooter.setShooterVelocity(Constants.ShooterConstants.SHOOTERL_PASS_VELOCITY,
                Constants.ShooterConstants.SHOOTERH_PASS_VELOCITY);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intaker.setIntakerDutyCycle(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(0.0);
        intaker.setIntakerDutyCycle(0);
    }
}
