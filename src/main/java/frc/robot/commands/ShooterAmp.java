package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intaker.Intaker;
// import frc.robot.drivers.BeamBreak;

import edu.wpi.first.wpilibj2.command.Command;

public class ShooterAmp extends Command {
    private Shooter shooter;
    private Intaker intaker;

    public ShooterAmp(Shooter shooter, Intaker intaker) {
        this.shooter = shooter;
        this.intaker = intaker;
        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        shooter.setShooterVelocity(Constants.ShooterConstants.SHOOTERL_AMP_VELOCITY,Constants.ShooterConstants.SHOOTERH_AMP_VELOCITY);
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

