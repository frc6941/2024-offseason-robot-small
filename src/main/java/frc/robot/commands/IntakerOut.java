package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.intaker.*;
import frc.robot.subsystems.shooter.*;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakerOut extends Command {
    private final Intaker intaker;
    private final Shooter shooter;

    public IntakerOut(Intaker intaker , Shooter shooter) {
        this.intaker = intaker;
        this.shooter = shooter;
        addRequirements(intaker , shooter);
    }

    @Override
    public void execute() {
        intaker.setIntakerDutyCycle(Constants.IntakerConstants.INTAKER_OUT_SPEED);
        shooter.setShooterVelocity(Constants.ShooterConstants.SHOOTER_OUT_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setIntakerDutyCycle(0);
        shooter.setShooterVelocity(0);
    }

    

}
