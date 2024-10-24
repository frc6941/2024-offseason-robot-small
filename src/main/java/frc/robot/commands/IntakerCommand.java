package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.intaker.*;
import frc.robot.subsystems.shooter.*;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakerCommand extends Command {

    private final Intaker intaker;
    private final Shooter shooter;
    private final BeamBreak intakerbeambreakH;
    private final BeamBreak intakerbeambreakL;

    public IntakerCommand(Intaker intaker, Shooter shooter, BeamBreak intakerbeambreakH , BeamBreak intakerbeambreakL) {
        this.intaker = intaker;
        this.shooter = shooter;
        this.intakerbeambreakH = intakerbeambreakH;
        this.intakerbeambreakL = intakerbeambreakL;
        addRequirements(intaker, shooter);
    }

    @Override
    public void execute() {
        intaker.setIntakerDutyCycle(Constants.IntakerConstants.INTAKER_IN_SPEED);
        shooter.setShooterVelocity(Constants.ShooterConstants.SHOOTER_IDLE_VELOCITY);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setIntakerDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return (intakerbeambreakH.get() || intakerbeambreakL.get());
    }
}

