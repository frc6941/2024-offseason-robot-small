package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.intaker.*;
import frc.robot.subsystems.shooter.*;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class intakercommand extends Command {

    private final intaker intaker;
    private final shooter shooter;
    private final BeamBreak intakerbeambreakH;
    private final BeamBreak intakerbeambreakL;

    public intakercommand(intaker intaker, shooter shooter, BeamBreak intakerbeambreakH , BeamBreak intakerbeambreakL) {
        this.intaker = intaker;
        this.shooter = shooter;
        this.intakerbeambreakH = intakerbeambreakH;
        this.intakerbeambreakL = intakerbeambreakL;
        addRequirements(intaker, shooter);
    }

    @Override
    public void execute() {
        intaker.setintaker(Constants.IntakerConstants.INTAKER_IN_SPEED);
        shooter.setshooter(Constants.ShooterConstants.SHOOTER_IDLE_SPEED, Constants.ShooterConstants.SHOOTER_IDLE_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setintaker(0);
    }

    @Override
    public boolean isFinished() {
        return (intakerbeambreakH.get() || intakerbeambreakL.get());
    }
}
