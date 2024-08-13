package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.intaker.*;
import frc.robot.subsystems.shooter.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
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
        intaker.setintaker(Constants.IntakerConstants.INTAKER_OUT_SPEED);
        shooter.setshooter(Constants.ShooterConstants.SHOOTER_OUT_SPEED, Constants.ShooterConstants.SHOOTER_OUT_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setintaker(0);
        shooter.setshooter(0,0);
    }

    

}
