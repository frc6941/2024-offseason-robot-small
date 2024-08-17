package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.display.OperatorDashboard;
import frc.robot.subsystems.intaker.*;
import frc.robot.subsystems.shooter.*;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakerOut extends Command {
    private final intaker intaker;
    private final shooter shooter;

    public IntakerOut(intaker intaker, shooter shooter) {
        this.intaker = intaker;
        this.shooter = shooter;
        addRequirements(intaker, shooter);
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
        OperatorDashboard.getInstance().updateRobotStatus(
                false,
                false);
    }

}
