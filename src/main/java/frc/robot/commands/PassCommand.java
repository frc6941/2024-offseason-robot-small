package frc.robot.commands;

import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.intaker.intaker;
import frc.robot.Constants;

import edu.wpi.first.wpilibj2.command.Command;

public class PassCommand extends Command {
    private shooter shooter;
    private intaker intaker;

    public PassCommand(shooter shooter, intaker intaker) {
        this.shooter = shooter;
        this.intaker = intaker;
        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        shooter.setShooterDutyCycle(Constants.ShooterConstants.SHOOTERL_PASS_SPEED,
                Constants.ShooterConstants.SHOOTERH_PASS_SPEED);
        try {
            Thread.sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intaker.setIntakerDutyCycle(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterDutyCycle(0, 0);
        intaker.setIntakerDutyCycle(0);
    }
}
