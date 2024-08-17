package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.intaker.intaker;

import java.util.function.DoubleSupplier;

import com.google.errorprone.annotations.OverridingMethodsMustInvokeSuper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterCommand extends Command {
    private final shooter shooter;
    private final intaker intaker;
    private final BeamBreak intakerbeambreakH;
    private final BeamBreak intakerbeambreakL;

    public ShooterCommand(shooter shooter, intaker intaker, BeamBreak intakerbeambreakH, BeamBreak intakerbeambreakL) {

        this.intakerbeambreakL = intakerbeambreakL;
        this.intakerbeambreakH = intakerbeambreakH;
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
