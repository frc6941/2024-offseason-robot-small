package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Intaker.Intaker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShootingDecider;

import java.util.function.DoubleSupplier;

public class SpeakerShootCommand extends ParallelCommandGroup {
    public SpeakerShootCommand(
            Shooter shooter,
            Intaker intakeSubsystem,
            Swerve Swerve,
            DoubleSupplier driverX,
            DoubleSupplier driverY) {
        addCommands(
                Commands.deadline(
                        Commands.sequence(
                                new WaitUntilCommand(() -> {
                                    boolean swerveReady = Swerve.aimingReady(10);
                                    boolean shooterReady = shooter.ShooterVelocityReady();
                                    boolean distanceReady = Swerve.getLocalizer().getCoarseFieldPose(0).getTranslation().minus(
                                            FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()
                                    ).getNorm() < 1.5;

                                    return swerveReady && shooterReady;
                                }),
                                Commands.runOnce(() -> Timer.delay(0.02)),
                                new DeliverNoteCommand(intakeSubsystem, shooter)),
                        new ChassisAimCommand(Swerve, () -> ShootingDecider.Destination.SPEAKER, driverX, driverY),
                        new FlyWheelRampUp(shooter)

                ));

    }

}
