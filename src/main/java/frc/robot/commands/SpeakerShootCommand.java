package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.intaker.IntakerSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;

import java.util.function.DoubleSupplier;

public class SpeakerShootCommand extends ParallelCommandGroup {
    public SpeakerShootCommand(
            Shooter shooter,
            IntakerSubsystem intakeSubsystem,
            Swerve Swerve,
            Arm arm,
            DoubleSupplier driverX,
            DoubleSupplier driverY) {
        addCommands(
                Commands.deadline(
                        Commands.sequence(
                                new WaitUntilCommand(() -> {
                                    boolean swerveReady = Swerve.aimingReady(10);
                                    boolean shooterReady = shooter.ShooterVelocityReady();
                                    boolean armReady = arm.armReady();
//                                    boolean distanceReady = Swerve.getLocalizer().getCoarseFieldPose(0).getTranslation().minus(
//                                            FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()
//                                    ).getNorm() < 1.5;
//                                    SmartDashboard.putBoolean("/AutoAiming/swerve",swerveReady);
//                                    SmartDashboard.putBoolean("/AutoAiming/shooter",shooterReady);
//                                    SmartDashboard.putBoolean("/AutoAiming/arm",armReady);
                                    return swerveReady && shooterReady && armReady ;
                                }),
                                Commands.runOnce(() -> Timer.delay(0.02)),
                                Commands.runOnce(() -> intakeSubsystem.setWantedState(IntakerSubsystem.WantedState.TRIGGER))),
                        new ChassisAimCommand(Swerve, () -> ShootingDecider.Destination.SPEAKER, driverX, driverY),
                        new FlyWheelRampUp(shooter),
                        new ArmAimCommand(arm, () -> ShootingDecider.Destination.SPEAKER)

                ));

    }

}
