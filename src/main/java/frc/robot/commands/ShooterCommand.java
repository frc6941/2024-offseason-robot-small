package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.intaker.intaker;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class ShooterCommand extends Command {
    private final shooter shooter;
    private final intaker intaker;

    public ShooterCommand(shooter shooter, intaker intaker) {
        
        
        
        this.shooter = shooter;
        this.intaker = intaker;
        
        addRequirements(shooter, intaker);
    }
    // public SpeakerShootCommand(
    //         ShooterSubsystem shooterSubsystem,
    //         ArmSubsystem armSubsystem,
    //         IndexerSubsystem indexerSubsystem,
    //         BeamBreakSubsystem beamBreakSubsystem,
    //         IndicatorSubsystem indicatorSubsystem,
    //         Swerve Swerve,
    //         DoubleSupplier driverX,
    //         DoubleSupplier driverY,
    //         boolean isAuto) {
    //     addCommands(
    //             new ChassisAimCommand(Swerve, () -> Destination.SPEAKER, driverX, driverY, isAuto),
    //             new ArmAimCommand(armSubsystem, () -> Destination.SPEAKER),
    //             new FlyWheelRampUp(shooterSubsystem, () -> Destination.SPEAKER),
    //             Commands.sequence(
    //                     new WaitUntilCommand(() -> {
    //                         boolean swerveReady = Swerve.aimingReady(2.5);
    //                         boolean shooterReady = shooterSubsystem.ShooterVelocityReady();
    //                         boolean armReady = armSubsystem.armAimingReady();
    //                         return swerveReady && shooterReady && armReady;
    //                     }),
    //                     Commands.runOnce(() -> Timer.delay(0.02), indicatorSubsystem),
    //                     new DeliverNoteCommand(indexerSubsystem, beamBreakSubsystem, indicatorSubsystem)));
    // }

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
