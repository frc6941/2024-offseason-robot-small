// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.display.Display;
import frc.robot.display.OperatorDashboard;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.ShootingDecider;
import frc.robot.utils.Utils;
import frc.robot.utils.ShootingDecider.Destination;
import frc.robot.commands.ledPattern.BlinkLight;
import frc.robot.commands.ledPattern.ConstLight;
import frc.robot.commands.ledPattern.ConstLight;
import frc.robot.drivers.BeamBreak;
import org.frcteam6941.looper.UpdateManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.Seconds;
import java.util.function.Supplier;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.IntakerOut;
import frc.robot.commands.FlyWheelRampUp;
import frc.robot.commands.IntakerCommand;
import frc.robot.commands.PassCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterAmp;
import frc.robot.commands.RumbleCommand;
import frc.robot.subsystems.intaker.intaker;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.led.led;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private BeamBreak intakerBeamBreakH = new BeamBreak(3);
    private BeamBreak intakerBeamBreakL = new BeamBreak(2);
    Supplier<ShootingDecider.Destination> destinationSupplier;
    Swerve swerve = Swerve.getInstance();
    intaker intaker = new intaker(Constants.IntakerConstants.INTAKE_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    shooter shooter = new shooter(Constants.ShooterConstants.SHOOTER_MOTORH_ID,
            Constants.ShooterConstants.SHOOTER_MOTORL_ID,
            Constants.RobotConstants.CAN_BUS_NAME);
    led led = new led();
    Limelight limelight = Limelight.getInstance();
    Display display = Display.getInstance();
    OperatorDashboard dashboard = OperatorDashboard.getInstance();
    @Getter
    private LoggedDashboardChooser<Command> autoChooser;

    @Getter
    private UpdateManager updateManager;

    public RobotContainer() {

        updateManager = new UpdateManager(swerve,
                limelight,
                display);
        updateManager.registerAll();

        configureAuto();
        configureBindings();
        System.out.println("Init Completed!");
    }



    /** Bind Auto */
    private void configureAuto() {

        NamedCommands.registerCommand("shoot", shoot().withTimeout(0.2));
        NamedCommands.registerCommand("intake", intake());
        AutoBuilder.configureHolonomic(
                () -> Swerve.getInstance().getLocalizer().getCoarseFieldPose(0),
                (Pose2d pose2d) -> Swerve.getInstance().resetPose(pose2d),
                () -> Swerve.getInstance().getChassisSpeeds(),
                (ChassisSpeeds chassisSpeeds) -> Swerve.getInstance().driveSpeed(chassisSpeeds),
                new HolonomicPathFollowerConfig(
                        Constants.SwerveConstants.maxSpeed.magnitude(),
                        0.55,
                        new ReplanningConfig()),
                // new HolonomicPathFollowerConfig(
                // new PIDConstants(
                // Constants.AutoConstants.swerveXGainsClass.swerveX_KP.get(),
                // Constants.AutoConstants.swerveXGainsClass.swerveX_KI.get(),
                // Constants.AutoConstants.swerveXGainsClass.swerveX_KD.get()
                // ),
                // new PIDConstants(
                // Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KP.get(),
                // Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KI.get(),
                // Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KD.get()
                // ),
                // Constants.SwerveConstants.maxSpeed.magnitude(),
                // 0.55,
                // new ReplanningConfig()),
                Utils::flip,
                swerve);

        autoChooser = new LoggedDashboardChooser<>("Chooser", AutoBuilder.buildAutoChooser());
        // TODO: operator dashboard
        // dashboard.registerAutoSelector(autoChooser.getSendableChooser());
    }

    /** Bind controller keys to commands */
    private void configureBindings() {
        if(intakerBeamBreakL.get()||intakerBeamBreakH.get()) {
            OperatorDashboard.getInstance().updateRobotStatus(
                    false,
                    true)
            ;
        }
        // swerve
        swerve.setDefaultCommand(Commands
                .runOnce(() -> swerve.drive(
                        new Translation2d(
                                -Constants.RobotConstants.driverController.getLeftY()
                                        * Constants.SwerveConstants.maxSpeed.magnitude(),
                                -Constants.RobotConstants.driverController.getLeftX()
                                        * Constants.SwerveConstants.maxSpeed.magnitude()),
                        -Constants.RobotConstants.driverController.getRightX()
                                * Constants.SwerveConstants.maxAngularRate.magnitude(),
                        true,
                        false),
                        swerve));

        Constants.RobotConstants.driverController.start().onTrue(
                Commands.runOnce(() -> {
                    swerve.resetHeadingController();
                    swerve.resetPose(
                            new Pose2d(
                                    AllianceFlipUtil.apply(
                                            Constants.FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()),
                                    Rotation2d.fromDegrees(
                                            swerve.getLocalizer().getLatestPose().getRotation().getDegrees())));
                }));

        // intake
        Constants.RobotConstants.driverController.rightBumper().onTrue(
                Commands.sequence(
                        Commands.parallel(
                                new ConstLight(led, 255, 0, 0),
                                new IntakerCommand(intaker, shooter, intakerBeamBreakH, intakerBeamBreakL),
                                new RumbleCommand(Seconds.of(0.5),
                                        Constants.RobotConstants.driverController.getHID())),
                        Commands.parallel(
                                new RumbleCommand(Seconds.of(1),
                                        Constants.RobotConstants.driverController.getHID()),
                                Commands.sequence(
                                        new BlinkLight(led, Seconds.of(1), 0, 255, 0),
                                        new ConstLight(led, 0, 255, 0)))));

        // shoot speaker
        Constants.RobotConstants.driverController.leftBumper().whileTrue(
                Commands.parallel(
                        new ShooterCommand(shooter, intaker, intakerBeamBreakH, intakerBeamBreakL),
                        new ConstLight(led, 0, 0, 0)));

        // intake out
        Constants.RobotConstants.driverController.a().whileTrue(
                Commands.sequence(new FlyWheelRampUp(intaker, shooter, () -> Destination.FERRY)));
        Constants.RobotConstants.driverController.b().whileTrue(Commands.parallel(
                new IntakerOut(intaker, shooter),
                new ConstLight(led, 0, 0, 0)));

        // shoot amp
        Constants.RobotConstants.driverController.y().whileTrue(
                Commands.parallel(
                        new ShooterAmp(shooter, intaker),
                        new ConstLight(led, 0, 0, 0)));

        Constants.RobotConstants.driverController.x().whileTrue(
                Commands.parallel(
                        new PassCommand(shooter, intaker),
                        new ConstLight(led, 0, 0, 0)));
    }

    public Command getAutonomousCommand() {
        // return autoChooser.get();
        return AutoBuilder.buildAuto("test");

    }

    // commands
    private Command shoot() {
        return new ShooterCommand(shooter, intaker, intakerBeamBreakH, intakerBeamBreakL);
    }

    private Command intake() {
        return new IntakerCommand(intaker, shooter, intakerBeamBreakH, intakerBeamBreakL);
    }
}
