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
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.Utils;
import org.frcteam6941.looper.UpdateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.Seconds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.intakercommand;
import frc.robot.commands.intakerout;
import frc.robot.commands.ledcommand;
// import frc.robot.commands.passcommand;
import frc.robot.commands.shootercommand;
import frc.robot.commands.shooteramp;
import frc.robot.commands.rumblecommand;
import frc.robot.subsystems.intaker.intaker;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.led.led;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private BeamBreak intakerBeamBreakH = new BeamBreak(3);
    private BeamBreak intakerBeamBreakL = new BeamBreak(2);
    Swerve swerve = Swerve.getInstance();
    intaker intaker = new intaker(Constants.IntakerConstants.INTAKE_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    shooter shooter = new shooter(Constants.ShooterConstants.SHOOTER_MOTORH_ID,
            Constants.ShooterConstants.SHOOTER_MOTORL_ID,
            Constants.RobotConstants.CAN_BUS_NAME);
    led led = new led();
    Limelight limelight = Limelight.getInstance();
    Display display = Display.getInstance();
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
        //TODO: register commands
//        NamedCommands.registerCommand("AutoShoot", speakerAutoShot().withTimeout(2.0));
        AutoBuilder.configureHolonomic(
                () -> Swerve.getInstance().getLocalizer().getCoarseFieldPose(0),
                (Pose2d pose2d) -> Swerve.getInstance().resetPose(pose2d),
                () -> Swerve.getInstance().getChassisSpeeds(),
                (ChassisSpeeds chassisSpeeds) -> Swerve.getInstance().driveSpeed(chassisSpeeds),
                new HolonomicPathFollowerConfig(
                        Constants.SwerveConstants.maxSpeed.magnitude(),
                        0.55,
                        new ReplanningConfig()),
//                new HolonomicPathFollowerConfig(
//                        new PIDConstants(
//                                Constants.AutoConstants.swerveXGainsClass.swerveX_KP.get(),
//                                Constants.AutoConstants.swerveXGainsClass.swerveX_KI.get(),
//                                Constants.AutoConstants.swerveXGainsClass.swerveX_KD.get()
//                        ),
//                        new PIDConstants(
//                                Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KP.get(),
//                                Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KI.get(),
//                                Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KD.get()
//                        ),
//                        Constants.SwerveConstants.maxSpeed.magnitude(),
//                        0.55,
//                        new ReplanningConfig()),
                Utils::flip,
                swerve
        );

        autoChooser = new LoggedDashboardChooser<>("Chooser", AutoBuilder.buildAutoChooser());
        //TODO: operator dashboard
//        dashboard.registerAutoSelector(autoChooser.getSendableChooser());
    }
    /** Bind controller keys to commands */
    private void configureBindings() {
        // Drive mode 1
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

        // Drive mode 2
        // swerve.setDefaultCommand(Commands
        // .runOnce(() -> swerve.drive(
        // new Translation2d(
        // -
        // Constants.RobotConstants.driverController.getLeftY()*Constants.SwerveConstants.maxSpeed.magnitude(),
        // -
        // Constants.RobotConstants.driverController.getRightX()*Constants.SwerveConstants.maxSpeed.magnitude()),
        // (-Constants.RobotConstants.driverController.getRightTriggerAxis()
        // + Constants.RobotConstants.driverController.getLeftTriggerAxis())
        // * Constants.SwerveConstants.maxAngularRate.magnitude(),
        // true,
        // true),
        // swerve));
        // }

        // 1111111111111111
        Constants.RobotConstants.driverController.start().onTrue(
                Commands.runOnce(() -> {
                    swerve.resetHeadingController();
                    swerve.resetPose(
                            new Pose2d(AllianceFlipUtil.apply(Constants.FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()),
                                    Rotation2d.fromDegrees(swerve.getLocalizer().getLatestPose().getRotation().getDegrees())));
                }));
        Constants.RobotConstants.driverController.rightBumper().onTrue(
                Commands.sequence(
                        Commands.parallel(
                                new ledcommand(led, 255, 0, 0),
                                new intakercommand(intaker, shooter, intakerBeamBreakH, intakerBeamBreakL),
                                new rumblecommand(Seconds.of(0.5),
                                        Constants.RobotConstants.driverController.getHID())),
                        Commands.parallel(
                                new ledcommand(led, 0, 255, 0),
                                new rumblecommand(Seconds.of(1),
                                        Constants.RobotConstants.driverController.getHID()))));

        Constants.RobotConstants.driverController.leftBumper().whileTrue(
                Commands.parallel(
                        new shootercommand(shooter, intaker),
                        new ledcommand(led, 0, 0, 0)));

        Constants.RobotConstants.driverController.b().whileTrue(new intakerout(intaker, shooter));

        Constants.RobotConstants.driverController.y().whileTrue(
                Commands.parallel(
                        new shooteramp(shooter, intaker),
                        new ledcommand(led, 0, 0, 0)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
