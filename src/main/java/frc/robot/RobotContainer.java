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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.display.Display;
import frc.robot.display.OperatorDashboard;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.Utils;
import frc.robot.commands.ledPattern.BlinkLight;
import frc.robot.commands.ledPattern.ConstLight;
import org.frcteam6941.looper.UpdateManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.Seconds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.IntakerOut;
import frc.robot.commands.IntakerCommand;
import frc.robot.commands.PassCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterAmp;
import frc.robot.commands.RumbleCommand;
import frc.robot.subsystems.intaker.Intaker;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.led.Led;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    private BeamBreak intakerBeamBreakH = new BeamBreak(3);
    private BeamBreak intakerBeamBreakL = new BeamBreak(2);
    Swerve swerve = Swerve.getInstance();
    Intaker intaker = new Intaker(Constants.IntakerConstants.INTAKE_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    Shooter shooter = new Shooter(Constants.ShooterConstants.SHOOTER_MOTORH_ID,
            Constants.ShooterConstants.SHOOTER_MOTORL_ID,
            Constants.RobotConstants.CAN_BUS_NAME);
    Led led = new Led();
    Display display = Display.getInstance();
    @Getter
    private LoggedDashboardChooser<Command> autoChooser;

    OperatorDashboard dashboard = OperatorDashboard.getInstance();

    VisionSubsystem vision;

    @Getter
    private UpdateManager updateManager;

    public RobotContainer() {
        updateManager = new UpdateManager(
                swerve,
                display);
        updateManager.registerAll();

        configureAuto();
        configureBindings();
        configureSubsystems();
        System.out.println("Init Completed!");
    }

    public void configureSubsystems() {
        if (RobotBase.isReal()) {
            vision = new VisionSubsystem(new VisionIOLimelight());
        } else {
            vision = new VisionSubsystem(new VisionIOLimelight());
        }
    }

    /** Bind Auto */
    private void configureAuto() {
        NamedCommands.registerCommand("shoot", shoot());
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
        SmartDashboard.putBoolean("Swerve/autoConfigured",AutoBuilder.isConfigured());
        SmartDashboard.putBoolean("Swerve/autoPathConfigured",AutoBuilder.isPathfindingConfigured());
        // TODO: operator dashboard
        dashboard.registerAutoSelector(autoChooser.getSendableChooser());
    }

    /** Bind controller keys to commands */
    private void configureBindings() {
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
                        new ShooterCommand(shooter, intaker),
                        new ConstLight(led, 0, 0, 0)));

        // intake out
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
        //return autoChooser.get();
        return AutoBuilder.buildAuto("R1");
    }

    // commands
    private Command shoot() {
        return new ShooterCommand(shooter, intaker);
    }

    private Command intake() {
        return new IntakerCommand(intaker, shooter, intakerBeamBreakH, intakerBeamBreakL);
    }
}
