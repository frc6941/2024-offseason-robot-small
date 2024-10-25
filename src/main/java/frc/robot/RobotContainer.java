// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import static edu.wpi.first.units.Units.Seconds;
import org.frcteam6941.looper.UpdateManager;
import lombok.Getter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.function.Supplier;

import frc.robot.display.Display;
import frc.robot.display.OperatorDashboard;
import frc.robot.drivers.BeamBreak;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.ShootingDecider;
import frc.robot.subsystems.swerve.Swerve;


public class RobotContainer {
    private double distance;
    Supplier<ShootingDecider.Destination> destinationSupplier;
    Swerve swerve = Swerve.getInstance();
    Display display = Display.getInstance();
    OperatorDashboard dashboard = OperatorDashboard.getInstance();
    @Getter
    private LoggedDashboardChooser<Command> autoChooser;

    @Getter
    private UpdateManager updateManager;

    public RobotContainer() {
        updateManager = new UpdateManager(swerve,
                display);
        updateManager.registerAll();

        configureAuto();
        configureBindings();
        System.out.println("Init Completed!");
    }

    /** Bind Auto */
    private void configureAuto() {

        // AutoBuilder.configureHolonomic(
        //         () -> Swerve.getInstance().getLocalizer().getCoarseFieldPose(0),
        //         (Pose2d pose2d) -> Swerve.getInstance().resetPose(pose2d),
        //         () -> Swerve.getInstance().getChassisSpeeds(),
        //         (ChassisSpeeds chassisSpeeds) -> Swerve.getInstance().driveSpeed(chassisSpeeds),
        //         new HolonomicPathFollowerConfig(
        //                 Constants.SwerveConstants.maxSpeed.magnitude(),
        //                 0.55,
        //                 new ReplanningConfig()),
        //         Utils::flip,
        //         swerve);

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

        // initial
        Constants.RobotConstants.driverController.start().onTrue(
                Commands.runOnce(() -> {
                    swerve.resetHeadingController();
                    swerve.resetPose(
                            new Pose2d(
                                    AllianceFlipUtil.apply(
                                            Constants.FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()),
                                    Rotation2d.fromDegrees(
                                            swerve.getLocalizer().getLatestPose().getRotation().getDegrees())));
                }).ignoringDisable(true));
        }


    public Command getAutonomousCommand() {
        // return autoChooser.get();
        return null;

    }
}
