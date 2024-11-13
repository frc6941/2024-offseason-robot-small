// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.basics.AutoActions;
import frc.robot.display.Display;
import frc.robot.display.OperatorDashboard;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.apriltagvision.AprilTagVisionIONorthstar;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.ShootingDecider;
import lombok.Getter;
import org.frcteam6941.looper.UpdateManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.function.Supplier;


public class RobotContainer {
    Supplier<ShootingDecider.Destination> destinationSupplier;
    @Getter
    AprilTagVision aprilTagVision = new AprilTagVision(
            this::getAprilTagLayoutType,
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 0),
            new AprilTagVisionIONorthstar(this::getAprilTagLayoutType, 1));
    Swerve swerve = Swerve.getInstance();
    Display display = Display.getInstance();
    OperatorDashboard dashboard = OperatorDashboard.getInstance();
    private double distance;
    @Getter
    private LoggedDashboardChooser<Command> autoChooser;

    @Getter
    private UpdateManager updateManager;

    public RobotContainer() {
        updateManager = new UpdateManager(swerve,
                display);
        updateManager.registerAll();

        configureBindings();
        System.out.println("Init Completed!");
    }


    /**
     * Bind controller keys to commands
     */
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
                                            new Translation2d(0, 0)),
                                    Rotation2d.fromDegrees(
                                            swerve.getLocalizer().getLatestPose().getRotation().getDegrees())));
                }).ignoringDisable(true));
    }


    public Command getAutonomousCommand() {
//         return autoChooser.get();
        return new SequentialCommandGroup(
                AutoActions.waitFor(0.000001),
                AutoActions.followTrajectory(AutoActions.getTrajectory("T_1"), true, true),
                AutoActions.followTrajectory(AutoActions.getTrajectory("T_2"), true, true)
        );
//        return new CharacterizationDriveCommand(swerve, 2, 1.5, 5);

    }

    /**
     * Returns the current AprilTag layout type.
     */
    public Constants.FieldConstants.AprilTagLayoutType getAprilTagLayoutType() {
//        if (aprilTagsSpeakerOnly.getAsBoolean()) {
//            return FieldConstants.AprilTagLayoutType.SPEAKERS_ONLY;
//        } else if (aprilTagsAmpOnly.getAsBoolean()) {
//            return FieldConstants.AprilTagLayoutType.AMPS_ONLY;
//        } else {
        return Constants.FieldConstants.defaultAprilTagType;
//        }
    }
}
