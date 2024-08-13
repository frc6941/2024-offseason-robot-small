// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.drivers.BeamBreak;
import org.frcteam6941.drivers.Gyro;
import org.frcteam6941.looper.UpdateManager;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

import org.frcteam6941.drivers.Pigeon2Gyro;

public class RobotContainer {
    private BeamBreak intakerBeamBreakH = new BeamBreak(3);
    private BeamBreak intakerBeamBreakL = new BeamBreak(2);
    Swerve swerve = Swerve.getInstance();
    intaker intaker = new intaker(Constants.IntakerConstants.INTAKE_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    shooter shooter = new shooter(Constants.ShooterConstants.SHOOTER_MOTORH_ID,
            Constants.ShooterConstants.SHOOTER_MOTORL_ID,
            Constants.RobotConstants.CAN_BUS_NAME);
    led led = new led();

    @Getter
    private UpdateManager updateManager;

    public RobotContainer() {
        updateManager = new UpdateManager(swerve);
        updateManager.registerAll();

        configureBindings();
        System.out.println("Init Completed!");
    }

    /** Bind controller keys to commands */
    private void configureBindings() {
        // Drive mode 1
        swerve.setDefaultCommand(Commands
                .runOnce(() -> swerve.drive(
                        new Translation2d(
                                -Constants.RobotConstants.driverController.getLeftY()
                                        * Constants.SwerveDrivetrian.maxSpeed.magnitude(),
                                -Constants.RobotConstants.driverController.getLeftX()
                                        * Constants.SwerveDrivetrian.maxSpeed.magnitude()),
                        -Constants.RobotConstants.driverController.getRightX()
                                * Constants.SwerveDrivetrian.maxAngularRate.magnitude(),
                        true,
                        false),
                        swerve));

        // Drive mode 2
        // swerve.setDefaultCommand(Commands
        // .runOnce(() -> swerve.drive(
        // new Translation2d(
        // -
        // Constants.RobotConstants.driverController.getLeftY()*Constants.SwerveDrivetrian.maxSpeed.magnitude(),
        // -
        // Constants.RobotConstants.driverController.getRightX()*Constants.SwerveDrivetrian.maxSpeed.magnitude()),
        // (-Constants.RobotConstants.driverController.getRightTriggerAxis()
        // + Constants.RobotConstants.driverController.getLeftTriggerAxis())
        // * Constants.SwerveDrivetrian.maxAngularRate.magnitude(),
        // true,
        // true),
        // swerve));
        // }

        // 1111111111111111
        Constants.RobotConstants.driverController.start().onTrue(
                Commands.runOnce(() -> {
                    edu.wpi.first.math.geometry.Rotation2d a = swerve.getLocalizer().getLatestPose().getRotation();
                    System.out.println("A = " + a);
                    Pose2d b = new Pose2d(new Translation2d(0, 0), a);
                    swerve.resetPose(b);
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
