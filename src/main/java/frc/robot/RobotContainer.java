// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.drivers.BeamBreak;
import org.frcteam6941.drivers.Gyro;
import org.frcteam6941.looper.UpdateManager;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.team254.lib.geometry.Rotation2d;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static edu.wpi.first.units.Units.Seconds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.commands.intakercommand;
import frc.robot.commands.intakerout;
import frc.robot.commands.shootercommand;
import frc.robot.commands.shooteramp;
import frc.robot.commands.rumblecommand;
import frc.robot.subsystems.intaker.intaker;
import frc.robot.subsystems.limelight.Limelight;
import frc.robot.subsystems.shooter.shooter;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.LED.LED;
import frc.robot.commands.LEDcommand;
import frc.robot.commands.autoshoot;

import lombok.Getter;

import org.frcteam6941.drivers.Pigeon2Gyro;

public class RobotContainer {
    private final double initial_distance_to_volatge_list[] = {0.0 , 0.95 , 0.95};
    private final double final_distance_to_voltage_list[] = {1.0 , 0.5, 0.95};
    private BeamBreak intakerBeamBreak = new BeamBreak(3 , 4);
    private ChoreoTrajectory traj = Choreo.getTrajectory("NewPathCircle");
    private LED led = new LED();

    
    Swerve swerve = Swerve.getInstance();
    intaker intaker = new intaker(Constants.IntakerConstants.INTAKE_MOTOR_ID, Constants.RobotConstants.CAN_BUS_NAME);
    shooter shooter = new shooter(Constants.ShooterConstants.SHOOTER_MOTORH_ID,
            Constants.ShooterConstants.SHOOTER_MOTORL_ID, Constants.RobotConstants.CAN_BUS_NAME);
    int cnt = 0;

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
                SmartDashboard.putNumber("Distance", Math.abs(Math.sqrt(
                        (-Constants.RobotConstants.driverController.getLeftY()* Constants.SwerveDrivetrian.maxSpeed.magnitude())*(-Constants.RobotConstants.driverController.getLeftY()* Constants.SwerveDrivetrian.maxSpeed.magnitude()
                        +(-Constants.RobotConstants.driverController.getLeftX() * Constants.SwerveDrivetrian.maxSpeed.magnitude()*(-Constants.RobotConstants.driverController.getLeftX() * Constants.SwerveDrivetrian.maxSpeed.magnitude()))))));

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

        
        Constants.RobotConstants.driverController.start().onTrue(Commands.runOnce(() -> {
            edu.wpi.first.math.geometry.Rotation2d a = swerve.getLocalizer().getLatestPose().getRotation();
            Pose2d b = new Pose2d(new Translation2d(0, 0), a);
            swerve.resetPose(b);
        }));
        Constants.RobotConstants.driverController.rightBumper().onTrue(
                Commands.sequence(
                        Commands.parallel(
                                new rumblecommand(Seconds.of(0.5), Constants.RobotConstants.driverController.getHID()),
                                new intakercommand(intaker, shooter, 0.6, intakerBeamBreak)),
                        new rumblecommand(Seconds.of(1), Constants.RobotConstants.driverController.getHID())));
        Constants.RobotConstants.driverController.leftBumper().whileTrue(new shootercommand(shooter, intaker, 0.95 , 0.95));
        Constants.RobotConstants.driverController.leftTrigger().whileTrue(
                new autoshoot(shooter, intaker, 0.95, (final_distance_to_voltage_list[1]-initial_distance_to_volatge_list[1])*Math.abs(Limelight.getInstance().getSpeakerRelativePosition().getNorm())));
        Constants.RobotConstants.driverController.y().whileTrue(new shooteramp(shooter, intaker, 0.25 , 0.25));
        Constants.RobotConstants.driverController.a().onTrue(new LEDcommand(led));
        
       

    }

    public Command getAutonomousCommand() {
        return Choreo.choreoSwerveCommand(
                traj,
                () -> swerve.getLocalizer().getCoarseFieldPose(0),
                new PIDController(
                        Constants.AutoConstants.swerveXGainsClass.swerveX_KP.get(),
                        Constants.AutoConstants.swerveXGainsClass.swerveX_KI.get(),
                        Constants.AutoConstants.swerveXGainsClass.swerveX_KD.get()),
                new PIDController(
                        Constants.AutoConstants.swerveYGainsClass.swerveY_KP.get(),
                        Constants.AutoConstants.swerveYGainsClass.swerveY_KI.get(),
                        Constants.AutoConstants.swerveYGainsClass.swerveY_KD.get()),
                new PIDController(
                        Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KP.get(),
                        Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KI.get(),
                        Constants.AutoConstants.swerveOmegaGainsClass.swerveOmega_KD.get()),
                (ChassisSpeeds speeds) ->
                        swerve.autoDrive(
                                new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                                speeds.omegaRadiansPerSecond,
                                true,
                                false),
                () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                });
        
    }
   
}
