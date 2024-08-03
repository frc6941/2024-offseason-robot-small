// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ControllerDriveCommand;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import org.frcteam6941.looper.UpdateManager;

public class RobotContainer {
	Swerve swerve = Swerve.getInstance();
	CommandXboxController driverController = new CommandXboxController(0);
	@Getter
    private UpdateManager updateManager;

    public RobotContainer() {
        updateManager = new UpdateManager(swerve);
        updateManager.registerAll();

        configureBindings();
        System.out.println("Init Completed!");
    }

    /**
     * Bind controller keys to commands
     */
    private void configureBindings() {
        //Drive mode 1
        // swerve.setDefaultCommand(Commands
        //         .runOnce(() -> swerve.drive(
        //                         new Translation2d(
        //                                 -driverController.getLeftY() * Constants.SwerveDrivetrain.maxSpeed.magnitude(),
        //                                 -driverController.getLeftX() * Constants.SwerveDrivetrain.maxSpeed.magnitude()),
        //                         -Constants.RobotConstants.driverController.getRightX() * Constants.SwerveDrivetrain.maxAngularRate.magnitude(),
        //                         true,
        //                         false),
        //                 swerve));
        //Drive mode 2
        // swerve.setDefaultCommand(Commands
        // 		.runOnce(() -> swerve.drive(
        // 				new Translation2d(
        // 						- driverController.getLeftY()*Constants.SwerveDrivetrian.maxSpeed.magnitude(),
        // 						- driverController.getRightX()*Constants.SwerveDrivetrian.maxSpeed.magnitude()),
        // 				(-Constants.RobotConstants.driverController.getRightTriggerAxis()
        // 						+ Constants.RobotConstants.driverController.getLeftTriggerAxis())
        // 						* Constants.SwerveDrivetrian.maxAngularRate.magnitude(),
        // 				true,
        // 				false),
		// 				swerve));
		//Drive test 1
		swerve.setDefaultCommand(new ControllerDriveCommand(swerve,
				() -> -driverController.getLeftY(),
				() -> -driverController.getLeftX(),
				() -> -driverController.getRightX()));
        //Point Wheel
        // swerve.setDefaultCommand(Commands.runOnce(() -> swerve.pointWheelsAt(
        // 		new edu.wpi.first.math.geometry.Rotation2d(
        // 				driverController.getLeftX()*Math.PI/2)),
        // 		swerve));
        //field relative heading
        // driverController.a().
        driverController.a().onTrue(Commands.runOnce(() -> {
            swerve.setLockHeading(true);
            swerve.setHeadingTarget(0.0);
        }, swerve));
        driverController.b().onTrue(Commands.runOnce(() -> swerve.setLockHeading(false), swerve));
        driverController.start().onTrue(Commands.runOnce(() -> {
			swerve.resetHeadingController();
			ControllerDriveCommand.facingAngle = 0.0;
            //Pigeon2 mPigeon2 = new Pigeon2(Constants.SwerveDrivetrian.PIGEON_ID, Constants.RobotConstants.CAN_BUS_NAME);
            edu.wpi.first.math.geometry.Rotation2d a = swerve.getLocalizer().getLatestPose().getRotation();//new edu.wpi.first.math.geometry.Rotation2d(mPigeon2.getYaw().getValueAsDouble());
            //swerve.getGyro().getYaw().;//.getLocalizer().getLatestPose().getRotation();
            System.out.println("A = " + a);
            Pose2d b = new Pose2d(new Translation2d(0, 0), a);
            swerve.resetPose(b);
        }));
    }

    public Command getAutonomousCommand() {
        //return new CharacterizationDriveCommand(swerve, 3, 1.5, 6);
        return Commands.print("No autonomous command configured");
    }
}
