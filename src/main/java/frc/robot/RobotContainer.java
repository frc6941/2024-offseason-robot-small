package frc.robot;

import frc.robot.drivers.BeamBreak;
import org.frcteam6941.looper.UpdateManager;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.subsystems.shooter.shooter;
import lombok.Getter;

public class RobotContainer {
    private BeamBreak intakerBeamBreak = new BeamBreak(3);
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
            System.out.println("A = " + a);
            Pose2d b = new Pose2d(new Translation2d(0, 0), a);
            swerve.resetPose(b);
        }));
        Constants.RobotConstants.driverController.rightBumper().onTrue(
                Commands.sequence(
                        Commands.parallel(
                                new rumblecommand(Seconds.of(0.5), Constants.RobotConstants.driverController.getHID()),
                                new intakercommand(intaker, shooter, 0.6, intakerBeamBreak)),
                        new rumblecommand(Seconds.of(1), Constants.RobotConstants.driverController.getHID())));
        Constants.RobotConstants.driverController.leftBumper().whileTrue(new shootercommand(shooter, intaker, 0.95));
        Constants.RobotConstants.driverController.b().whileTrue(new intakerout(intaker));
        Constants.RobotConstants.driverController.y().whileTrue(new shooteramp(shooter, intaker, 0.25));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
