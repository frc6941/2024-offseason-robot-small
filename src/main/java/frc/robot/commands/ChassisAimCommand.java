package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;
import frc.robot.utils.shooting.ShootingParameters;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ChassisAimCommand extends Command {
    private final Swerve swerve;
    private final DoubleSupplier driverX;
    private final DoubleSupplier driverY;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;
    private double[] inputBuffer;
    private double[] outputBuffer;


    public ChassisAimCommand(
            Swerve Swerve,
            Supplier<ShootingDecider.Destination> destinationSupplier,
            DoubleSupplier driverX,
            DoubleSupplier driverY) {
        this.swerve = Swerve;
        this.driverX = driverX;
        this.driverY = driverY;
        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();
    }

    @Override
    public void initialize() {
        boolean reset = false;
    }

    @Override
    public void execute() {
        swerve.drive(
                new Translation2d(
                        -driverX.getAsDouble() * RobotConstants.SwerveConstants.maxSpeed.magnitude(),
                        -driverY.getAsDouble() * RobotConstants.SwerveConstants.maxSpeed.magnitude()),
                0,
                true,
                false);

        ShootingParameters parameter = shootingDecider.getShootingParameter(
                destinationSupplier.get(),
                swerve.getLocalizer().getCoarseFieldPose(0));
        double degrees = parameter.getFieldAimingAngle().getDegrees();
        swerve.setHeadingTarget(degrees);
        swerve.setLockHeading(true);
        Logger.recordOutput("swerve/DistanceShooting",parameter.getDistance());
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setLockHeading(false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
