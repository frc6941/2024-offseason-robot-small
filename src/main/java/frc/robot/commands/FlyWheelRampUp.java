package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShootingDecider;

import java.util.function.Supplier;

public class FlyWheelRampUp extends Command {
    private final shooter shooter;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;

    public FlyWheelRampUp(
            shooter shooter,
            Supplier<ShootingDecider.Destination> destinationSupplier

    ) {
        this.shooter = shooter;
        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        shooter.setShooterVelocity(
                shootingDecider.getShootingParameter(
                        destinationSupplier.get(),
                        Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)
        ).getShootingVelocityL(),
        shootingDecider.getShootingParameter(
                        destinationSupplier.get(),
                        Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)
        ).getShootingVelocityH());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterVelocity(Constants.ShooterConstants.SHOOTER_IDLE_VELOCITY,Constants.ShooterConstants.SHOOTER_IDLE_VELOCITY);
    }
}
