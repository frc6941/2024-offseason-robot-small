package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.ShootingDecider;

public class FlyWheelRampUp extends Command {
    private final Shooter shooter;
    //private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;

    public FlyWheelRampUp(
            Shooter shooter


    ) {
        this.shooter = shooter;
//        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();
        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooter.setShooterRPM(
                RobotConstants.ShooterConstants.hShooterTestRPM.get(),
                RobotConstants.ShooterConstants.lShooterTestRPM.get()
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterDirectVoltage(0);
    }
}
