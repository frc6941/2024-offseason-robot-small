package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.ShootingDecider;

public class FlyWheelRampUp extends Command {
    private final ShooterSubsystem shooterSubsystem;
    //private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;

    public FlyWheelRampUp(
            ShooterSubsystem shooterSubsystem


    ) {
        this.shooterSubsystem = shooterSubsystem;
//        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterRPM(
                Constants.ShooterConstants.hShooterTestRPM.get(),
                Constants.ShooterConstants.lShooterTestRPM.get()
        );
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterDirectVoltage(0);
    }
}
