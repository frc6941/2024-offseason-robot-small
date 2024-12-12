package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.shooting.ShootingDecider;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Degrees;

public class ArmAimCommand extends Command {
    private final Arm armSubsystem;
    private final ShootingDecider shootingDecider;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;

    public ArmAimCommand(
            Arm armSubsystem,
            Supplier<ShootingDecider.Destination> destinationSupplier
    ) {
        this.shootingDecider = ShootingDecider.getInstance();
        this.armSubsystem = armSubsystem;
        this.destinationSupplier = destinationSupplier;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        armSubsystem.setArmPosition(Degrees.of(
                shootingDecider.getShootingParameter(
                        destinationSupplier.get(),
                        Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)
                ).getShootingAngle()));
    }


    @Override
    public void end(boolean interrupted) {
        //armSubsystem.getIo().setArmPosition(Radians.zero());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

