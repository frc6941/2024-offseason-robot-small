package frc.robot.commands;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShootingDecider;

import java.util.function.Supplier;



public class ChassisAimSpeakerYAutoCommand extends Command {
    private final Swerve swerve;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;

    LinearFilter filter = LinearFilter.singlePoleIIR(0.2, 0.02);

    public ChassisAimSpeakerYAutoCommand(
            Swerve Swerve,
            Supplier<ShootingDecider.Destination> destinationSupplier) {
        this.swerve = Swerve;
        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();
    }

    @Override
    public void initialize() {
        filter.reset();
        swerve.setLockHeading(true);
    }

    @Override
    public void execute() {

        swerve.setDefaultCommand(Commands
                .runOnce(() -> swerve.drive(
                    new Translation2d(
                        0,3.4),
                        0,
                        true,
                        false),
                        swerve).withTimeout(shootingDecider.getShootingParameter(destinationSupplier.get(),
                        swerve.getLocalizer().getCoarseFieldPose(0))
                        .getDelta().getY()/3.4));


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
