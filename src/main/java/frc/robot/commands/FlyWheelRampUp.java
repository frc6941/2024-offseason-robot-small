package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.display.OperatorDashboard;
import frc.robot.drivers.BeamBreak;
import frc.robot.subsystems.intaker.intaker;
import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShootingDecider;

import java.util.function.Supplier;

public class FlyWheelRampUp extends Command {
    private final shooter shooter;
    private final Supplier<ShootingDecider.Destination> destinationSupplier;
    private final ShootingDecider shootingDecider;
    private final intaker intaker;
    private final BeamBreak intakerBeamBreakH;
    private final BeamBreak intakerBeamBreakL;

    public FlyWheelRampUp(
            intaker intaker,
            shooter shooter,
            BeamBreak intakerBeamBreakL,
            BeamBreak intakerBeamBreakH,
            Supplier<ShootingDecider.Destination> destinationSupplier

    ) {
        this.shooter = shooter;
        this.intakerBeamBreakH = intakerBeamBreakH;
        this.intakerBeamBreakL = intakerBeamBreakL;
        this.intaker = intaker;
        this.destinationSupplier = destinationSupplier;
        this.shootingDecider = ShootingDecider.getInstance();

    }

    @Override
    public void initialize() {
        shooter.setShooterVelocity(3000);
    }

    @Override
    public void execute() {

        shooter.setShooterVelocity(
                shootingDecider.getShootingParameter(
                        destinationSupplier.get(),
                        Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)).getShootingVelocityL(),
                shootingDecider.getShootingParameter(
                        destinationSupplier.get(),
                        Swerve.getInstance().getLocalizer().getCoarseFieldPose(0)).getShootingVelocityH());
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intaker.setIntakerDutyCycle(1);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShooterDutyCycle(0, 0);
        intaker.setIntakerDutyCycle(0);
    }

    @Override
    public boolean isFinished() {
        return (!intakerBeamBreakH.get() && !intakerBeamBreakL.get());
    }
}
