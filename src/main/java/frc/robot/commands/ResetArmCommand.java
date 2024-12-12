package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Degrees;


public class ResetArmCommand extends Command {
    private final Arm arm;

    public ResetArmCommand(Arm arm) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmVoltage(-1.5);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return arm.armMotor.getSupplyCurrent().getValueAsDouble() > 1.0;
    }

    @Override
    public void end(boolean interrupted) {
        arm.setArmVoltage(0);
        arm.setArmHome(Degrees.of(0));
    }
}
