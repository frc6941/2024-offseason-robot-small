package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Arm;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;

public class ArmDownCommand extends Command {
    private final Arm arm;

    public ArmDownCommand(Arm arm) {
        this.arm = arm;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotConstants.armPosition = RobotConstants.armPosition.minus(Degrees.of(0.3));
        Logger.recordOutput("Arm/Position1: ", RobotConstants.armPosition.magnitude());
        arm.setArmPosition(RobotConstants.armPosition);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
