package frc.robot.commands.test;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Arm;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Degrees;

public class ArmUpCommand extends Command {
    private final Arm arm;

    public ArmUpCommand(Arm arm) {
        this.arm = arm;

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        RobotConstants.armPosition = RobotConstants.armPosition.plus(Degrees.of(0.3));
        arm.setArmPosition(RobotConstants.armPosition);
        Logger.recordOutput("Arm/Position1: ", RobotConstants.armPosition.magnitude());
    }

    @Override
    public void end(boolean interrupted) {

    }
}
