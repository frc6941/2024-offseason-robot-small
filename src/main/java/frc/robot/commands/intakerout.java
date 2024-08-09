package frc.robot.commands;

import frc.robot.subsystems.intaker.*;
import frc.robot.subsystems.shooter.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class intakerout extends Command {
    private final intaker intaker;
    private final shooter shooter;

    public intakerout(intaker intaker , shooter shooter) {
        this.intaker = intaker;
        this.shooter = shooter;
        addRequirements(intaker , shooter);
    }

    @Override
    public void execute() {
        intaker.setintaker(-0.7);
        shooter.setshooter(0 , 0);
    }

    @Override
    public void end(boolean interrupted) {
        intaker.setintaker(0);
    }

    

}
