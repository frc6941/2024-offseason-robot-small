package frc.robot.commands;

import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.intaker.intaker;

import edu.wpi.first.wpilibj2.command.Command;

public class shootercommand extends Command {
    private shooter shooter;
    private double Voltage;
    private intaker intaker;

    public shootercommand(shooter shooter, intaker intaker, double Voltage) {
        this.shooter = shooter;
        this.Voltage = Voltage;
        this.intaker = intaker;
        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        shooter.setshooter(Voltage);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intaker.setintaker(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setshooter(0);
        intaker.setintaker(0);
    }
}
