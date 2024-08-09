package frc.robot.commands;

import frc.robot.subsystems.shooter.shooter;
import frc.robot.subsystems.intaker.intaker;

import edu.wpi.first.wpilibj2.command.Command;

public class shootercommand extends Command {
    private shooter shooter;
    private double VoltageL;
    private intaker intaker;
    private double VoltageH;
    private boolean flag;

    public shootercommand(shooter shooter, intaker intaker, double VoltageL , double VoltageH) {
        this.shooter = shooter;
        this.VoltageL = VoltageL;
        this.intaker = intaker;
        this.VoltageH = VoltageH;
        addRequirements(shooter, intaker);
    }

    @Override
    public void execute() {
        shooter.setshooter(VoltageL,VoltageH);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        intaker.setintaker(1.0);
        flag = false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setshooter(0,0);
        intaker.setintaker(0);
    }
}
