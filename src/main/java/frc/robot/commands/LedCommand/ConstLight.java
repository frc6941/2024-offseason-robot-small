package frc.robot.commands.LedCommand;

import frc.robot.subsystems.led.*;

import edu.wpi.first.wpilibj2.command.Command;

public class ConstLight extends Command {

    private led led;
    private int R;
    private int G;
    private int B;
    private boolean flag = false;

    public ConstLight(led led, int R, int G, int B) {
        this.led = led;
        this.R = R;
        this.G = G;
        this.B = B;
        addRequirements(led);
    }

    @Override
    public void execute() {
        led.setled(R, G, B);
        flag = true;
    }

    @Override
    public void end(boolean interrupted) {
        led.setled(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return flag;
    }
}
