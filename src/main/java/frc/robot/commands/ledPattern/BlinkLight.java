package frc.robot.commands.ledPattern;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.Led;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.Timer;

public class BlinkLight extends Command {
    private Led led;
    private int R;
    private int G;
    private int B;
    private final Timer timer = new Timer();
    private final Measure<Time> ledTime;
    private double lastChange;
    private final double interval = 0.05;
    private boolean on = true;

    public BlinkLight(Led led, Measure<Time> seconds, int R, int G, int B) {
        this.R = R;
        this.G = G;
        this.B = B;
        this.led = led;
        this.ledTime = seconds;
        addRequirements(led);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        var timestamp = Timer.getFPGATimestamp();
        if (timestamp - lastChange > interval) {
            on = !on;
            lastChange = timestamp;
        }
        if (on) {
            led.setled(R, G, B);
            return;
        }
        led.setled(0, 0, 0);
    }

    @Override
    public void end(boolean interrupted) {
        led.setled(0, 0, 0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(ledTime.magnitude());
    }
}