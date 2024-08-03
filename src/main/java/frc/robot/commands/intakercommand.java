package frc.robot.commands;

import frc.robot.subsystems.intaker.*;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class intakercommand extends Command {
    private final intaker intaker;
    private final double Voltage;

    public intakercommand(intaker intaker,double Voltage){
      this.intaker = intaker;
      this.Voltage = Voltage;
      addRequirements(intaker);
    }

    @Override
    public void execute(){
      intaker.setintaker(Voltage);
    }

    @Override
    public void end(boolean interrupted){
      intaker.setintaker(0);
    }
}
