package frc.robot.commands;

import frc.robot.subsystems.shooter.*;
import frc.robot.subsystems.shooter.shooter;

import com.google.common.base.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class shootercommand extends Command {
    private shooter shooter;
    // private double Voltage;
    private double Voltage;

    public shootercommand(shooter shooter,Double Voltage){
      this.shooter = shooter;
      this.Voltage = Voltage;
      addRequirements(shooter);
    }

    @Override
    public void execute(){
          shooter.setshooter(Voltage);

    }

    
    @Override
    public void end(boolean interrupted){
      shooter.setshooter(0);
    }
}
