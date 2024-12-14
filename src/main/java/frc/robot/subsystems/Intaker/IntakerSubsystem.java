package frc.robot.subsystems.Intaker;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.Alert;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class IntakerSubsystem extends SubsystemBase {
    private static double collectVoltage = RobotConstants.IntakerConstants.collectingVoltage.get();
    private static double outtakeVoltage = RobotConstants.IntakerConstants.outtakingVoltage.get();
    private static double feedingRPM = RobotConstants.IntakerConstants.feedingRPM.get();
    private static double triggerRPM = RobotConstants.IntakerConstants.triggerRPM.get();
    private static double idleRPM = RobotConstants.IntakerConstants.idlingRPM.get();

    public enum WantedState {
        IDLE,
        COLLECT,
        FEED,
        TRIGGER,
        OUTTAKE,
        OFF
    }

    public enum SystemState {
        IDLING,
        COLLECTING,
        FEEDING,
        TRIGGERING,
        OUTTAKING,
        OFF
    }

    private IntakerIO io;
    private IntakerIOInputsAutoLogged inputs = new IntakerIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;


    private final Alert intakerDisconnected = new Alert("Intaker motor disconnected!", Alert.AlertType.WARNING);

    public IntakerSubsystem(IntakerIO io) {
        this.io = io;
    }
    
    @Override
    public void periodic() {
        // read inputs
        io.updateInputs(inputs);
        // log inputs
        Logger.processInputs("Intaker", inputs);

        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Intaker/SystemState", newState.toString());
            systemState = newState;
        }

        // holds the values to apply
        double intakerMotorRPM;

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLING;
        }

        // refresh RPM tunable numbers
        if (RobotConstants.TUNING) {
            collectVoltage = RobotConstants.IntakerConstants.collectingVoltage.get();
            outtakeVoltage = RobotConstants.IntakerConstants.outtakingVoltage.get();
            feedingRPM = RobotConstants.IntakerConstants.feedingRPM.get();
            triggerRPM = RobotConstants.IntakerConstants.triggerRPM.get();
            idleRPM = RobotConstants.IntakerConstants.idlingRPM.get();
        }

        // set speeds based on state
        switch (systemState) {
            case TRIGGERING:
                io.setVelocity(RotationsPerSecond.of(triggerRPM/60));
                break;
            case FEEDING:
                io.setVelocity(RotationsPerSecond.of(feedingRPM/60));
                break;
            case COLLECTING:
                io.setVoltage(Volts.of(collectVoltage));
                break;
            case IDLING:
                io.setVoltage(Volts.of(idleRPM));
                break;
            case OUTTAKING:
                io.setVoltage(Volts.of(outtakeVoltage));
            case OFF:
            default:
                io.setVelocity(RotationsPerSecond.of(0));
                break;
        }


        // Alerts
        intakerDisconnected.set(!inputs.intakerConnected);
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case TRIGGER -> {
                if(!inputs.beamBreakState) {
                    yield  SystemState.OFF;
                }
                yield SystemState.TRIGGERING;
            }
            case OUTTAKE -> SystemState.OUTTAKING;
            case COLLECT -> {
                if (!inputs.beamBreakState && inputs.intakerSpeed.magnitude()> 0) {
                    //Decide if note has entered intaker
                    yield SystemState.COLLECTING;
                }
                yield SystemState.FEEDING;
            }
            case FEED -> {
                if (!inputs.beamBreakState ) {
                    yield SystemState.FEEDING;
                }
                yield SystemState.IDLING;
            }
            default -> SystemState.IDLING;
        };
    }

    public boolean isBeamBreakTripped() {
        return inputs.beamBreakState;
    }

    /**
     * Sets the target state for the intake subsystem.
     * @param wantedState the target state for the intake subsystem.
     */
    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }
}
