package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drivers.BeamBreak;
import frc.robot.Constants;

public class BeamBreakSubsystem extends SubsystemBase{
    private final BeamBreak intakerBeamBreak =
            new BeamBreak(Constants.BeamBreakConstants.INTAKER_BEAMBREAK_ID);
    private final BeamBreak shooterBeamBreak =
            new BeamBreak(Constants.BeamBreakConstants.SHOOTER_BEAMBREAK_ID);
    private boolean lastRecordedState;
    private boolean noteState = false;

    public BeamBreakSubsystem() {
        boolean isIntakerBeamBreakOn = intakerBeamBreak.get();
        boolean isShooterBeamBreakOn = shooterBeamBreak.get();

        if (!lastRecordedState && isIntakerBeamBreakOn) {
            noteState = true;
        }
        lastRecordedState = isIntakerBeamBreakOn;

    public boolean hasNote() {
        return noteState;
    }

        public boolean isIntakeReady() {
            return isIndexerBeamBreakOn &&
                    !isIntakerBeamBreakOn;
        }

    }
}