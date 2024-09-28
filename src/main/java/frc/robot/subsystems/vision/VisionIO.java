package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

import java.util.Optional;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public Pose2d botEstimate;
        public boolean botEstimatePresent;
        public double deviationX, deviationY, deviationOmega;
        public boolean targetUpdated;
    }

    void updateInputs(VisionIOInputs inputs);

    Optional<LimelightHelpers.PoseEstimate> getPoseEstimate();
    void updateOdometry();
}
