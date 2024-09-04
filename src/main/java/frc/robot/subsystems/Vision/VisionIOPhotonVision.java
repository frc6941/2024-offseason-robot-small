package frc.robot.subsystems.Vision;

import java.util.Optional;

public class VisionIOPhotonVision implements VisionIO {
	public VisionIOPhotonVision() {
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
//		inputs.botEstimate=new Pose2d();
	}

	@Override
	public Optional<LimelightHelpers.PoseEstimate> getPoseEstimate() {
		return Optional.empty();
	}

	@Override
	public void updateOdometry(){

	}
}
