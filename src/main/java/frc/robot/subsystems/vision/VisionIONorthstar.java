package frc.robot.subsystems.vision;

import java.util.Optional;

public class VisionIONorthstar implements VisionIO {
	public VisionIONorthstar() {
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
