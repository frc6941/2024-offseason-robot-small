package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldLayout;

import java.util.Optional;

public class VisionIOLimelight implements VisionIO {
	private static int measuerCnt = 0;
	private static double deviationX, deviationY, deviationOmega;
	private Translation2d kdeltaToTag;
	private Pose3d kTagPose;

	private int loopCnt = 0;
	private boolean[][] tagFlagCnt = new boolean[30][60];
	private double[] tagCnt = new double[30];//17

	private boolean targetUpdated;
	
	public VisionIOLimelight() {
	}

	private static final NetworkTable limelightTable = NetworkTableInstance
			.getDefault()
			.getTable(Constants.VisionConstants.AIM_LIMELIGHT_NAME);

	private static final NetworkTableEntry tv = limelightTable.getEntry("tv");

	private Optional<LimelightHelpers.PoseEstimate> botEstimate;

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		inputs.botEstimatePresent=botEstimate.isPresent();
		if(inputs.botEstimatePresent) inputs.botEstimate=botEstimate.get().pose;

		inputs.deviationX=deviationX;
		inputs.deviationY=deviationY;
		inputs.deviationOmega=deviationOmega;

		inputs.targetUpdated=targetUpdated;
	}

	@Override
	public Optional<LimelightHelpers.PoseEstimate> getPoseEstimate() {
		if (hasTarget()) {
			botEstimate = Optional.of(
					LimelightHelpers
							.getBotPoseEstimate_wpiBlue(Constants.VisionConstants.AIM_LIMELIGHT_NAME));
		} else {
			botEstimate = Optional.empty();
		}
		return botEstimate;
	}

	/**
	 * @return whether there is a target on the camera.
	 */
	public static boolean hasTarget() {
		return tv.getDouble(0) == 1;
	}

	@Override
	public void updateOdometry(){
		int speakerTag1 = AllianceFlipUtil.shouldFlip() ? 4 : 7;
		int speakerTag2 = AllianceFlipUtil.shouldFlip() ? 3 : 8;
		loopCnt++;
		int ktagID = (int) LimelightHelpers.getFiducialID(Constants.VisionConstants.AIM_LIMELIGHT_NAME);
		for (int i = 1; i <= 16; i++) {
			if (tagFlagCnt[i][loopCnt]) tagCnt[i]--;
			tagFlagCnt[i][loopCnt] = false;
		}
		if (ktagID == speakerTag1 || ktagID == speakerTag2) {
			tagFlagCnt[ktagID][loopCnt] = true;
			tagCnt[ktagID]++;
			if (getPoseEstimate().isPresent()) {
				if (getPoseEstimate().get().rawFiducials.length >= 2) {
					ktagID = ktagID == speakerTag1 ? speakerTag2 : speakerTag1;
					tagFlagCnt[ktagID][loopCnt] = true;
					tagCnt[ktagID]++;
					ktagID = ktagID == speakerTag1 ? speakerTag2 : speakerTag1;
				}
			}
		}
		loopCnt %= 20;
		boolean isAutoDrive = Swerve.getInstance().getState() == Swerve.State.PATH_FOLLOWING;

		LimelightHelpers.SetRobotOrientation(Constants.VisionConstants.AIM_LIMELIGHT_NAME,
				Swerve.getInstance().getLocalizer().getLatestPose().getRotation().getDegrees(),
				Swerve.getInstance().getLocalizer().getSmoothedVelocity().getRotation().getDegrees(),
				0, 0, 0, 0);
		//rejection
		if (Swerve.getInstance().getLocalizer().getSmoothedVelocity().getTranslation().getNorm() > Constants.SwerveConstants.maxSpeed.magnitude())
			return;
		if (Math.abs(Swerve.getInstance().getLocalizer().getSmoothedVelocity().getRotation()
				.getDegrees()) > Math.toDegrees(Constants.SwerveConstants.maxAngularRate.magnitude()))
			return;

		if (FieldLayout.kTagMap.getTagPose(ktagID).isPresent() && getPoseEstimate().isPresent()) {
			kTagPose = FieldLayout.kTagMap.getTagPose(ktagID).get();
			kdeltaToTag = new Translation2d(kTagPose.getX(), kTagPose.getY()).minus(getPoseEstimate().get().pose.getTranslation());
			if (
					(((tagCnt[speakerTag1] < 15 || tagCnt[speakerTag2] < 15)
							&& !(tagCnt[speakerTag1] == 0 && tagCnt[speakerTag2] == 20 && kdeltaToTag.getNorm() < 2.5)
							&& !(tagCnt[speakerTag2] == 0 && tagCnt[speakerTag1] == 20 && kdeltaToTag.getNorm() < 2.5)
							&& (ktagID == speakerTag1 || ktagID == speakerTag2)))) {
				targetUpdated=false;
				return;
			}
		}
		if (isAutoDrive) {//TODO: fix it
			targetUpdated=false;
			return;
		}

		// if (Swerve.getInstance().getLocalizer().getLatestPose().getX() < 0
		// 		|| Swerve.getInstance().getLocalizer().getLatestPose().getX() > Constants.FieldConstants.fieldLength
		// 		|| Swerve.getInstance().getLocalizer().getLatestPose().getY() < 0
		// 		|| Swerve.getInstance().getLocalizer().getLatestPose().getY() > Constants.FieldConstants.fieldWidth)
		// 	return;
		if (getPoseEstimate().isPresent()) {
			if (measuerCnt <= 3) {
				measuerCnt++;
				deviationX = 0.01;
				deviationY = 0.01;
				deviationOmega = 0.001;
			} else {
				deviationX = (0.0062 * getPoseEstimate().get().pose.getX() + 0.0087) * 40;//80
				deviationY = (0.0062 * getPoseEstimate().get().pose.getY() + 0.0087) * 40;
				deviationOmega = 15;//(0.0062 * getPoseEstimate().get().pose.getRotation().getDegrees() + 0.0087) * 0.2;
			}
			getPoseEstimate().ifPresent((poseEstimate) -> {
				Swerve.getInstance().getLocalizer().addMeasurement(
						getPoseEstimate().get().timestampSeconds,
						getPoseEstimate().get().pose,
						new Pose2d(new Translation2d(deviationX, deviationY),
								Rotation2d.fromDegrees(deviationOmega)));
				targetUpdated=true;
			});
		}
	}
}
