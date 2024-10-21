package frc.robot.display;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

public class FieldView {
    private final Field2d mField2d = new Field2d();
//    NetworkTableInstance.
    public FieldView() {
        SmartDashboard.putData(mField2d);
    }

    private void drawField() {
        Translation2d blueSpeakerTranslation = FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d();
        Pose2d blueSpeaker = new Pose2d(blueSpeakerTranslation, Rotation2d.fromDegrees(180.0));
        Translation2d redSpeakerTranslation = new Translation2d(
                FieldConstants.fieldLength - blueSpeakerTranslation.getX(), blueSpeakerTranslation.getY());
        Pose2d redSpeaker = new Pose2d(redSpeakerTranslation, Rotation2d.fromDegrees(0.0));

        mField2d.getObject("Blue Speaker").setPose(blueSpeaker);
        mField2d.getObject("Red Speaker").setPose(redSpeaker);
    }

    public void setFerryLocation(Translation2d pos) {
        mField2d.getObject("FerryLoc").setPose(new Pose2d(pos, new Rotation2d()));
    }

    public void update(Pose2d pose, Pose2d ghost) {
        drawField();

        mField2d.setRobotPose(pose);
        mField2d.getObject("Ghost").setPose(ghost);
        Logger.recordOutput("SouthStar",
                new Pose2d(
                        NetworkTableInstance.getDefault().getDoubleTopic("/orangePi5Plus/output/demo_observations/1").subscribe(0.0).get(),
                        NetworkTableInstance.getDefault().getDoubleTopic("/orangePi5Plus/output/demo_observations/2").subscribe(0.0).get(),
                Rotation2d.fromDegrees(NetworkTableInstance.getDefault().getDoubleTopic("/orangePi5Plus/output/demo_observations/3").subscribe(0.0).get()))
        );
    }

}
