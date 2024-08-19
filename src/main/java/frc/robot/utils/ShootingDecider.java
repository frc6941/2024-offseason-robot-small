package frc.robot.utils;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
import frc.robot.display.Display;
import frc.robot.display.OperatorDashboard;
import frc.robot.utils.AllianceFlipUtil;
import frc.robot.utils.FieldLayout;
import frc.robot.utils.TunableNumber;
import org.frcteam6941.looper.Updatable;

public class ShootingDecider implements Updatable {
    public static final Translation2d kCornerTarget = new Translation2d(1.0, FieldLayout.kFieldWidth - 1.5);
    public static final Translation2d kMidUpperTarget = new Translation2d((FieldLayout.kFieldLength / 2.0) - 1.0,
            FieldLayout.kFieldWidth - 0.2);

    private static ShootingDecider instance;
    // TunableNumber ampAngle;
    // TunableNumber ampVelocity;
    LaunchParameterTable speakerParams;
    LaunchParameterTable highFerryParams;
    LaunchParameterTable lowFerryParams;

    private ShootingDecider() {
        speakerParams = new LaunchParameterTable("Speaker");
        highFerryParams = new LaunchParameterTable("HighFerry");
        lowFerryParams = new LaunchParameterTable("Low Ferry");

        // ampAngle = new TunableNumber("Amp Angle Degs", 165.0);
        // ampVelocity = new TunableNumber("Amp Velocity Rpm", -3000.0);

        speakerParams.loadParameter(1.064, 0, 0);// 20240808
        speakerParams.loadParameter(1.245, 0, 0);// 20240808
        speakerParams.loadParameter(1.600, 0, 0);// 20240808
        speakerParams.loadParameter(1.910, 0, 0);// 20240808
        speakerParams.loadParameter(2.20, 0, 0);// 20240808
        speakerParams.loadParameter(2.51, 0, 0);// 20240808
        speakerParams.loadParameter(2.8, 0, 0);
        speakerParams.loadParameter(3.06, 0, 0);// 20240808
        speakerParams.loadParameter(3.47, 0, 0);// 20240808
        speakerParams.loadParameter(3.79, 0, 0);//
        speakerParams.loadParameter(3.9, 0, 0);// 20240808
        speakerParams.ready();

        highFerryParams.loadParameter(3.0, 1000, 1000);
        highFerryParams.loadParameter(5.0, 2800, 2800);
        highFerryParams.loadParameter(6.5, 4100, 4100);
        highFerryParams.loadParameter(8.0, 4400, 4400);
        highFerryParams.loadParameter(9.5, 4800, 4800);
        highFerryParams.loadParameter(11.0, 4800, 4800);

        highFerryParams.ready();
        lowFerryParams.loadParameter(1.5, 1000, 1000);
        lowFerryParams.loadParameter(3.4, 1000, 1000);
        lowFerryParams.ready();
    }

    public static ShootingDecider getInstance() {
        if (instance == null) {
            instance = new ShootingDecider();
        }
        return instance;
    }

    private static boolean inHighFerryZone(Pose2d robotPose) {
        Pose2d pose = AllianceFlipUtil.apply(robotPose);
        return pose.getX() > FieldConstants.wingOpponentX;
    }

    @Override
    public void update(double time, double dt) {
        speakerParams.update();
        highFerryParams.update();
        lowFerryParams.update();
    }

    public ShootingParameters getShootingParameter(Destination destination, Pose2d robotPose) {
        Translation2d target, delta;
        Pair<Double, Double> launchParam;

        switch (destination) {
            // case AMP:
            // return new ShootingParameters(Double.NaN, ampVelocity.get(), ampAngle.get(),
            // new Rotation2d());
            case FERRY:
                boolean useMid = inHighFerryZone(robotPose);
                target = useMid ? AllianceFlipUtil.apply(kMidUpperTarget) : AllianceFlipUtil.apply(kCornerTarget);
                Display.getInstance().setFerryLocation(target);
                delta = target.minus(robotPose.getTranslation());
                launchParam = useMid ? lowFerryParams.getParameters(delta.getNorm())
                        : highFerryParams.getParameters(delta.getNorm());
                return new ShootingParameters(delta.getNorm(), launchParam.getFirst(), launchParam.getSecond(),
                        new Rotation2d(delta.getX(), delta.getY()).rotateBy(Rotation2d.fromDegrees(180)));
            case SPEAKER:
                target = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening).toTranslation2d();
                delta = target.minus(robotPose.getTranslation());
                launchParam = speakerParams.getParameters(delta.getNorm());
                return new ShootingParameters(delta.getNorm(), launchParam.getFirst(), launchParam.getSecond(),
                        new Rotation2d(delta.getX(), delta.getY()).rotateBy(Rotation2d.fromDegrees(180)));
            default:
                throw new IllegalArgumentException("Illegal destination: undefined.");
        }
    }

    public enum Destination {
        FERRY, SPEAKER
    }
}
