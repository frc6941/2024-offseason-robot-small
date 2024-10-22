// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.junction.AutoLogOutput;

import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.BooleanSupplier;

@ExtensionMethod({GeomUtil.class})
public class RobotState {
    private static final LoggedTunableNumber autoLookahead =
            new LoggedTunableNumber("RobotState/AutoLookahead", 0.5);
    private static final LoggedTunableNumber lookahead =
            new LoggedTunableNumber("RobotState/lookaheadS", 0.35);
    private static final LoggedTunableNumber superPoopLookahead =
            new LoggedTunableNumber("RobotState/SuperPoopLookahead", 0.1);
    private static final LoggedTunableNumber closeShootingZoneFeet =
            new LoggedTunableNumber("RobotState/CloseShootingZoneFeet", 10.0);
    private static final double poseBufferSizeSeconds = 2.0;
    private static final double armAngleCoefficient = 57.254371165197;
    private static final double armAngleExponent = -0.593140189605718;
    // Super poop
    private static final InterpolatingDoubleTreeMap superPoopArmAngleMap =
            new InterpolatingDoubleTreeMap();
    private static final InterpolatingTreeMap<Double, FlywheelSpeeds> superPoopFlywheelSpeedsMap =
            new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), FlywheelSpeeds::interpolate);
    private static final double autoFarShotCompensationDegrees = 0.0; // 0.6 at NECMP
    private static final Translation2d superPoopTarget =
            Constants.FieldConstants.Subwoofer.centerFace
                    .getTranslation()
                    .interpolate(Constants.FieldConstants.ampCenter, 0.5);
    private static final LoggedTunableNumber demoTargetDistance =
            new LoggedTunableNumber("RobotState/DemoTargetDistance", 2.0);
    private static RobotState instance;

    static {
        superPoopArmAngleMap.put(Units.feetToMeters(33.52713263758169), 35.0);
        superPoopArmAngleMap.put(Units.feetToMeters(28.31299227120627), 39.0);
        superPoopArmAngleMap.put(Units.feetToMeters(25.587026383435525), 48.0);
    }

    static {
        superPoopFlywheelSpeedsMap.put(
                Units.feetToMeters(33.52713263758169), new FlywheelSpeeds(3500, 4500));
        superPoopFlywheelSpeedsMap.put(
                Units.feetToMeters(28.31299227120627), new FlywheelSpeeds(2800, 3500));
        superPoopFlywheelSpeedsMap.put(
                Units.feetToMeters(25.587026383435525), new FlywheelSpeeds(2500, 3200));
    }

    private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
            TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
    private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
    // Odometry
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics();
    @AutoLogOutput
    @Getter
    @Setter
    private boolean flywheelAccelerating = false;
    @AutoLogOutput
    @Getter
    @Setter
    private double shotCompensationDegrees = 0.0;
    // Pose Estimation Members
    private Pose2d odometryPose = new Pose2d();
    private Pose2d estimatedPose = new Pose2d();
    @Getter
    @Setter
    private Pose2d trajectorySetpoint = new Pose2d();
    private SwerveDriveWheelPositions lastWheelPositions =
            new SwerveDriveWheelPositions(
                    new SwerveModulePosition[]{
                            new SwerveModulePosition(),
                            new SwerveModulePosition(),
                            new SwerveModulePosition(),
                            new SwerveModulePosition()
                    });
    private Rotation2d lastGyroAngle = new Rotation2d();
    private Twist2d robotVelocity = new Twist2d();
    private Twist2d trajectoryVelocity = new Twist2d();
    /**
     * Cached latest aiming parameters. Calculated in {@code getAimingParameters()}
     */
    private AimingParameters latestParameters = null;
    private AimingParameters latestSuperPoopParameters = null;
    // Demo parameters
    private Pose3d demoTagPose = null;
    private DemoFollowParameters latestDemoParamters = null;
    @Setter
    @Getter
    private DemoShotParameters demoShotParameters =
            new DemoShotParameters(Rotation2d.fromDegrees(0.0), new FlywheelSpeeds(0.0, 0.0));
    @Setter
    private BooleanSupplier lookaheadDisable = () -> false;

    private RobotState() {
//        for (int i = 0; i < 3; ++i) {
//            qStdDevs.set(i, 0, Math.pow(DriveConstants.odometryStateStdDevs.get(i, 0), 2));
//        }
//        kinematics = DriveConstants.kinematics;
//
//        // Setup NoteVisualizer
//        NoteVisualizer.setRobotPoseSupplier(this::getEstimatedPose);
    }

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    /**
     * Add odometry observation
     */
    public void addOdometryObservation(OdometryObservation observation) {
        latestParameters = null;
        latestSuperPoopParameters = null;
        Twist2d twist = kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
        lastWheelPositions = observation.wheelPositions();
        // Check gyro connected
        if (observation.gyroAngle != null) {
            // Update dtheta for twist if gyro connected
            twist =
                    new Twist2d(
                            twist.dx, twist.dy, observation.gyroAngle().minus(lastGyroAngle).getRadians());
            lastGyroAngle = observation.gyroAngle();
        }
        // Add twist to odometry pose
        odometryPose = odometryPose.exp(twist);
        // Add pose to buffer at timestamp
        poseBuffer.addSample(observation.timestamp(), odometryPose);
        // Calculate diff from last odometry pose and add onto pose estimate
        estimatedPose = estimatedPose.exp(twist);
    }

    public void addVisionObservation(VisionObservation observation) {
        latestParameters = null;
        latestSuperPoopParameters = null;
        // If measurement is old enough to be outside the pose buffer's timespan, skip.
        try {
            if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
                    > observation.timestamp()) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }
        // Get odometry based pose at timestamp
        var sample = poseBuffer.getSample(observation.timestamp());
        if (sample.isEmpty()) {
            // exit if not there
            return;
        }

        // sample --> odometryPose transform and backwards of that
        var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
        var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        // get old estimate by applying odometryToSample Transform
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

        // Calculate 3 x 3 vision matrix
        var r = new double[3];
        for (int i = 0; i < 3; ++i) {
            r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
        }
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md.
        Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; ++row) {
            double stdDev = qStdDevs.get(row, 0);
            if (stdDev == 0.0) {
                visionK.set(row, row, 0.0);
            } else {
                visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
            }
        }
        // difference between estimate and vision pose
        Transform2d transform = new Transform2d(estimateAtTime, observation.visionPose());
        // scale transform by visionK
        var kTimesTransform =
                visionK.times(
                        VecBuilder.fill(
                                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        Transform2d scaledTransform =
                new Transform2d(
                        kTimesTransform.get(0, 0),
                        kTimesTransform.get(1, 0),
                        Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

        // Recalculate current estimate by applying scaled transform to old estimate
        // then replaying odometry data
        estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
    }

    public void setDemoTagPose(Pose3d demoTagPose) {
        this.demoTagPose = demoTagPose;
        latestDemoParamters = null;
    }

    public Optional<DemoFollowParameters> getDemoTagParameters() {
//        if (latestDemoParamters != null) {
//            // Use cached demo parameters.
//            return Optional.of(latestDemoParamters);
//        }
//        // Return empty optional if no demo tag pose.
//        if (demoTagPose == null) return Optional.empty();
//
//        // Calculate target pose.
//        Pose2d targetPose =
//                demoTagPose
//                        .toPose2d()
//                        .transformBy(
//                                new Transform2d(
//                                        new Translation2d(demoTargetDistance.get(), 0.0), new Rotation2d(Math.PI)));
//
//        // Calculate heading without movement.
//        Translation2d demoTagFixed = demoTagPose.getTranslation().toTranslation2d();
//        Translation2d robotToDemoTagFixed = demoTagFixed.minus(getEstimatedPose().getTranslation());
//        Rotation2d targetHeading = robotToDemoTagFixed.getAngle();
//
//        // Calculate arm angle.
//        double z = demoTagPose.getZ();
//        Rotation2d armAngle =
//                new Rotation2d(
//                        robotToDemoTagFixed.getNorm() - ArmConstants.armOrigin.getX(),
//                        z - ArmConstants.armOrigin.getY());
//
//        latestDemoParamters = new DemoFollowParameters(targetPose, targetHeading, armAngle);
        return Optional.empty();
    }

    @AutoLogOutput(key = "RobotState/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public record OdometryObservation(
            SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {
    }

    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    }

    public record FlywheelSpeeds(double leftSpeed, double rightSpeed) {
        public static FlywheelSpeeds interpolate(FlywheelSpeeds t1, FlywheelSpeeds t2, double v) {
            double leftSpeed = MathUtil.interpolate(t1.leftSpeed(), t2.leftSpeed(), v);
            double rightSpeed = MathUtil.interpolate(t1.rightSpeed(), t2.rightSpeed(), v);
            return new FlywheelSpeeds(leftSpeed, rightSpeed);
        }
    }

    public record AimingParameters(
            Rotation2d driveHeading,
            Rotation2d armAngle,
            double effectiveDistance,
            FlywheelSpeeds flywheelSpeeds) {
    }

    public record DemoFollowParameters(
            Pose2d targetPose, Rotation2d targetHeading, Rotation2d armAngle) {
    }

    public record DemoShotParameters(Rotation2d armAngle, FlywheelSpeeds flywheelSpeeds) {
    }
}
