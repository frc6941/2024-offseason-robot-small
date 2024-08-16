package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.frcteam6941.swerve.SwerveSetpointGenerator.KinematicLimits;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.TunableNumber;

public class Constants {
    public static final boolean TUNING = true;
    public static final double LOOPER_DT = 1 / 100.0;

    public class RobotConstants {

        public static String CAN_BUS_NAME = "rio";

        public static final CommandXboxController driverController = new CommandXboxController(0);

        public static final CommandXboxController operatorController = new CommandXboxController(1);

    }

    public static class FieldConstants {
        public static final double fieldLength = edu.wpi.first.math.util.Units.inchesToMeters(651.223);
        public static final double fieldWidth = edu.wpi.first.math.util.Units.inchesToMeters(323.277);
        public static final double wingX = edu.wpi.first.math.util.Units.inchesToMeters(229.201);
        public static final double wingOpponentX = fieldLength - wingX;
        public static final double podiumX = edu.wpi.first.math.util.Units.inchesToMeters(126.75);
        public static final double startingLineX = edu.wpi.first.math.util.Units.inchesToMeters(74.111);

        public static final Translation2d ampCenter = new Translation2d(
                edu.wpi.first.math.util.Units.inchesToMeters(72.455), fieldWidth);
        public static final double aprilTagWidth = edu.wpi.first.math.util.Units.inchesToMeters(6.50);

        /**
         * Staging locations for each note
         */
        public static final class StagingLocations {
            public static final double centerlineX = fieldLength / 2.0;

            // need to update
            public static final double centerlineFirstY = edu.wpi.first.math.util.Units.inchesToMeters(29.638);
            public static final double centerlineSeparationY = edu.wpi.first.math.util.Units.inchesToMeters(66);
            public static final double spikeX = edu.wpi.first.math.util.Units.inchesToMeters(114);
            // need
            public static final double spikeFirstY = edu.wpi.first.math.util.Units.inchesToMeters(161.638);
            public static final double spikeSeparationY = edu.wpi.first.math.util.Units.inchesToMeters(57);

            public static final Translation2d[] centerlineTranslations = new Translation2d[5];
            public static final Translation2d[] spikeTranslations = new Translation2d[3];

            static {
                for (int i = 0; i < centerlineTranslations.length; i++) {
                    centerlineTranslations[i] = new Translation2d(centerlineX,
                            centerlineFirstY + (i * centerlineSeparationY));
                }
            }

            static {
                for (int i = 0; i < spikeTranslations.length; i++) {
                    spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
                }
            }
        }

        /**
         * Each corner of the speaker *
         */
        public static final class Speaker {

            // corners (blue alliance origin)
            public static final Translation3d topRightSpeaker = new Translation3d(
                    edu.wpi.first.math.util.Units.inchesToMeters(18.055),
                    edu.wpi.first.math.util.Units.inchesToMeters(238.815),
                    edu.wpi.first.math.util.Units.inchesToMeters(83.091));

            public static final Translation3d topLeftSpeaker = new Translation3d(
                    edu.wpi.first.math.util.Units.inchesToMeters(18.055),
                    edu.wpi.first.math.util.Units.inchesToMeters(197.765),
                    edu.wpi.first.math.util.Units.inchesToMeters(83.091));

            public static final Translation3d bottomRightSpeaker = new Translation3d(0.0,
                    edu.wpi.first.math.util.Units.inchesToMeters(238.815),
                    edu.wpi.first.math.util.Units.inchesToMeters(78.324));
            public static final Translation3d bottomLeftSpeaker = new Translation3d(0.0,
                    edu.wpi.first.math.util.Units.inchesToMeters(197.765),
                    edu.wpi.first.math.util.Units.inchesToMeters(78.324));

            /**
             * Center of the speaker opening (blue alliance)
             */
            public static final Translation3d centerSpeakerOpening = bottomLeftSpeaker.interpolate(topRightSpeaker,
                    0.5);
        }

        public static final class Subwoofer {
            public static final Pose2d ampFaceCorner = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(35.775),
                    edu.wpi.first.math.util.Units.inchesToMeters(239.366),
                    Rotation2d.fromDegrees(-120));

            public static final Pose2d sourceFaceCorner = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(35.775),
                    edu.wpi.first.math.util.Units.inchesToMeters(197.466),
                    Rotation2d.fromDegrees(120));

            public static final Pose2d centerFace = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(35.775),
                    edu.wpi.first.math.util.Units.inchesToMeters(218.416),
                    Rotation2d.fromDegrees(180));
        }

        public static final class Stage {
            public static final Pose2d center = new Pose2d(edu.wpi.first.math.util.Units.inchesToMeters(192.55),
                    edu.wpi.first.math.util.Units.inchesToMeters(161.638), new Rotation2d());
            public static final Pose2d podiumLeg = new Pose2d(edu.wpi.first.math.util.Units.inchesToMeters(126.75),
                    edu.wpi.first.math.util.Units.inchesToMeters(161.638), new Rotation2d());
            public static final Pose2d ampLeg = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(220.873),
                    edu.wpi.first.math.util.Units.inchesToMeters(212.425),
                    Rotation2d.fromDegrees(-30));
            public static final Pose2d sourceLeg = new Pose2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(220.873),
                    edu.wpi.first.math.util.Units.inchesToMeters(110.837),
                    Rotation2d.fromDegrees(30));

            public static final Pose2d centerPodiumAmpChain = new Pose2d(
                    podiumLeg.getTranslation().interpolate(ampLeg.getTranslation(), 0.5),
                    Rotation2d.fromDegrees(120.0));
            public static final double centerToChainDistance = center.getTranslation()
                    .getDistance(centerPodiumAmpChain.getTranslation());
            public static final Pose2d centerAmpSourceChain = new Pose2d(
                    ampLeg.getTranslation().interpolate(sourceLeg.getTranslation(), 0.5), new Rotation2d());
            public static final Pose2d centerSourcePodiumChain = new Pose2d(
                    sourceLeg.getTranslation().interpolate(podiumLeg.getTranslation(), 0.5),
                    Rotation2d.fromDegrees(240.0));
        }

        public static final class Amp {
            public static final Translation2d ampTapeTopCorner = new Translation2d(
                    edu.wpi.first.math.util.Units.inchesToMeters(130.0),
                    edu.wpi.first.math.util.Units.inchesToMeters(305.256));
            public static final double ampBottomY = fieldWidth - edu.wpi.first.math.util.Units.inchesToMeters(17.75);
        }
    }

    public static class IntakerConstants {
        public static final int INTAKE_MOTOR_ID = 15;

        public static final int INTAKER_BEAMBREAKH_ID = 3;
        public static final int INTAKER_BEAMBREAKL_ID = 2;

        public static final double INTAKER_IN_SPEED = 0.5;
        public static final double INTAKER_OUT_SPEED = -0.8;
        public static final double INTAKER_SHOOT_SPEED = 1.0;
    }

    public static class ShooterConstants {
        public static final int SHOOTER_MOTORH_ID = 17;
        public static final int SHOOTER_MOTORL_ID = 16;

        public static final double SHOOTERH_SPEAKER_SPEED = 0.75;
        public static final double SHOOTERL_SPEAKER_SPEED = 0.65;

        public static final double SHOOTERH_AMP_SPEED = 0.17;
        public static final double SHOOTERL_AMP_SPEED = 0.25;

        public static final double SHOOTERH_PASS_SPEED = 0.98;
        public static final double SHOOTERL_PASS_SPEED = 0.98;

        public static final double SHOOTER_OUT_SPEED = -0.8;

        public static final double SHOOTER_IDLE_SPEED = 0.5;
    }

    public static class LedConstants {
        public static final int LED_PORT = 0;
        public static final int LED_BUFFER_LENGTH = 40;
    }

    public class SwerveConstants {

        public static final double VOLTAGE_CLOSED_LOOP_RAMP_PERIOD = 0.0003;// 0.0003

        public static final double statorCurrent = 110;
        public static final double supplyCurrent = 50;

        public static final ClosedLoopRampsConfigs rampConfigs = new ClosedLoopRampsConfigs()
                .withVoltageClosedLoopRampPeriod(0.3);

        public static final Measure<Voltage> MAX_VOLTAGE = Volts.of(12.0);

        public static final int PIGEON_ID = 14;

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withCANbusName(RobotConstants.CAN_BUS_NAME)
                .withPigeon2Id(PIGEON_ID)
                .withPigeon2Configs(null); // optional

        /** The max speed of the swerve (should not larger than speedAt12Volts) */
        public static final Measure<Velocity<Distance>> maxSpeed = MetersPerSecond.of(4.5);
        /** The max turning speed of the swerve */
        public static final Measure<Velocity<Angle>> maxAngularRate = RotationsPerSecond.of(1.2 * Math.PI);

        public static final double deadband = maxSpeed.magnitude() * 0.01;
        public static final double rotationalDeadband = maxAngularRate.magnitude() * 0.01;

        public static final SlewRateLimiter xLimiter = new SlewRateLimiter(3, -3.25, 0);
        public static final SlewRateLimiter yLimiter = new SlewRateLimiter(3, -3.25, 0);

        /** Gearing between the drive motor output shaft and the wheel. */
        private static final double DRIVE_GEAR_RATIO = 6.7460317460317460317460317460317;
        /** Gearing between the steer motor output shaft and the azimuth gear. */
        private static final double STEER_GEAR_RATIO = 21.428571428571428571428571428571;

        /** Radius of the wheel in meters. */
        private static final Measure<Distance> wheelRadius = Meters.of(0.05);

        /** The stator current at which the wheels start to slip */
        private static final Measure<Current> slipCurrent = Amps.of(150.0);

        public static class steerGainsClass {
            public static final TunableNumber STEER_KP = new TunableNumber("STEER PID/kp", 120);
            public static final TunableNumber STEER_KI = new TunableNumber("STEER PID/ki", 0.2);
            public static final TunableNumber STEER_KD = new TunableNumber("STEER PID/kd", 0.005);
            public static final TunableNumber STEER_KA = new TunableNumber("STEER PID/ka", 0);
            public static final TunableNumber STEER_KV = new TunableNumber("STEER PID/kv", 0);
            public static final TunableNumber STEER_KS = new TunableNumber("STEER PID/ks", 0);
        }

        public static class driveGainsClass {
            public static final TunableNumber DRIVE_KP = new TunableNumber("DRIVE PID/kp", 0.03);
            public static final TunableNumber DRIVE_KI = new TunableNumber("DRIVE PID/ki", 0);
            public static final TunableNumber DRIVE_KD = new TunableNumber("DRIVE PID/kd", 0.0001);
            public static final TunableNumber DRIVE_KA = new TunableNumber("DRIVE PID/ka", 0);
            public static final TunableNumber DRIVE_KV = new TunableNumber("DRIVE PID/kv", 0.12);
            public static final TunableNumber DRIVE_KS = new TunableNumber("DRIVE PID/ks", 0.14);
        }

        /** Swerve steering gains */
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(120)// 120
                .withKI(0.2)// 0.2
                .withKD(0.005)// 0.005
                .withKS(0)
                .withKV(0)
                .withKA(0);

        /** Swerve driving gains */
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(1)
                .withKI(0)
                .withKD(0)
                .withKS(0)
                .withKV(0.12)
                .withKA(0);

        /**
         * The closed-loop output type to use for the steer motors;
         * This affects the PID/FF gains for the steer motors
         */
        private static final SwerveModule.ClosedLoopOutputType steerClosedLoopOutput = SwerveModule.ClosedLoopOutputType.Voltage;

        /**
         * The closed-loop output type to use for the drive motors;
         * This affects the PID/FF gains for the drive motors
         */
        private static final SwerveModule.ClosedLoopOutputType driveClosedLoopOutput = SwerveModule.ClosedLoopOutputType.Voltage;

        /** Theoretical free speed (m/s) at 12v applied output; */
        public static final Measure<Velocity<Distance>> speedAt12Volts = MetersPerSecond.of(6.0);

        /** Simulation only */
        private static final double STEER_INERTIA = 0.00001;
        /** Simulation only */
        private static final double DRIVE_INERTIA = 0.001;
        /** Simulation only */
        private static final Measure<Voltage> steerFrictionVoltage = Volts.of(0.25);
        /** Simulation only */
        private static final Measure<Voltage> driveFrictionVoltage = Volts.of(0.25);

        /**
         * Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
         */
        private static final double COUPLE_RATIO = 3.5;

        private static final boolean STEER_MOTOR_REVERSED = true;

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
                .withSteerMotorGearRatio(STEER_GEAR_RATIO)
                .withWheelRadius(wheelRadius.in(Inches))
                .withSlipCurrent(slipCurrent.magnitude())
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSpeedAt12VoltsMps(speedAt12Volts.magnitude())
                .withSteerInertia(STEER_INERTIA)
                .withDriveInertia(DRIVE_INERTIA)
                .withSteerFrictionVoltage(steerFrictionVoltage.magnitude())
                .withDriveFrictionVoltage(driveFrictionVoltage.magnitude())
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.SyncCANcoder)
                .withCouplingGearRatio(COUPLE_RATIO)
                .withSteerMotorInverted(STEER_MOTOR_REVERSED);

        // Front Left
        private static final int FRONT_LEFT_DRIVE_MOTOR_ID = 13;
        private static final int FRONT_LEFT_STEER_MOTOR_ID = 4;
        private static final int FRONT_LEFT_ENCODER_ID = 8;
        private static final double FRONT_LEFT_ENCODER_OFFSET = -0.60466015625;// 0.052955;//0.127686//0.5329550781
        private static final Measure<Distance> frontLeftXPos = Meters.of(0.5);
        private static final Measure<Distance> frontLeftYPos = Meters.of(0.5);

        // Front Right
        private static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 1;
        private static final int FRONT_RIGHT_STEER_MOTOR_ID = 5;
        private static final int FRONT_RIGHT_ENCODER_ID = 9;
        private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.309041015625;// 0.125685;//0.13623046875//0.117686//0.046875
        private static final Measure<Distance> frontRightXPos = Meters.of(0.5);
        private static final Measure<Distance> frontRightYPos = Meters.of(-0.5);

        // Back Left
        private static final int BACK_LEFT_DRIVE_MOTOR_ID = 2;
        private static final int BACK_LEFT_STEER_MOTOR_ID = 6;
        private static final int BACK_LEFT_ENCODER_ID = 10;
        private static final double BACK_LEFT_ENCODER_OFFSET = 0.666462890625;// 0.773925;//-0.223//0.401611//0.77392578125
        private static final Measure<Distance> backLeftXPos = Meters.of(-0.5);
        private static final Measure<Distance> backLeftYPos = Meters.of(0.5);

        // Back Right
        private static final int BACK_RIGHT_DRIVE_MOTOR_ID = 3;
        private static final int BACK_RIGHT_STEER_MOTOR_ID = 7;
        private static final int BACK_RIGHT_ENCODER_ID = 11;
        private static final double BACK_RIGHT_ENCODER_OFFSET = 0.257337890625;// 0.422119;//-0.5684550781//-0.064453//0.432279296875
        private static final Measure<Distance> backRightXPos = Meters.of(-0.5);
        private static final Measure<Distance> backRightYPos = Meters.of(-0.5);

        public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_ENCODER_ID,
                FRONT_LEFT_ENCODER_OFFSET,
                frontLeftXPos.magnitude(),
                frontLeftYPos.magnitude(),
                false);
        public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_ENCODER_ID,
                FRONT_RIGHT_ENCODER_OFFSET,
                frontRightXPos.magnitude(),
                frontRightYPos.magnitude(),
                true);
        public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_ENCODER_ID,
                BACK_LEFT_ENCODER_OFFSET,
                backLeftXPos.magnitude(),
                backLeftYPos.magnitude(),
                false);
        public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_ENCODER_ID,
                BACK_RIGHT_ENCODER_OFFSET,
                backRightXPos.magnitude(),
                backRightYPos.magnitude(),
                true);

        public static SwerveModuleConstants[] modules = { FrontLeft, FrontRight, BackLeft, BackRight };

        public static final Translation2d[] modulePlacements = new Translation2d[] {
                new Translation2d(SwerveConstants.FrontLeft.LocationX,
                        SwerveConstants.FrontLeft.LocationY),
                new Translation2d(SwerveConstants.FrontRight.LocationX,
                        SwerveConstants.FrontRight.LocationY),
                new Translation2d(SwerveConstants.BackLeft.LocationX,
                        SwerveConstants.BackLeft.LocationY),
                new Translation2d(SwerveConstants.BackRight.LocationX,
                        SwerveConstants.BackRight.LocationY)
        };

        public static final KinematicLimits DRIVETRAIN_UNCAPPED = new KinematicLimits(
                4.5,
                50.0,
                2000.0);
        public static final KinematicLimits DRIVETRAIN_SMOOTHED = new KinematicLimits(
                4.5,
                30.0,
                200.0);
        public static final KinematicLimits DRIVETRAIN_LIMITED = new KinematicLimits(
                2.0,
                10.0,
                1200.0);
        public static final KinematicLimits DRIVETRAIN_ROBOT_ORIENTED = new KinematicLimits(
                2.0,
                5.0,
                1500.0);

        public static final SimpleMotorFeedforward DRIVETRAIN_FEEDFORWARD = new SimpleMotorFeedforward(
                0.69522, 2.3623, 0.19367);

        /** Spin PID */
        public static final Slot0Configs headingGains = new Slot0Configs()
                .withKP(0.04)
                .withKI(0)
                .withKD(0);

        public static class headingController {
            public static final frc.robot.utils.TunableNumber HEADING_KP = new frc.robot.utils.TunableNumber("HEADING PID/kp", 0.08);
            public static final frc.robot.utils.TunableNumber HEADING_KI = new frc.robot.utils.TunableNumber("HEADING PID/ki", 0.0002);
            public static final frc.robot.utils.TunableNumber HEADING_KD = new frc.robot.utils.TunableNumber("HEADING PID/kd", 0.002);
            public static final frc.robot.utils.TunableNumber MAX_ERROR_CORRECTION_ANGLE = new frc.robot.utils.TunableNumber("HEADING/Max Error Correction Angle", 90.0);
            // TODO
        }

    }

    public static class VisionConstants {
        public static final String AIM_LIMELIGHT_NAME = "limelight";

        public static double REJECT_ANGULAR_SPEED = 360;// degree
        public static double REJECT_LINEAR_SPEED = 2.5;// m/s
    }

}
