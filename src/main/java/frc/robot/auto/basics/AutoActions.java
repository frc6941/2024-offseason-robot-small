package frc.robot.auto.basics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;

import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import lombok.Synchronized;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class AutoActions {
    private final static Swerve swerve = Swerve.getInstance();

    @Getter
    private static Map<String, Command> eventMap = new HashMap<>();

    static {
    }


    // private final static FullAutoBuilder autoBuilder = new FullAutoBuilder(
    //         swerve,
    //         swerve::resetPose,
    //         eventMap
    // );


    @Synchronized
    public static PathPlannerTrajectory getTrajectory(String name) {
        return new PathPlannerTrajectory(
            PathPlannerPath.fromPathFile(name),
            swerve.getChassisSpeeds(),
            swerve.getLocalizer().getLatestPose().getRotation()
            );
    }


    //todo: Think Choreo
    // private PathPlannerPath getPath(String name){
    //     return PathPlannerPath.fromChoreoTrajectory(null);
    // }


    // @Synchronized
    // public static List<PathPlannerTrajectory> getTrajectoryGroup(String name, PathConstraints constraints) {
    //     return PathPlanner.loadPathGroup(name, constraints);
    // }

    

    // ToDo: think about following with events

    // @Synchronized
    // public static Command followTrajectoryWithEvents(PathPlannerTrajectory trajectory, boolean lockAngle) {
    //     return new FollowTrajectoryWithEvents(swerve, trajectory, lockAngle, true, eventMap);
    // }

    @Synchronized
    public static Command followTrajectory(PathPlannerTrajectory trajectory, boolean lockAngle) {
        return new FollowTrajectory(swerve, trajectory, lockAngle, true);
    }

    @Synchronized
    public static Command resetOdometry(Pose2d startingPose) {
        return new InstantCommand(() -> swerve.resetPose(startingPose));
    }


    @Synchronized
    public static Command waitFor(double seconds) {
        return new WaitCommand(seconds);
    }

    @Synchronized
    public static Command print(String message) {
        return new PrintCommand(message);
    }

    // @Synchronized
    // public static Command fullAuto(PathPlannerTrajectory trajectory) {
    //     return autoBuilder.fullAuto(trajectory);
    // }

    // @Synchronized
    // public static Command fullAuto(List<PathPlannerTrajectory> trajectories) {
    //     return new WaitUntilCommand(
    //             () -> (intaker.isHomed() && hood.isCalibrated()) || !Constants.IS_REAL
    //     ).andThen(
    //             autoBuilder.fullAuto(trajectories)
    //     );
    // }
}
