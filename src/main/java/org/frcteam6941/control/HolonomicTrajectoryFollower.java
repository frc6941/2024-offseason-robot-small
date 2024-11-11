package org.frcteam6941.control;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPlannerTrajectory.State;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

public class HolonomicTrajectoryFollower extends PathPlannerTrajectoryFollowerBase<HolonomicDriveSignal> {
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController thetaController;
    private final SimpleMotorFeedforward feedforward;

    private State lastState = null;
    private Pose2d actualPose = null;


    private boolean finished = false;
    private boolean requiredOnTarget = false;
    private boolean lockAngle = true;

    private double TARGET_DISTANCE_ACCURACY_REQUIREMENT = 0.2;
    private double TARGET_VELOCITY_ACCURACY_REQUIREMENT = 0.25;


    public HolonomicTrajectoryFollower(PIDController xController, PIDController yController,
                                       ProfiledPIDController thetaController, SimpleMotorFeedforward feedforward) {
        this.xController = xController;
        this.yController = yController;
        this.thetaController = thetaController;
        this.feedforward = feedforward;

        this.xController.setTolerance(TARGET_DISTANCE_ACCURACY_REQUIREMENT, TARGET_VELOCITY_ACCURACY_REQUIREMENT);
        this.yController.setTolerance(TARGET_DISTANCE_ACCURACY_REQUIREMENT, TARGET_VELOCITY_ACCURACY_REQUIREMENT);
    }

    @Override
    protected HolonomicDriveSignal calculateDriveSignal(Pose2d currentPose, Translation2d velocity,
                                                        double rotationalVelocity, PathPlannerTrajectory trajectory, double time,
                                                        double dt) {
        if (time > trajectory.getTotalTimeSeconds()) {
            if (this.requiredOnTarget) {
                if (this.xController.atSetpoint() && this.yController.atSetpoint()) {
                    finished = true;
                    return new HolonomicDriveSignal(new Translation2d(0, 0), 0.0, true, false);
                }
            } else {
                finished = true;
                return new HolonomicDriveSignal(new Translation2d(0, 0), 0.0, true, false);
            }
        }

        actualPose = currentPose;

        lastState = (State) trajectory.sample(time);
        double x = xController.calculate(currentPose.getX(), lastState.positionMeters.getX());
        double y = yController.calculate(currentPose.getY(), lastState.positionMeters.getY());
        double rotation = 0.0;
        Translation2d translationVector = new Translation2d(x, y);


        if (this.lastState != null) {
            Translation2d targetDisplacement = lastState.positionMeters
                    .minus(this.lastState.positionMeters);
            double feedForwardGain = feedforward.calculate(lastState.velocityMps,
                    lastState.accelerationMpsSq) / 12.0;

            if (targetDisplacement.getNorm() != 0.00) { // Prevent NaN cases
                Translation2d feedForwardVector = targetDisplacement
                        .times(feedForwardGain / targetDisplacement.getNorm());
                translationVector = translationVector.plus(feedForwardVector);
            }
        }

        if (this.lockAngle) {
            rotation = this.thetaController.calculate(currentPose.getRotation().getDegrees(),
                    lastState.targetHolonomicRotation.getDegrees());
        }

        return new HolonomicDriveSignal(
                translationVector,
                rotation,
                true,
                false);
    }

    public State getLastState() {
        return lastState;
    }

    public void setLockAngle(boolean lock) {
        this.lockAngle = lock;
    }

    public void setRequiredOnTarget(boolean requiredOnTarget) {
        this.requiredOnTarget = requiredOnTarget;
    }

    public void setTolerance(double distance, double velocity) {
        this.TARGET_DISTANCE_ACCURACY_REQUIREMENT = distance;
        this.TARGET_VELOCITY_ACCURACY_REQUIREMENT = velocity;
    }

    @Override
    protected boolean isFinished() {
        return finished;
    }

    public boolean isPathFollowing() {
        return !finished;
    }

    public Pose2d[] getTrajectoryPoses() {
        PathPlannerTrajectory trajectory = getCurrentTrajectory().get();
        Pose2d[] poses = new Pose2d[trajectory.getStates().size()];
        for (int i = 0; i < trajectory.getStates().size(); i++) {
            State state = trajectory.getStates().get(i);
            poses[i] = state.getTargetHolonomicPose();
        }
        return poses;
    }


    public void sendData() {
        if (isPathFollowing() && this.lastState != null && getCurrentTrajectory() != null) {

            Logger.recordOutput("swerve/PathPlanner/lastState", this.lastState.getTargetHolonomicPose());
            Logger.recordOutput("swerve/PathPlanner/xErrorV", xController.getVelocityError());
            Logger.recordOutput("swerve/PathPlanner/xErrorP", xController.getPositionError());


        }
    }

    @Override
    protected void reset() {
        this.xController.reset();
        this.yController.reset();
        this.finished = false;
    }
}
