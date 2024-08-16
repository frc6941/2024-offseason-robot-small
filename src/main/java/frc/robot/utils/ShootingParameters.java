package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Data;

@Data
public class ShootingParameters {
    private double distance;
    private double shootingVelocityL;
    private double shootingVelocityH;
    private Rotation2d fieldAimingAngle;

    public ShootingParameters(double distance,
            double shootingVelocityL,
            double shootingVelocityH,
            Rotation2d fieldAimingAngle) {
        this.distance = distance;
        this.shootingVelocityL = shootingVelocityL;
        this.shootingVelocityH = shootingVelocityH;
        this.fieldAimingAngle = fieldAimingAngle;
    }
}