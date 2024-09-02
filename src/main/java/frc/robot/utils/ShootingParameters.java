package frc.robot.utils;

import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.math.geometry.Rotation2d;
import lombok.Data;

@Data
public class ShootingParameters {
    private double distance;
    private double shootingVelocityL;
    private double shootingVelocityH;
    private Rotation2d fieldAimingAngle;
    private edu.wpi.first.math.geometry.Translation2d delta;

    public ShootingParameters(double distance,
            double shootingVelocityL,
            double shootingVelocityH,
            Rotation2d fieldAimingAngle, edu.wpi.first.math.geometry.Translation2d delta) {
        this.distance = distance;
        this.shootingVelocityL = shootingVelocityL;
        this.shootingVelocityH = shootingVelocityH;
        this.fieldAimingAngle = fieldAimingAngle;
        this.delta = delta;
    }




}