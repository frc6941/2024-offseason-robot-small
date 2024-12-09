package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor;

    public Arm() {
        armMotor = new TalonFX(0);
        armMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(0.2)
                .withKI(0)
                .withKD(0.001)
                .withKA(0.0037512677)
                .withKV(0.115)
                .withKS(0.28475008)
        );
        armMotor.getConfigurator().apply(new TalonFXConfiguration()
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(3)
                        .withMotionMagicCruiseVelocity(2)
                )
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake))

                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(90d / 24 * 90 / 24 * 84 / 14))
        );

    }

    public void setArmHome(Measure<Angle> rad) {
        armMotor.setPosition(rad.in(Rotations));
    }


    public void setArmPosition(Measure<Angle> rad) {
        armMotor.setControl(new MotionMagicVoltage(rad.in(Rotations)));
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Arm/Position", Radians.of(Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble())));
    }

}
