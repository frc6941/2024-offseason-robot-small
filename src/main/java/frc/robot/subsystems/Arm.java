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
import frc.robot.RobotConstants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

public class Arm extends SubsystemBase {
    public final TalonFX armMotor;

    public Arm() {
        armMotor = new TalonFX(20, RobotConstants.CAN_BUS_NAME);

        armMotor.getConfigurator().apply(new TalonFXConfiguration()
                .withMotionMagic(new MotionMagicConfigs()
                        .withMotionMagicAcceleration(1000)
                        .withMotionMagicCruiseVelocity(1000)
                )
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake))

                .withFeedback(new FeedbackConfigs()
                        .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                        .withSensorToMechanismRatio(175d / 1))
        );
        armMotor.getConfigurator().apply(new Slot0Configs()
                .withKP(100)
                .withKI(0)
                .withKD(0.001)
                .withKA(0)
                .withKV(0)
                .withKS(0)
        );
        armMotor.setPosition(0);

    }

    public void setArmHome(Measure<Angle> rad) {
        armMotor.setPosition(rad.in(Rotations));

    }

    public void setArmVoltage(double voltage) {
        armMotor.setVoltage(voltage);
    }

    public Measure<Angle> getArmPosition() {
        return Radians.of(Units.rotationsToRadians(armMotor.getPosition().getValueAsDouble() / 175));
    }


    public void setArmPosition(Measure<Angle> rad) {
        armMotor.setControl(new MotionMagicVoltage(Units.degreesToRotations(rad.magnitude())));
    }

    @Override
    public void periodic() {

        Logger.recordOutput("Arm/position", getArmPosition());
    }

}
