package frc.robot.display;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.utils.ShootingDecider.Destination;
import lombok.Getter;

@Getter
public class OperatorDashboard {
    private static OperatorDashboard instance;
    private final ShuffleboardTab operatorTab;
    private final GenericEntry intakerOn,intakerGet;
    // TargetStatus
    @Getter
    private Destination currDestination = Destination.SPEAKER;

    private OperatorDashboard() {
        operatorTab = Shuffleboard.getTab("Operator");
        intakerGet = operatorTab
                .add("intakerGet", false)
                .withPosition(0, 1)
                .withSize(1, 1)
                .getEntry();
        intakerOn = operatorTab
                .add("intakerOn", false)
                .withPosition(0, 2)
                .withSize(1, 1)
                .getEntry();
    }

    public static OperatorDashboard getInstance() {
        if (instance == null) {
            instance = new OperatorDashboard();
        }
        return instance;
    }

    public void registerAutoSelector(SendableChooser<String> selector) {
        operatorTab
                .add("Auto Selector", selector)
                .withSize(2, 1)
                .withPosition(0, 0);
    }

    public void updateRobotStatus(boolean intakerOn,boolean intakerGet){
        this.intakerOn.setBoolean(intakerOn);
        this.intakerGet.setBoolean(intakerGet);
    }
}