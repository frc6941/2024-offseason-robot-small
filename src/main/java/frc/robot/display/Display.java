package frc.robot.display;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.Swerve;
import org.frcteam6941.looper.Updatable;

public class Display implements Updatable {
    FieldView fieldView;
    Swerve swerve;

    private static Display instance;

    private Display() {
        swerve = Swerve.getInstance();
        fieldView = new FieldView();
    }

    public static Display getInstance() {
        if (instance == null) {
            instance = new Display();
        }
        return instance;
    }

    public void setFerryLocation(Translation2d pos) {
        fieldView.setFerryLocation(pos);
    }

    @Override
    public void update(double time, double dt) {
        fieldView.update(
            swerve.getLocalizer().getLatestPose(), 
            swerve.getLocalizer().getPredictedPose(0.02)
        );
    }
}
