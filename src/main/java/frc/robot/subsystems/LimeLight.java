package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;

public class LimeLight extends SubsystemBase {
    /* Creates new LimeLight */
    NetworkTable table;

    private boolean validTargets;
    private double horizontalOffset;
    private double verticalOffset;
    private double targetArea;
    private Pose3d botPose3d;

    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        botPose3d = new Pose3d();
    }

    @Override
    public void periodic() {
        validTargets = isTargetValid();
        horizontalOffset = getHorizontalOffset();
        verticalOffset = getVerticalOffset();
        targetArea = getTargetArea();
    }

    public boolean isTargetValid() {
        return (table.getEntry("tv").getDouble(0) == 1);
    }

    public double getHorizontalOffset() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getVerticalOffset() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getTargetArea() {
        return table.getEntry("ta").getDouble(0);
    }

    public Pose3d getBotPose3d() {
         var poseArrary = table.getEntry("botpose").getDoubleArray(new double[6]);
         botPose3d = new Pose3d(poseArrary[0], poseArrary[1], poseArrary[2], new Rotation3d(poseArrary[3], poseArrary[4], poseArrary[5]));
         return botPose3d;
    }
}
