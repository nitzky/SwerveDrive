// This is the Limelight subsytem for 2024

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {

    @Override
    public void periodic() {

        /////////////////////////////////////////////////////////////////////////////////////////////////////
        // copied from limelight docs - "estimating distance"
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 25.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 20.0;

        // distance from the target to the floor
        double goalHeightInches = 60.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)
                / Math.tan(angleToGoalRadians);
        //////////////////////////////////////////////////////////////////////////////////////////////////////

        // get tx ty and ta values
        double tx = LimelightHelpers.getTX("limelight");
        // double ty = LimelightHelpers.getTY("limelight");
        double area = LimelightHelpers.getTA("limelight");

        // post values to smart dashboard periodically
        SmartDashboard.putNumber("Limelight", tx);
        // SmartDashboard.putNumber("Limelight", ty);
        SmartDashboard.putNumber("Limelight", area);
        SmartDashboard.putNumber("Limelight", distanceFromLimelightToGoalInches);

    }

}