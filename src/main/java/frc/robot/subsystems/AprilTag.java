package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.*;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// download link and instructions https://docs.limelightvision.io/docs/docs-limelight/getting-started/imaging
public class AprilTag extends SubsystemBase {

    // find april tag with our id and keep scanning until it's found

    // here we are implementing code directly from the Limelight API 
    // LimelightHelpers.setLEDMode_PipelineControl("");
    // LimelightHelpers.setLEDMode_ForceBlink("")
    // LimelightHelpers.setCropWindow("",-1,1,-1,1);
    // double tx = LimelightHelpers.getTX("");
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
    NetworkTable nt = LimelightHelpers.getLimelightNTTable("limelight");
    getLimelightNTTableEntry();
    getLimelightNTDouble();
    setLimelightNTDouble();
    setLimelightNTDoubleArray();
    getLimelightNTDouleArray();
    getLimelightNTString();
    getLimelightURLString();



}
