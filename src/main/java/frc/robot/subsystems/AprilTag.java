package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.*;
import frc.robot.LimelightHelpers;

public class AprilTag extends SubsystemBase {

    // here we are implementing code directly from the Limelight API 
    // LimelightHelpers.setLEDMode_PipelineControl("");
    // LimelightHelpers.setLEDMode_ForceBlink("")
    // LimelightHelpers.setCropWindow("",-1,1,-1,1);
    // double tx = LimelightHelpers.getTX("");
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");


}
