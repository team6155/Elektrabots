// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {
  private final SwerveDrivePoseEstimator poseEstimator;
  /** Creates a new Vision. */
  public Vision(SwerveDrivePoseEstimator poseEstimator) {
    this.poseEstimator = poseEstimator;
  }

  public boolean foundAprilTag(){
    return LimelightHelpers.getTargetCount("") > 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Limelightx", LimelightHelpers.getTX(""));
    SmartDashboard.putNumber("Limelighty", LimelightHelpers.getTY(""));
    SmartDashboard.putNumber("Limelighta", LimelightHelpers.getTA(""));

    LimelightHelpers.SetRobotOrientation("", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate megaTag = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

    if(megaTag.tagCount > 0){
      poseEstimator.addVisionMeasurement(megaTag.pose,megaTag.timestampSeconds);
    }  
    SmartDashboard.putNumber("poseX", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("poseY", poseEstimator.getEstimatedPosition().getY());
  }
}

