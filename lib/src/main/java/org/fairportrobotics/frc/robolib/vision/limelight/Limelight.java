// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.fairportrobotics.frc.robolib.vision.limelight;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.geometry.Pose3d;

public class Limelight {
  private String name;
  private SwerveDrivePoseEstimator3d poseEstimator;
  private Pose3d pose;

  public Limelight(String name, SwerveDrivePoseEstimator3d poseEstimator, Pose3d pose) {
    this.name = name;
    this.poseEstimator = poseEstimator;
    this.pose=pose;
  } 

  public LimelightHelpers.PoseEstimate updateOdometry() {      
      LimelightHelpers.SetRobotOrientation(name, Math.toDegrees(poseEstimator.getEstimatedPosition().getRotation().getZ()), 0, 0, 0, 0, 0);
      return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }
}
