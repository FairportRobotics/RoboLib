// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.fairportrobotics.frc.robolib.vision.limelight;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class Limelight {
  private String name;
  private Pigeon2 gyro;
  private SwerveDrivePoseEstimator3d poseEstimator;

  public Limelight(String name, Pigeon2 gyro, SwerveDrivePoseEstimator3d poseEstimator) {
    this.name = name;
    this.gyro = gyro;
    this.poseEstimator = poseEstimator;
  } 

  public LimelightHelpers.PoseEstimate updateOdometry() {
      boolean useMegaTag2 = true; //set to false to use MegaTag1
      boolean doRejectUpdate = false;
      
      LimelightHelpers.SetRobotOrientation(name, Math.toDegrees(poseEstimator.getEstimatedPosition().getRotation().getZ()), 0, 0, 0, 0, 0);
      return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    }
}
