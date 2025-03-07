// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable limelighttable;
  int aprilTagId = -1;
  AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);


  /** Creates a new Limelight. */
  public Limelight() {
    this.limelighttable = NetworkTableInstance.getDefault().getTable("limelight");
  }


  public Pose2d getAprilTagPose() {
    Translation2d translation = new Translation2d(fieldLayout.getTagPose(aprilTagId).get().getX(), fieldLayout.getTagPose(aprilTagId).get().getY());
    Rotation2d rot = new Rotation2d(fieldLayout.getTagPose(aprilTagId).get().getRotation().getAngle());
    return new Pose2d(translation, rot);
  }

  @Override
  public void periodic() {
    int id =  (int) limelighttable.getEntry("tid").getDouble(-1);

    if (id != -1) {
     aprilTagId = id;
    }
  }
}
