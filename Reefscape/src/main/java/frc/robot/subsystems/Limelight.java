// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  NetworkTable limelighttable;
  // NetworkTable limelightableBack;
  int aprilTagId = -1;
  AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight Subsystem");


  /** Creates a new Limelight. */
  public Limelight() {
    this.limelighttable = NetworkTableInstance.getDefault().getTable("limelight");
    // this.limelightableBack = NetworkTableInstance.getDefault().getTable("limelight-back");
  }

  public Pose2d getLimelightBotPose() {
      // double[] botpose = limelighttable.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
      // Translation2d transl = new Translation2d(botpose[0], botpose[1]);
      // Rotation2d rot = new Rotation2d(botpose[5]);
      // return new Pose2d(transl, rot);
      return LimelightHelpers.getBotPose3d_TargetSpace("").toPose2d();

  }


  public Pose2d getTargetPose() {
      if (aprilTagId != -1) {
        Translation2d finalTranslation;
        Rotation2d aprilTagRotation;
        Pose3d aprilTagpose = fieldLayout.getTagPose(aprilTagId).get();
        Translation2d aprilTagTranslation = new Translation2d(aprilTagpose.getX(), aprilTagpose.getY());
        aprilTagRotation = new Rotation2d(aprilTagpose.getRotation().getAngle());
        finalTranslation = new Translation2d(aprilTagTranslation.getX()-Math.cos(aprilTagRotation.getDegrees())*.45, aprilTagTranslation.getY()-Math.sin(aprilTagRotation.getDegrees())*.45);    
        return new Pose2d(finalTranslation, aprilTagRotation);
      } else {
        return new Pose2d();
      }
  }


  public double getTX() {
    return limelighttable.getEntry("tx").getDouble(-1);
  }

  public int getAprilTag() {
    return (int) limelighttable.getEntry("tid").getDouble(-1);
  }

  // public int getAprilTagBack() {
  //   return (int) limelightableBack.getEntry("tid").getDouble(-1);
 
  // }

  public double getTY() {
    return limelighttable.getEntry("ty").getDouble(-1);
  }

  // public double getBackTY() {
  //   return limelightableBack.getEntry("ty").getDouble(-1);
  // }

  // public double getBackTX() {
  //   return limelightableBack.getEntry("tx").getDouble(-1);
  // }

  public double getRotation() {
    if (aprilTagId != -1) {
      Pose3d aprilTagpose = fieldLayout.getTagPose(aprilTagId).get();
      return aprilTagpose.getRotation().getAngle();
    } else {
      return 0.0;
    }
  }


//Rory was Here
  @Override
  public void periodic() {
    int id =  (int) limelighttable.getEntry("tid").getDouble(-1);

    if (id != -1) {
     aprilTagId = id;
    }
  }
}
