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

public class Limelight extends SubsystemBase {
  NetworkTable limelighttable;
  int aprilTagId = -1;
  AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("Limelight Subsystem");


  /** Creates a new Limelight. */
  public Limelight() {
    this.limelighttable = NetworkTableInstance.getDefault().getTable("limelight");
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
    }
    return new Pose2d(new Translation2d(), new Rotation2d());
    // if (isLeft) {
    //   double newX = finalTranslation.getX()+(.45*(-Math.sin(aprilTagRotation.getRadians())));
    //   double newY = finalTranslation.getY()+(.45*(Math.cos(aprilTagRotation.getRadians())));
    //   Translation2d targetTranslation = new Translation2d(newX, newY);
    //   return new Pose2d(targetTranslation, aprilTagRotation);
    // } else {
    //   double newX = finalTranslation.getX()+(.45*(Math.sin(aprilTagRotation.getRadians())));
    //   double newY = finalTranslation.getY()+(.45*(Math.cos(aprilTagRotation.getRadians())));
    //   Translation2d targetTranslation = new Translation2d(newX, newY);
    //   return new Pose2d(targetTranslation, aprilTagRotation);
    // }
    
    
  }
//Rory was Here
  @Override
  public void periodic() {
    int id =  (int) limelighttable.getEntry("tid").getDouble(-1);

    if (id != -1) {
     aprilTagId = id;
     table.putValue("Is AprilTag Detected", NetworkTableValue.makeBoolean(true));
    } else {
      table.putValue("Is AprilTag Detected", NetworkTableValue.makeBoolean(false));
    }
  }
}
