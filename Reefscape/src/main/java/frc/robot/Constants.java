// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxSpeed = 3.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static boolean fieldRelative = true;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs


    public static final int kFrontLeftDrivingCanId = 3; // 3FLDM
    public static final int kRearLeftDrivingCanId = 7; // 7BLDM
    public static final int kFrontRightDrivingCanId = 1; // 1FRDM
    public static final int kRearRightDrivingCanId = 5; // 5BRDM

    public static final int kFrontLeftTurningCanId = 4; // 4FLSM
    public static final int kRearLeftTurningCanId = 8; // 8BLSM
    public static final int kFrontRightTurningCanId = 2; // 2FRSM
    public static final int kRearRightTurningCanId = 6; // 6BRSM

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 13;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    // public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.12;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI*.8;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
  public static final class ArmConstants {
    // public static final double kArmInitPos = 44.2; //Degrees
    public static final double kArmInitOffset = -0.139; // Rotations
    public static final double kLFourPosition = 161; // degrees
    public static final double kLThreePosition = 74;
    public static final double kLTwoPosition = 48;
    public static final double kArmDownPosition = 4;
    // Coral FeedForward values
    public static final double kS = 0;
    public static final double kGC = 0.055;
    public static final double kV = 0.2;
    public static final double kA = 0;
    // Empty FeedForward values
    public static final double kGE = 0.05;
  }

  public static final class ClimbConstants {
    public static final double kMaxClimberPosition = 0;
    public static final double kMinClimberPosition = 0;
  }

  public static final class LimelightConstants {
    public static final double kLFourLeftTX = -17.48;
    public static double kLFourRightTX = 9.88;
    public static final double kLTwoThreeLeftTX = 0;
    public static final double kLTwoThreeRightTX = 0;
    public static final double kLTwoThreeTY = 0;
  }


  public static final class DriverButtonBindings {
    /*
     * For Arm setpoints, there are dpad values ranging from 0-270.
     * 0 - Up
     * 90 - Right
     * 180 - Down
     * 270 - Left
     */
    public static final int DPAD_DOWN = 180;
    public static final int DPAD_UP = 0;
    public static final int DPAD_RIGHT = 90;
    public static final int DPAD_LEFT = 270;

    public static final int[] DAVID = {
      XboxController.Button.kY.value, // Arm Up Manual
      XboxController.Button.kA.value, // Arm Down Manual
      XboxController.Button.kLeftBumper.value, // Release Coral AND Intake (INVERSE)
      XboxController.Button.kRightBumper.value, // Release Coral AND Intake
      XboxController.Button.kB.value, // Climber (Unclimb)
      XboxController.Button.kX.value, // Climber (Climb)
      DPAD_DOWN, // ArmDown Setpoint
      DPAD_RIGHT, // L4 Setpoint
      DPAD_UP, // L3 Setpoint
      DPAD_LEFT, // L2 Setpoint
      XboxController.Button.kBack.value, // AutoAlign Left AND L4 Setpoint
      XboxController.Button.kStart.value // AutoAlign Right AND L4 Setpoint
    };

    public static final int[] ZACH = {
      XboxController.Button.kY.value, // Arm Up Manual
      XboxController.Button.kA.value, // Arm Down Manual
      XboxController.Button.kLeftBumper.value, // Release Coral AND Intake (INVERSE)
      XboxController.Button.kRightBumper.value, // Release Coral AND Intake
      XboxController.Button.kB.value, // Climber (Unclimb)
      XboxController.Button.kX.value, // Climber (Climb)
      DPAD_DOWN, // ArmDown Setpoint
      DPAD_RIGHT, // L4 Setpoint
      DPAD_UP, // L3 Setpoint
      DPAD_LEFT, // L2 Setpoint
      XboxController.Button.kBack.value, // AutoAlign Left AND L4 Setpoint
      XboxController.Button.kStart.value // AutoAlign Right AND L4 Setpoint
    };

    public static final int[] JONAH = {
      XboxController.Button.kY.value, // Arm Up Manual
      XboxController.Button.kA.value, // Arm Down Manual
      XboxController.Button.kLeftBumper.value, // Release Coral AND Intake (INVERSE)
      XboxController.Button.kRightBumper.value, // Release Coral AND Intake
      XboxController.Button.kB.value, // Climber (Unclimb)
      XboxController.Button.kX.value, // Climber (Climb)
      DPAD_DOWN, // ArmDown Setpoint
      DPAD_RIGHT, // L4 Setpoint
      DPAD_UP, // L3 Setpoint
      DPAD_LEFT, // L2 Setpoint
      XboxController.Button.kBack.value, // AutoAlign Left AND L4 Setpoint
      XboxController.Button.kStart.value // AutoAlign Right AND L4 Setpoint
    };

    public static final int[] NICHOLAS = {
      XboxController.Button.kB.value, // Arm Up Manual
      XboxController.Button.kA.value, // Arm Down Manual
      XboxController.Button.kLeftBumper.value, // Release Coral AND Intake (INVERSE)
      XboxController.Button.kRightBumper.value, // Release Coral AND Intake
      XboxController.Button.kX.value, // Climber (Unclimb)
      XboxController.Button.kY.value, // Climber (Climb)
      DPAD_DOWN, // ArmDown Setpoint
      DPAD_UP, // L4 Setpoint
      DPAD_RIGHT, // L3 Setpoint
      DPAD_LEFT, // L2 Setpoint
      XboxController.Button.kBack.value, // AutoAlign Left AND L4 Setpoint
      XboxController.Button.kStart.value // AutoAlign Right AND L4 Setpoint
    };

    public static final int[] ARVINDH = {
      XboxController.Button.kY.value, // Arm Up Manual
      XboxController.Button.kA.value, // Arm Down Manual
      XboxController.Button.kLeftBumper.value, // Release Coral AND Intake (INVERSE)
      XboxController.Button.kRightBumper.value, // Release Coral AND Intake
      XboxController.Button.kB.value, // Climber (Unclimb)
      XboxController.Button.kX.value, // Climber (Climb)
      DPAD_DOWN, // ArmDown Setpoint
      DPAD_RIGHT, // L4 Setpoint
      DPAD_UP, // L3 Setpoint
      DPAD_LEFT, // L2 Setpoint
      XboxController.Button.kBack.value, // AutoAlign Left AND L4 Setpoint
      XboxController.Button.kStart.value // AutoAlign Right AND L4 Setpoint
    };
  }
}