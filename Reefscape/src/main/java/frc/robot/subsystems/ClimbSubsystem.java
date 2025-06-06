// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  public SparkMax m_climberMotor = new SparkMax(9, MotorType.kBrushless);
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("Climb");
  private AbsoluteEncoder m_encoder = m_climberMotor.getAbsoluteEncoder();
  private SparkBaseConfig config = new SparkMaxConfig().idleMode(IdleMode.kBrake);

  public ClimbSubsystem() {
    m_climberMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimbPower(double pow) {
      m_climberMotor.set(pow);
  } 

  @Override
  public void periodic() {
    table.putValue("Arm Angle", NetworkTableValue.makeDouble(m_climberMotor.getEncoder().getPosition()));
  }
} 
