// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
  // SparkMax m_climberMotor = new SparkMax(9, MotorType.kBrushless);

  public ClimbSubsystem() {
  }

  public void setClimbPower(double pow) {
    // m_climberMotor.set(pow);
  } 

  @Override
  public void periodic() {
    
  }
}
