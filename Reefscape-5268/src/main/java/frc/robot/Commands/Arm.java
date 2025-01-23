package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.SparkMax;
public class Arm extends Command {
    SparkMax arm_motor;
    TalonSRX coral_motor;
    
    public void moveArm(double power) {
        arm_motor.set(power);
    }
    public void stopArm() {
        arm_motor.set(0);
    }
    public void grabCoral(double power) {
        coral_motor.set(ControlMode.PercentOutput, power);
    }
    public void holdCoral() {
        coral_motor.set(ControlMode.PercentOutput, 0);
    }
}