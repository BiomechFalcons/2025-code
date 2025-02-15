package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;

public class test extends Command {
    ArmSubsystem m_ArmSubsystem;

    public test(ArmSubsystem armSubsystem) {
        this.m_ArmSubsystem = armSubsystem;
    }

    public void initialize() {
        System.out.println(m_ArmSubsystem.getArmPosition());
    }
    public void execute() {
        System.out.println(m_ArmSubsystem.getArmPosition());
    }
    public void end(boolean interrupted) {

    }

    public boolean isFinished() {
        return false;
    }
}
