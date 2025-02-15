package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.Command;

public class Intakecoral extends Command {
    double power;
    VictorSPX coralIntake;

    public Intakecoral(double power, VictorSPX coralIntake) {
        this.coralIntake = coralIntake;
        this.power = power;
    }

    public void initialize() {
        coralIntake.set(ControlMode.PercentOutput, -power);
    }

    public void end(boolean interrupted) {
        coralIntake.set(ControlMode.PercentOutput, 0);
    }

    public boolean isFinished() {
        return false;
    }
}
