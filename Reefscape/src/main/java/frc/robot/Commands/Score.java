package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.Command;

public class Score extends Command {
    double power;
    VictorSPX coralIntake;
    long startTime;

    public Score(double power, VictorSPX coralIntake) {
        this.coralIntake = coralIntake;
        this.power = power;
    }

    public void initialize() {
        coralIntake.set(ControlMode.PercentOutput, -power);
        startTime = System.currentTimeMillis();
    }

    public void end(boolean interrupted) {
        coralIntake.set(ControlMode.PercentOutput, 0);
    }

    public boolean isFinished() {
        if (Math.round((System.currentTimeMillis() - startTime) / 1000) > 0.5) {
            return true;
        } else {
        return false;
        }
    }
}
