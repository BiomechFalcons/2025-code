package frc.robot.Commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCorral extends Command {
    double power;
    PWMVictorSPX corralIntake;

    public IntakeCorral(double power, PWMVictorSPX corralIntake) {
        this.corralIntake = corralIntake;
        this.power = power;
    }

    public void initialize() {
        corralIntake.set(-power);
    }

    public boolean isFinished() {
        return false;
    }
}
