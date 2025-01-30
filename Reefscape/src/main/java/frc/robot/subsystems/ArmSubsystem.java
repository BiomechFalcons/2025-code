package frc.robot.subsystems;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMot = new SparkMax(0, MotorType.kBrushless);
    private SparkMaxConfig m_pidcontroller;
    private SparkClosedLoopController m_closedloopcontroller;
    private double position;

    public ArmSubsystem() {
        // set sensor to a default pos

        m_pidcontroller.closedLoop.pid(0.1,0, 0);
        position = 0;
        m_closedloopcontroller.setReference(position, ControlType.kPosition);
    }

    public void setArmPosition(double pos) {
        position = pos;
        m_closedloopcontroller.setReference(position, ControlType.kPosition);
    }


    // public double getArmPosition() {
    //     return 0.0;
    // }
}