package frc.robot.subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.Constants.ArmConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private final SparkMax armMotor = new SparkMax(11, MotorType.kBrushless);
    private SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
    private SparkClosedLoopController m_closedloopcontroller = armMotor.getClosedLoopController();
    private double position;
    private RelativeEncoder m_encoder = armMotor.getEncoder();
    private ArmFeedforward m_ArmFeedforward;

    public ArmSubsystem() {
        sparkMaxConfig.closedLoop.pid(0.05,0, 0);
        m_encoder.setPosition(0);
        armMotor.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        position = 0;
        m_closedloopcontroller.setReference(position, ControlType.kPosition);
        m_ArmFeedforward = new ArmFeedforward(0, 0.03, 0, 0);
    }
    public void setArmPosition(double pos) {
        position = pos;
        m_closedloopcontroller.setReference(position, ControlType.kPosition);
    }
    public void setArmPower(double armPower) {
        double power = armPower;
        armMotor.set(power);
    }
    public void setArmFeedForward() {
        double pos = (ArmConstants.kArmInitPos / 180 * Math.PI) + getArmPositionInRadians();
   
        
        double feedForward = m_ArmFeedforward.calculate(pos, 0);
        armMotor.set(feedForward);
    }

    public double getArmPosition() {
        System.out.println(m_encoder.getPosition());
        return m_encoder.getPosition();
    }
    public double getArmPositionInRadians() {
        return m_encoder.getPosition() * 2 * Math.PI;
    }
}