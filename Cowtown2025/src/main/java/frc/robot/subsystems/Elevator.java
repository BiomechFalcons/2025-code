package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.

  private final TrapezoidProfile.Constraints m_constraints =
      new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration);
  private final ProfiledPIDController m_controller =
      new ProfiledPIDController(
        ElevatorConstants.kP, 
        ElevatorConstants.kI, 
        ElevatorConstants.kD, 
        m_constraints, 
        ElevatorConstants.kDt);

  //private final Encoder m_encoder = new Encoder(2, 3);
  private final SparkMax m_motor = new SparkMax(9, MotorType.kBrushless);
  private final SparkMax m_motor2 = new SparkMax(10, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
    ElevatorConstants.kS, 
    ElevatorConstants.kG, 
    ElevatorConstants.kV);



    public Elevator()
    {
      SparkMaxConfig motorconfig = new SparkMaxConfig();
      motorconfig.inverted(true);
      m_motor.configure(motorconfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
      m_motor2.configure(motorconfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
        //Elevator PID
       // m_encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 0.5);
      //  m_encoder.setDistancePerPulse(1.0 / 2048.0 * Math.PI * 1.625);
      //  m_encoder.reset();
      initializeElevator();
        
    }

    public void initializeElevator()
    {
      System.out.println("Reset to zero");
      m_encoder.setPosition(0);
    
    }

    public double getEncoderDistance() {
      return m_encoder.getPosition();
    }

    public boolean isAtGoal() {
      System.out.println(getEncoderDistance());
      return m_controller.atGoal();
    }

    public void ElevatorChange(boolean aButtonPressed, boolean bButtonPressed, boolean xButtonPressed, boolean yButtonPressed)
    {
          SmartDashboard.putNumber("Encoder get distance:", getEncoderDistance());
          
              if (aButtonPressed) {
                // L1
                m_controller.setGoal(0.05);
                
              } 
              else if (xButtonPressed) {
                // L2
                m_controller.setGoal(1);
                 
                }
                else if (yButtonPressed)
                {
                  // L3
                  m_controller.setGoal(5);
                }
                else if (bButtonPressed)
                {
                  // L4
                  m_controller.setGoal(7);
                } 
    
              m_motor.setVoltage( 
                -(m_controller.calculate(getEncoderDistance())
                    + m_feedforward.calculate(m_controller.getSetpoint().velocity)));
          
              m_motor2.setVoltage( 
                  (m_controller.calculate(getEncoderDistance())
                      + m_feedforward.calculate(m_controller.getSetpoint().velocity)));
      
        
    }
  }