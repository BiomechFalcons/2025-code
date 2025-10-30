package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    // Create a PID controller whose setpoint's change is subject to maximum
  // velocity and acceleration constraints.

  // private final TrapezoidProfile.Constraints m_constraints =
  //     new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration);
  private final PIDController m_controller =
      new PIDController(
        ElevatorConstants.kP, 
        ElevatorConstants.kI, 
        ElevatorConstants.kD);

  //private final Encoder m_encoder = new Encoder(2, 3);
  private final SparkMax m_motor = new SparkMax(9, MotorType.kBrushless);
  private final SparkMax m_motor2 = new SparkMax(10, MotorType.kBrushless);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();
  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
    ElevatorConstants.kS, 
    ElevatorConstants.kG, 
    ElevatorConstants.kV);
  private DigitalInput m_sensor = new DigitalInput(9);


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
      m_controller.setTolerance(0.25); //Added
      initializeElevator();
        
    }

    public void initializeElevator()
    {
      System.out.println("Reset to zero");
      m_encoder.setPosition(0);
    
    }

    public double getEncoderDistance() {
      return -1*m_encoder.getPosition();
    }

    public boolean isCoralDetected() {
      return m_sensor.get();
    }

    public boolean isAtGoal() {
      double elevatorSetting = m_controller.calculate(getEncoderDistance());
      SmartDashboard.putNumber("Encoder get distance:", getEncoderDistance());
      SmartDashboard.putNumber("profiled elevator output", elevatorSetting);
      SmartDashboard.putNumber("error", m_controller.getPositionError());
      SmartDashboard.putNumber("tolerance", m_controller.getPositionTolerance());



      System.out.println(getEncoderDistance() + "Goal Reached = " + m_controller.atSetpoint() + " motor setting = " + m_motor.get());
      if (m_controller.atSetpoint()) {
        System.out.println("TRUETRUE");
        m_motor.setVoltage(-ElevatorConstants.kG);
        m_motor2.setVoltage(ElevatorConstants.kG);
      } else {
        System.out.println(elevatorSetting);
        m_motor.setVoltage( 
          -(elevatorSetting+ElevatorConstants.kG));
            //  + m_feedforward.calculate(m_controller.getSetpoint().velocity));
    
        m_motor2.setVoltage( 
            (elevatorSetting+ElevatorConstants.kG));
                        //  + m_feedforward.calculate(m_controller.getSetpoint().velocity));

      }

      return m_controller.atSetpoint();
    }

   


    public void ElevatorChange(boolean aButtonPressed, boolean bButtonPressed, boolean xButtonPressed, boolean yButtonPressed)
    {
          
      
          
    if (aButtonPressed) {
      // L1
      m_controller.setSetpoint(0);
      
    } 
    else if (xButtonPressed) {
      // L2
      m_controller.setSetpoint(15.7);
        
      }
    else if (yButtonPressed)
      {
        // L3
        m_controller.setSetpoint(34.5);
      }
    else if (bButtonPressed)
      {
        // L4
        m_controller.setSetpoint(67.7);
      } 

      

        
    }
  }