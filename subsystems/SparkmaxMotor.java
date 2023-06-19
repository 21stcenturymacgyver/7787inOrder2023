// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// **** feb 24
import edu.wpi.first.wpilibj.Timer;

public class SparkmaxMotor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private CANSparkMax m_motor;
  //private static int deviceID;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // **** feb 24 
  private double position = 0;
  private Timer Timer1;


  public SparkmaxMotor(int deviceID) {
    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_pidController = m_motor.getPIDController();
    m_encoder = m_motor.getEncoder();
    
    // PID coefficients
    kP = 0.1; 
    kI = 0;//1e-4;
    kD = 0.01; //was 1 feb 24 test
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.2;//1; feb 24 test
    kMinOutput = -0.2;//-1; feb 24 test

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    //on creation of the motor
    // **** feb 24
    Timer1 = new Timer();//
    //Timer1.start();//

  }

  private void runToTimer(){
    position =  Timer1.get();//
   
    m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
  }
  public void startTimer(){
    Timer1.start();//
  }
  private void runToposition(double targetPosition){
   
    m_pidController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  // public CommandBase exampleMethodCommand() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         LimeLightSubsystem.setPipeline(7);
  //       });
  // }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //runToTimer();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


  //Commands
  public CommandBase joystickMotorCommand(DoubleSupplier Yval) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> this.runToposition(Yval.getAsDouble()))
        .withName("Joystick Motor");
  }
}
