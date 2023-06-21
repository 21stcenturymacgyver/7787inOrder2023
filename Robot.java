package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimelightSubsystem;
//import frc.robot.subsystems.SparkmaxMotor;


public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private static LimelightSubsystem m_LimelightSubsystem;
  //private static SparkmaxMotor m_MotorTest; // **** feb 24

  
  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();
    m_LimelightSubsystem = m_robotContainer.getLimelightSubsystem();
    //m_MotorTest = m_robotContainer.getMotorTest(); // **** feb 24

  }
  @Override
  public void teleopInit() {

    m_LimelightSubsystem.setPipeline(9);
    //m_MotorTest.startTimer();

  }
  @Override
  public void teleopPeriodic() {
    
    CommandScheduler.getInstance().run();
    
  }

  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

}