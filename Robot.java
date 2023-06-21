package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavXSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private static LimelightSubsystem m_LimelightSubsystem;
  private NavXSubsystem m_NavXSubsystem;
  private DriveSubsystem m_DriveSubsystem; // **** feb 24
  

  
  @Override
  public void robotInit() {

    DataLogManager.start();

    m_robotContainer = new RobotContainer();
    m_LimelightSubsystem = m_robotContainer.getLimelightSubsystem();
    m_DriveSubsystem =m_robotContainer.getDriveSubsystem();
    //m_NavXSubsystem = m_robotContainer.getNavXSubsystem();
    //m_MotorTest = m_robotContainer.getMotorTest(); // **** feb 24
    //m_NavXSubsystem.ahrsInit();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
    //m_DriveSubsystem.arcadeDriveAdaptiveSteering(1.0, 0.0);
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();



    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }


  @Override
  public void autonomousPeriodic() {
   
  }
} 