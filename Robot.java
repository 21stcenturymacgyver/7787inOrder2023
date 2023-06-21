package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.NavXSubsystem;


public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private NavXSubsystem m_navXSubsystem;
  //private final VictorSP motorOne = new VictorSP(0);

  
  @Override
  public void robotInit() {

    DataLogManager.start();

    m_robotContainer = new RobotContainer();
    m_navXSubsystem = m_robotContainer.getNavXSubsystem();
    
    m_navXSubsystem.ahrsInit();
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
    //m_MotorTest.startTimer();

  }
  @Override
  public void teleopPeriodic() {
    
    CommandScheduler.getInstance().run();
    
  }

  
  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();



    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }


  @Override
  public void autonomousPeriodic() {
  }
} 