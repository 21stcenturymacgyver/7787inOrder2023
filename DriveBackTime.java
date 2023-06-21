package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveBackTime extends CommandBase{
    private boolean isComplete = false;
    private DriveSubsystem subsystem;
    private double pwr;
    private double time;

    public DriveBackTime(DriveSubsystem m_subsystem, double m_pwr, double m_time) {

        subsystem = m_subsystem;
        addRequirements(subsystem);
        pwr = m_pwr;
        time = m_time;
      }
    
      // Called when the command is initialized
      @Override
      public void initialize() {
      }
      
      // Called every time the scheduler runs
      @Override
      public void execute() {
              subsystem.arcadeDriveSquared(pwr, 0.0);
              Timer.delay(4);
              isComplete = true;
          
  
        // Command to run the motor
        SmartDashboard.putBoolean("isComplete", isComplete);
  
        
      }
    
      // Called once the command ends or is interrupted.
      @Override
      public void end(boolean interrupted) {
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return isComplete;
      }
    
}
