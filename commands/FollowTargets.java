package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class FollowTargets extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private DriveSubsystem m_DriveSubsystem;
    private LimelightSubsystem m_LimelightSubsystem;
  
    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public FollowTargets(DriveSubsystem subsystem_drive, LimelightSubsystem subsystem_limelight) {
       m_DriveSubsystem = subsystem_drive;
       m_LimelightSubsystem= subsystem_limelight;
      // // Use addRequirements() here to declare subsystem dependencies.
       addRequirements(subsystem_drive);
      // 
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      
      // Use addRequirements() here to declare subsystem dependencies.
      double targetPosition = m_LimelightSubsystem.getTargets()[0];
      //double rot = Math.min(Math.max((targetPosition/5)*(Math.abs(targetPosition)/5),-0.8),0.6);//limit speed 
      double rot = Math.min(Math.max((targetPosition/Math.abs(targetPosition))*Math.sqrt(Math.abs(targetPosition/20)),-0.8),0.8);
      m_DriveSubsystem.arcadeDriveSquared(0.0,-rot);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }
  
