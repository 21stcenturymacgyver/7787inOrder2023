package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RunMotor extends CommandBase {
    
    // Ignores the warnings that we are two lazy to fix.
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    // Creates an instance of the drive subsystem
    private DriveSubsystem m_subsystem;

    // Creates the dependencies for the command.
    public RunMotor(DriveSubsystem subsystem) {

      m_subsystem = subsystem;
      addRequirements(subsystem);
      m_subsystem.arcadeDriveSquared(1.0,0.0);
    }
  
    // Called when the command is initialized
    @Override
    public void initialize() {}
    
    // Called every time the scheduler runs
    @Override
    public void execute() {

      // Command to run the motor
      m_subsystem.arcadeDriveSquared(1.0,0.0);
      
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
  
