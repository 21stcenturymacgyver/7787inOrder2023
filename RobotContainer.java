package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.Constants.ButtonMappings;
import frc.robot.commands.FollowTargets;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller;

public class RobotContainer {
  
  // Defines the Arm, Drive, and Limelight subsystems.
  private final DriveSubsystem DriveSubsystem = new DriveSubsystem();
  private final LimelightSubsystem LimelightSubsystem = new LimelightSubsystem();
  //private final ArmSubsystem ArmSubsystem = new ArmSubsystem();****
  // **** feb 24
  private final SparkmaxMotor testSparkmaxMotor = new SparkmaxMotor(4); // **** Feb 24
  
  // Creates the PS4Controller
  private final PS4Controller ps4 = new PS4Controller(Constants.DRIVER_CONTROLLER);
  
  // Defines robot container
  public RobotContainer() {

    // Configure the trigger bindings
    configureBindings();
        
  }

  // Initilizes all of the subsystems
  public LimelightSubsystem getLimelightSubsystem(){
    return this.LimelightSubsystem;
  }
  // public ArmSubsystem getArmSubsystem(){****
  //   return this.ArmSubsystem;
  // }
  public DriveSubsystem getDriveSubsystem(){
    return this.DriveSubsystem;
  }

  // **** feb 24
  public SparkmaxMotor getMotorTest(){ 
    return this.testSparkmaxMotor;
  }

  private void configureBindings() {
  
    // PS4 bindings
    new JoystickButton(this.ps4, ButtonMappings.R1).whileTrue((this.LimelightSubsystem.setPipeline9Command()));//changed to whileTrue
    new JoystickButton(this.ps4, ButtonMappings.R1).whileFalse(new StopMotor(this.DriveSubsystem));//changed to whileFalse

    // Limelight Subsytem default commands
    this.LimelightSubsystem.setDefaultCommand(
      this.LimelightSubsystem.checkForTargetsCommand()
    );

    // Drive Subsytem default commands
    this.DriveSubsystem.setDefaultCommand(
          this.DriveSubsystem.arcadeDriveSquaredCommand(
              () -> -this.ps4.getLeftY(), () -> -this.ps4.getLeftX())
    );

    // **** Feb 24
    this.testSparkmaxMotor.setDefaultCommand(
         this.testSparkmaxMotor.joystickMotorCommand(
             () -> (this.ps4.getRightY()*20))
    );

    // New command to follow targets while true.
    new Trigger(this.LimelightSubsystem::hasTargets).whileTrue( new FollowTargets(this.DriveSubsystem,this.LimelightSubsystem));

  }
}
