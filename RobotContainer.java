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
  private final ArmSubsystem ArmSubsystem = new ArmSubsystem();
  private final NavXSubsystem navXSubsystem = new NavXSubsystem();
  private final Auto m_autoCommand = new Auto(DriveSubsystem, ArmSubsystem);
  
  // private final SparkmaxMotor testSparkmaxMotor = new SparkmaxMotor(5); // **** Feb 24
  
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
  public ArmSubsystem getArmSubsystem(){
    return this.ArmSubsystem;
  }
  public DriveSubsystem getDriveSubsystem(){
    return this.DriveSubsystem;
  }
  public NavXSubsystem getNavXSubsystem() {
    return this.navXSubsystem;
  }

  // 
  // public SparkmaxMotor getMotorTest(){ 
  //   return this.testSparkmaxMotor;
  // }

  private void configureBindings() {
  
    // PS4 bindings
    // new JoystickButton(this.ps4, ButtonMappings.R1).whileTrue((this.LimelightSubsystem.setPipeline9Command()));//changed to whileTrue
    // new JoystickButton(this.ps4, ButtonMappings.R1).whileFalse(new StopMotor(this.DriveSubsystem));//changed to whileFalse
    new JoystickButton(this.ps4, ButtonMappings.L1).onTrue(this.DriveSubsystem.shiftHigh());
    new JoystickButton(this.ps4, ButtonMappings.R1).onTrue(this.DriveSubsystem.shiftLow());
    //new JoystickButton(this.ps4, ButtonMappings.SQUARE).onTrue(this.DriveSubsystem.compressorOn());
    //new JoystickButton(this.ps4, ButtonMappings.CROSS).onTrue(this.DriveSubsystem.compressorOff());
    //new JoystickButton(this.ps4, ButtonMappings.L1).whileTrue(this.DriveSubsystem.driveToPositionCommand(2.0));
    // new JoystickButton(this.ps4, ButtonMappings.L2).onTrue(this.DriveSubsystem.spinToPosition(360.0));
    new JoystickButton(this.ps4, ButtonMappings.START).onTrue(this.ArmSubsystem.RunArmToPositionCommand(90, 15,0));//home test limit
    new JoystickButton(this.ps4, ButtonMappings.CIRCLE).onTrue(this.ArmSubsystem.RunArmToPositionCommand(90, 79, 90));//medium left
    new JoystickButton(this.ps4, ButtonMappings.SQUARE).onTrue(this.ArmSubsystem.RunArmToPositionCommand(90, 79, 0));  // Medium
    new JoystickButton(this.ps4, ButtonMappings.CROSS).onTrue(this.ArmSubsystem.RunArmToPositionCommand(66, 123, 90)); //High
    new JoystickButton(this.ps4, ButtonMappings.TRIANGLE).onTrue(this.ArmSubsystem.RunArmToPositionCommand(112, 63));//substation
    // Circle, claw toggle
    //  Second Controller  Fine Controll :(

    

    // Limelight Subsytem default commands
    this.LimelightSubsystem.setDefaultCommand(
      this.LimelightSubsystem.checkForTargetsCommand()
    );

    // Drive Subsytem default commands
    this.DriveSubsystem.setDefaultCommand(
          this.DriveSubsystem.arcadeDriveSquaredCommand(
              () -> -this.ps4.getLeftY(), () -> -this.ps4.getRightX()) // Taiga Drive **** Feb 28 to test encoders for drive motors


    );

    // **** Feb 24
    // this.ArmSubsystem.setDefaultCommand(
    //      this.ArmSubsystem.joystickMotorCommand(
    //          () -> (this.ps4.getRightX()*-30),() -> (this.ps4.getR2Axis()*30+30),() -> (this.ps4.getRightY()*-80),() -> (this.ps4.getL2Axis()*-20))
    // );
    this.ArmSubsystem.setDefaultCommand(
         this.ArmSubsystem.joystickMotorCommand(
             () -> (0),() -> (0),() -> (0),() -> (this.ps4.getR2Axis()*30+30))
    );

    // New command to follow targets while true.
    //new Trigger(this.LimelightSubsystem::hasTargets).whileTrue( new FollowTargets(this.DriveSubsystem,this.LimelightSubsystem));

    
  }
  public Auto getAutonomousCommand() {
    return m_autoCommand;
  }
}
