package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

import frc.robot.Constants.ButtonMappings;
import frc.robot.commands.FollowTargets;
import frc.robot.commands.auto.DriveAndPark;
import frc.robot.commands.auto.ScoreCone;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.prefs.Preferences;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  
  // Defines the Arm, Drive, and Limelight subsystems.
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final LimelightSubsystem LimelightSubsystem = new LimelightSubsystem();
  private final ArmSubsystem ArmSubsystem = new ArmSubsystem();
  private final NavXSubsystem navXSubsystem = new NavXSubsystem();
  private final MotorAutoTest motorAutoTest = new MotorAutoTest();

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<Command>();

  //private final Auto m_autoCommand = new Auto(DriveSubsystem, ArmSubsystem);
  
  // private final SparkmaxMotor testSparkmaxMotor = new SparkmaxMotor(5); // **** Feb 24
  
  // Creates the PS4Controller
  private final PS4Controller ps4_taiga = new PS4Controller(Constants.DRIVER_CONTROLLER_TAIGA);

  private final PS4Controller ps4_bri = new PS4Controller(Constants.DRIVER_CONTROLLER_BRI);

  private void initializeAutoChooser() {
    m_autoChooser.setDefaultOption("Do nothing", new WaitCommand(0));
    m_autoChooser.addOption("Drive And Park", new DriveAndPark(motorAutoTest));
    m_autoChooser.addOption("Score Cone", new ScoreCone(driveSubsystem));

    SmartDashboard.putData("Auto Selector", m_autoChooser);
  }
  
  // Defines robot container
  public RobotContainer() {


    SmartDashboard.putData("Scheduler", CommandScheduler.getInstance());
    initializeAutoChooser();

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
    return this.driveSubsystem;
  }
  public NavXSubsystem getNavXSubsystem() {
    return this.navXSubsystem;
  }


  private void configureBindings() {
  
    // PS4 bindings
    // new JoystickButton(this.ps4, ButtonMappings.R1).whileTrue((this.LimelightSubsystem.setPipeline9Command()));//changed to whileTrue
    // new JoystickButton(this.ps4, ButtonMappings.R1).whileFalse(new StopMotor(this.DriveSubsystem));//changed to whileFalse
    new JoystickButton(this.ps4_taiga, ButtonMappings.L1).onTrue(this.driveSubsystem.shiftHigh());
    new JoystickButton(this.ps4_taiga, ButtonMappings.R1).onTrue(this.driveSubsystem.shiftLow());
    //new JoystickButton(this.ps4, ButtonMappings.SQUARE).onTrue(this.DriveSubsystem.compressorOn());
    //new JoystickButton(this.ps4, ButtonMappings.CROSS).onTrue(this.DriveSubsystem.compressorOff());
    //new JoystickButton(this.ps4, ButtonMappings.L1).whileTrue(this.DriveSubsystem.driveToPositionCommand(2.0));
    // new JoystickButton(this.ps4, ButtonMappings.L2).onTrue(this.DriveSubsystem.spinToPosition(360.0));
    new JoystickButton(this.ps4_taiga, ButtonMappings.START).onTrue(this.ArmSubsystem.setArmPositionCommand(120, 11,0));// start home test limit
    new JoystickButton(this.ps4_taiga, ButtonMappings.CIRCLE).onTrue(this.ArmSubsystem.setArmPositionCommand(75, 30));// circle ground
    new JoystickButton(this.ps4_taiga, ButtonMappings.SQUARE).onTrue(this.ArmSubsystem.setArmPositionCommand(90, 79));  //  square Medium
    new JoystickButton(this.ps4_taiga, ButtonMappings.CROSS).onTrue(this.ArmSubsystem.setArmPositionCommand(66, 123)); // x High
    new JoystickButton(this.ps4_taiga, ButtonMappings.TRIANGLE).onTrue(this.ArmSubsystem.setArmPositionCommand(112, 68));//112,63 match triangle substation

    new JoystickButton(this.ps4_bri, ButtonMappings.START).onTrue(this.ArmSubsystem.setArmPositionCommand(120,11,0));// start same home position
    new JoystickButton(this.ps4_bri, ButtonMappings.CIRCLE).onTrue(this.ArmSubsystem.setArmAzimuthCommand(-90));// circle right -90
    new JoystickButton(this.ps4_bri, ButtonMappings.SQUARE).onTrue(this.ArmSubsystem.setArmAzimuthCommand(90));  //  square left 90
   // new JoystickButton(this.ps4_bri, ButtonMappings.CROSS).onTrue(this.ArmSubsystem.setArmAzimuthCommand(???)); // x unassigned
    new JoystickButton(this.ps4_bri, ButtonMappings.TRIANGLE).onTrue(this.ArmSubsystem.setArmAzimuthCommand(0));// triangle forward
    new JoystickButton(this.ps4_bri, ButtonMappings.R1).onTrue(this.ArmSubsystem.setArmAzimuthR30Command());// triangle forward
    new JoystickButton(this.ps4_bri, ButtonMappings.L1).onTrue(this.ArmSubsystem.setArmAzimuthL30Command());// triangle forward
    



    

    // Limelight Subsytem default commands
    this.LimelightSubsystem.setDefaultCommand(
      this.LimelightSubsystem.checkForTargetsCommand()
    );

    // Drive Subsytem default commands
    this.driveSubsystem.setDefaultCommand(
          this.driveSubsystem.arcadeDriveSquaredCommand(
            //() -> (0),() -> (0))//**** March 1st disable drive
              () -> -this.ps4_taiga.getLeftY(), () -> -this.ps4_taiga.getRightX()) // Taiga Drive **** Feb 28 to test encoders for drive motors


    );

    // **** Feb 24
    // this.ArmSubsystem.setDefaultCommand(
    //      this.ArmSubsystem.joystickMotorCommand(
    //          () -> (this.ps4.getRightX()*-30),() -> (this.ps4.getR2Axis()*30+30),() -> (this.ps4.getRightY()*-80),() -> (this.ps4.getL2Axis()*-20))
    // );
    this.ArmSubsystem.setDefaultCommand(
         this.ArmSubsystem.analogArmInputsCommand(//azimuth shoulder elbow claw
              //() -> (0),() -> (0),() -> (0),() -> (this.ps4_taiga.getR2Axis()*30+30))//1 **** March 1st
             //() -> (this.ps4_bri.getRightX()*-30),() -> ((this.ps4_bri.getLeftY()*-30)+((this.ps4_bri.getL2Axis()+1)*10)),() -> (this.ps4_bri.getRightY()*-30+((this.ps4_bri.getL2Axis()+1)*20)),() -> ((this.ps4_bri.getR2Axis()+1)*22))// for fine controll add to other controller
             () -> (this.ps4_bri.getRightX()*-10),() -> ((this.ps4_bri.getLeftY()*-10)+((this.ps4_bri.getL2Axis()+1)*5)),() -> (this.ps4_bri.getRightY()*-15+((this.ps4_bri.getL2Axis()+1)*10)),() -> ((this.ps4_bri.getR2Axis()+1)*22))// for fine controll add to other controller
    );

    // New command to follow targets while true.
    //new Trigger(this.LimelightSubsystem::hasTargets).whileTrue( new FollowTargets(this.DriveSubsystem,this.LimelightSubsystem));

    
  }
  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }
}
