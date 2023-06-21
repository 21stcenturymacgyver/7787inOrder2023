package frc.robot;

public final class Constants {

  public static final class OperatorConstants {
    public static final int K_DRIVER_CONTROLLER_PORT = 0;
  }

  // Constants relating to the Arm Subsystem
  public static final class ArmConstants {

    // Arm angle mininum values in degrees
    public static final int THETA1_MAX = 135;
    public static final int THETA2_MAX = 180;
    public static final int THETA1_MIN = 30;
    public static final int THETA2_MIN = 30;

    // Arm Lengths
    public static final int ARM_LENGTH1 = 40; // Arm Length is in inches
    public static final int ARM_LENGTH2 = ARM_LENGTH1; 

    // Motor ID's
    public static final int AZIMUTH_MOTOR_ID = 2;
    public static final int SHOULDER_MOTOR_ID = 3;
    public static final int ELBOW_MOTOR_ID = 4;
    public static final int CLAW_MOTOR_ID = 5;

  }

  // Constnats relating to the Drive Subsystem
  public static final class DriveConstants {
    
    // Motor Ports
    public static final int MOTOR_LEFT1_PORT = 0;
    public static final int MOTOR_LEFT2_PORT = 1;
    public static final int MOTOR_RIGHT1_PORT = 2;
    public static final int MOTOR_RIGHT2_PORT = 3;

    // Encoder Ports
    public static final int[] ENCODER_PORTS_LEFT = {0, 1};
    public static final int[] ENCODER_PORTS_RIGHT = {2, 3};

    // Encoder Directions
    public static final boolean ENCODER_REVERSED_LEFT = false;
    public static final boolean ENCODER_REVERSED_RIGHT= true;

    // Adaptive Steering Sensitivity
    public static final double ADAPTIVE_STEERING_SENSITIVITY = 0.002;

    // Encoder Ticks Per Revolution
    public static final int ENCODER_TICKS_PER_REV= 1024;
    
    // Encoder Distance
    public static final double ENCODER_DISTANCE_PER_PULSE_INCHES =3.0;
        
  }
  public static final class ButtonMappings {

    // Button Mappings
    public static final int SQUARE = 0;
    public static final int CROSS = 1;
    public static final int CIRCLE = 2;
    public static final int TRIANGLE = 3;

    // Trigger Mappings
    public static final int L1 = 4;
    public static final int R1 = 5;
    public static final int L2 = 6;
    public static final int R2 = 7;

    // Function Button Mappings
    public static final int SHARE = 8;
    public static final int START = 9;

    // Joystick Mappings
    public static final int L3 = 10;
    public static final int R3 = 11;

    // Playstation Button Maping
    public static final int PS = 12;

    // Touch Pad Mappings
    public static final int TOUCH_PAD = 13;
  }

  public static final int DRIVER_CONTROLLER = 0;

}