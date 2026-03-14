package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Constants {

       public class DriverProfile {
        /* Angular speed multiplier */
        public static final double kRotationMagnitude = 0.48;

        /* Slow mode speeds */
        public static final double x_SlowMultiplier = 0.65;
        public static final double y_SlowMultiplier = 0.65;
        public static final double rx_SlowMultiplier = 0.75;

        /* Driver station ports */
        public static final int driverPortNum = 0;
        public static final int operatorPortNum = 1;
        public static final int technicianPortNum = 2;

        /* Alignment Speed Multiplier */
        public static final double x_AlignmentMultiplier = 0.5;
        public static final double y_AlignmentMultiplier = 0.85; 

        /* Alignment Speeds */
        public static final double x_slowMode = 0.7;
        public static final double y_slowMode = 0.8;
        public static final double rx_slowMode = 0.4;
    }
        
  //   public static final class TestingConstants 
  // {
  //   public static final int testMotorID = 6;
  // } 

    public static final class ElevatorConstants 
  {
    public static final int ElevatorMotorID = 23;        //Pratice Robot ID: 3
    //public static final int kElevatorRightMotorID = 10;      //Pratice Robot ID: 44
    // public static final double kStartPosition = 0; 
    // public static final double kFirstPosition = 4.6; //originally 0
    // public static final double kSecondPosition = 9.2;             
    // public static final double kThirdPosition = 26.5;
    // public static final double kFourthPosition = 54; //originally 54
    //public static final double kFourthPositionAuto = 52;
  }

  public static class ShooterConstants {
    // public static final int ShooterMotorID = 23;
    public static final int ShooterMotorID = 50; //the motor on the right needs to move in reverse
    public static final int ShooterMotorID2 = 51;
    
    public static double shooterSpeed = 1;
    //OK BTW the motor moves counterclockwise if you're facing the "tape" or the talonFX part is facing away from you
  }

  public static class IntakeConstants{
    public static final int IntakeMotorID = 47;
    public static final int HopperMotorID = 48;
    public static double intakeSpeed = .8;
    public static double hopperSpeed = 0.3;
   public static final int IntakeSlideMotorID = 46;

    // public static final int HopperMotorID = 11;
    // I am michael murphy and I ruined the code
  }

   public static final class MiddleWheelConstants {
    public static final int MiddleWheelMotorID = 3;
    public static final double MiddleWheelSpeed = 0.05; //normally 65
  }


  public static class NeckWheelConstants {
    public static final int NeckWheelMotorID = 49;
    public static double NeckWheelSpeed = 0.40;
  }
  
      public class VisionProfile {
        /* Limelight names */
        public static String frontLimelight = "limelight-front";
        public static String elevatorLimelight = "limelight-rear";

        /* Calibrated frontLimelight Pipeline */
        public static int autoPipeline = 0;

        /* Calibrated elevator limelight pipelines */
        public static int reefPipeline_Test = 0;
        public static int blueReefCenterPipeline = 1;
        public static int blueReefLeftPipeline = 1;
        public static int blueReefRightPipeline = 3;
        public static int redReefCenterPipeline = 4;
        public static int redReefLeftPipeline = 4;
        public static int redReedRightPipeline = 5;

        /* Proportional limits for front limelight */
        public static double hubProportionalTx = 9;
        public static double algaeProportionalTx = 7.5;
    }

    public class LEDConstants{
          public static final int CANdleID = 1;
          //idk if we need the rest of this since it's basically just setting the LED lights to whatever button you press
    // public static final int JoystickId = 0;
    // public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
    // public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
    // public static final int BlockButton = XboxController.Button.kStart.value;
    // public static final int MaxBrightnessAngle = 90;
    // public static final int MidBrightnessAngle = 180;
    // public static final int ZeroBrightnessAngle = 270;
    // public static final int VbatButton = XboxController.Button.kA.value;
    // public static final int V5Button = XboxController.Button.kB.value;
    // public static final int CurrentButton = XboxController.Button.kX.value;
    // public static final int TemperatureButton = XboxController.Button.kY.value;
}
    }

  


