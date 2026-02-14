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
        
    public static final class TestingConstants 
  {
    public static final int testMotorID = 6;
  } 

    public static final class ElevatorConstants 
  {
    public static final int kElevatorLeftMotorID = 99;        //Pratice Robot ID: 3
    //public static final int kElevatorRightMotorID = 10;      //Pratice Robot ID: 44
    // public static final double kStartPosition = 0; 
    // public static final double kFirstPosition = 4.6; //originally 0
    // public static final double kSecondPosition = 9.2;             
    // public static final double kThirdPosition = 26.5;
    // public static final double kFourthPosition = 54; //originally 54
    //public static final double kFourthPositionAuto = 52;
  }

  public static final class ShooterConstants {
    public static final int ShooterMotorID = 10;
    public static final int ShooterMotorID2 = 2;
    
    public static final double shooterSpeed = 0.55;
  }

  public static final class IntakeConstants{
    public static final int IntakeMotorID = 8;
    public static final double intakeSpeed = 1.0;

    public static final int HopperMotorID = 11;
    public static final double hopperSpeed = 0.5;

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
        public static double reefProportionalTx = 28;
        public static double algaeProportionalTx = 7.5;
    }


  }


