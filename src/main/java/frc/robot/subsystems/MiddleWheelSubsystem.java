package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.MiddleWheelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MiddleWheelSubsystem extends SubsystemBase {

    private final TalonFX MiddleWheelMotor = new TalonFX(MiddleWheelConstants.MiddleWheelMotorID);
    //public final double MiddleWheelspeed = MiddleWheelConstants.MiddleWheelSpeed;
    public double MiddleWheelspeed = .65;

    public MiddleWheelSubsystem() 
    {
       addChild("MiddleWheelMotor", MiddleWheelMotor);

       var MiddleWheelMotorConfiguration = new TalonFXConfiguration();
       MiddleWheelMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       MiddleWheelMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       MiddleWheelMotor.getConfigurator().apply( MiddleWheelMotorConfiguration );

      var slot0ConfigsLeft = MiddleWheelMotorConfiguration.Slot0;
    slot0ConfigsLeft.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsLeft.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsLeft.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsLeft.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsLeft.kI = 0; // no output for integrated error
    slot0ConfigsLeft.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    //  set Motion Magic settings
    var motionMagicConfigsLeft = MiddleWheelMotorConfiguration.MotionMagic;
    motionMagicConfigsLeft.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsLeft.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsLeft.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
    // 20, 40, 400
    // 8000, 16000, 160000

    MiddleWheelMotor.getConfigurator().apply(MiddleWheelMotorConfiguration);

    }

    public void start()
    {
        MiddleWheelMotor.set(MiddleWheelspeed);
        System.out.println("MiddleWheel Speed: " + MiddleWheelspeed);
    }

    public void reverse() 
    {
        MiddleWheelMotor.set(MiddleWheelspeed);
    }

    public void stop()
    {
        MiddleWheelMotor.set( 0 );
    }
    public void increasetestingspeed() 
    {
        MiddleWheelspeed += 0.05;
        System.out.println("Testing Speed: " + MiddleWheelspeed);
    }
    public void decreasetestingspeed() 
    {
        MiddleWheelspeed -= 0.05;
        System.out.println("Testing Speed: " + MiddleWheelspeed);
    }

    @Override
    public void periodic() 
    {
      //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
      //log();
    }

}
