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
import frc.robot.Constants.NeckWheelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NeckWheelSubsystem extends SubsystemBase {

    private final TalonFX NeckWheelMotor = new TalonFX(NeckWheelConstants.NeckWheelMotorID);
    //public final double NeckWheelspeed = NeckWheelConstants.NeckWheelSpeed;
    public double NeckWheelspeed = NeckWheelConstants.NeckWheelSpeed;

    public NeckWheelSubsystem() 
    {
       addChild("NeckWheelMotor", NeckWheelMotor);

       var NeckWheelMotorConfiguration = new TalonFXConfiguration();
       NeckWheelMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       NeckWheelMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       NeckWheelMotor.getConfigurator().apply( NeckWheelMotorConfiguration );

      var slot0ConfigsLeft = NeckWheelMotorConfiguration.Slot0;
    slot0ConfigsLeft.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsLeft.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsLeft.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsLeft.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsLeft.kI = 0; // no output for integrated error
    slot0ConfigsLeft.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    //  set Motion Magic settings
    var motionMagicConfigsLeft = NeckWheelMotorConfiguration.MotionMagic;
    motionMagicConfigsLeft.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsLeft.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsLeft.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
    // 20, 40, 400
    // 8000, 16000, 160000

    NeckWheelMotor.getConfigurator().apply(NeckWheelMotorConfiguration);

    }

    public void start()
    {
        NeckWheelMotor.set(NeckWheelspeed);
        System.out.println("NeckWheel Speed: " + NeckWheelspeed);
    }

    public void reverse() 
    {
        NeckWheelMotor.set(-0.1);
    }

    public void stop()
    {
        NeckWheelMotor.set( 0 );
    }

    public void setNeckWheelSpeed (double newSpeed){
        NeckWheelspeed = newSpeed;
    }
    public void increasetestingspeed() 
    {
        NeckWheelspeed += 0.05;
        System.out.println("Testing Speed: " + NeckWheelspeed);
    }
    public void decreasetestingspeed() 
    {
        NeckWheelspeed -= 0.05;
        System.out.println("Testing Speed: " + NeckWheelspeed);
    }

    @Override
    public void periodic() 
    {
      //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
      //log();
    }

}
