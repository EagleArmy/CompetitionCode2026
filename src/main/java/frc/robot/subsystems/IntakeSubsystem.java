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
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX IntakeMotor = new TalonFX(IntakeConstants.IntakeMotorID);
    private final TalonFX HopperMotor = new TalonFX(IntakeConstants.HopperMotorID);
    //public final double intakeSpeed = IntakeConstants.intakeSpeed;
    public double intakeSpeed = IntakeConstants.intakeSpeed;
    public double hopperSpeed = IntakeConstants.hopperSpeed;

    
    public IntakeSubsystem() 
    {
       addChild("IntakeMotor", IntakeMotor);

       var IntakeMotorConfiguration = new TalonFXConfiguration();
       IntakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       IntakeMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       IntakeMotor.getConfigurator().apply( IntakeMotorConfiguration );

       var HopperMotorConfiguration = new TalonFXConfiguration();
       HopperMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
       HopperMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
       IntakeMotor.getConfigurator().apply( HopperMotorConfiguration );

      var slot0ConfigsIntake = IntakeMotorConfiguration.Slot0;
    slot0ConfigsIntake.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsIntake.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsIntake.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsIntake.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsIntake.kI = 0; // no output for integrated error
    slot0ConfigsIntake.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    var slot0ConfigsHopperMotor = HopperMotorConfiguration.Slot0;
    slot0ConfigsHopperMotor.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsHopperMotor.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsHopperMotor.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsHopperMotor.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsHopperMotor.kI = 0; // no output for integrated error
    slot0ConfigsHopperMotor.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    //  set Motion Magic settings
    var motionMagicConfigsIntake = IntakeMotorConfiguration.MotionMagic;
    motionMagicConfigsIntake.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsIntake.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsIntake.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
    // 20, 40, 400
    // 8000, 16000, 160000

     //  set Motion Magic settings
    var motionMagicConfigsHopperMotor = HopperMotorConfiguration.MotionMagic;
    motionMagicConfigsHopperMotor.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsHopperMotor.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsHopperMotor.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
    // 20, 40, 400
    // 8000, 16000, 160000

    }

    public void start()
    {
        IntakeMotor.set(-intakeSpeed);
        HopperMotor.set(intakeSpeed);
        System.out.println("Intake Speed: " + intakeSpeed);
    }

    public void reverse() 
    {
        IntakeMotor.set(intakeSpeed);
        HopperMotor.set(-intakeSpeed);
    }

    public void stop()
    {
        IntakeMotor.set( 0 );
        HopperMotor.set(0);
    }
    public void increasetestingspeed() 
    {
        intakeSpeed += 0.05;
        System.out.println("Testing Speed: " + intakeSpeed);
    }
    public void decreasetestingspeed() 
    {
        intakeSpeed -= 0.05;
        System.out.println("Testing Speed: " + intakeSpeed);
    }

    public void setIntakeHopperSpeed(double a, double b)
    {
        intakeSpeed = a;
        hopperSpeed = b;
    }
    
    @Override
    public void periodic() 
    {
      //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
      //log();
    }

}
