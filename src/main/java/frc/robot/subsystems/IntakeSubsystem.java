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

public class IntakeSubsystem extends SubsystemBase
{
    public final TalonFX IntakeMotor = new TalonFX(IntakeConstants.IntakeMotorID);
    //private final TalonFX intakeMotor = new TalonFX(intakeConstants.testMotorID);
    public double intakespeed;

    
    public IntakeSubsystem() {

    intakespeed = 1.0;
    // Let's name everything on the LiveWindow
    addChild("IntakeMotor", IntakeMotor);


    var intakeMotorConfiguration = new TalonFXConfiguration();
    intakeMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    intakeMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    IntakeMotor.getConfigurator().apply( intakeMotorConfiguration );
        
    // // set slot 0 gains
    var slot0ConfigsLeft = intakeMotorConfiguration.Slot0;
    slot0ConfigsLeft.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsLeft.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsLeft.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsLeft.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsLeft.kI = 0; // no output for integrated error
    slot0ConfigsLeft.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // // set Motion Magic settings
    var motionMagicConfigsLeft = intakeMotorConfiguration.MotionMagic;
    motionMagicConfigsLeft.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsLeft.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsLeft.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
    // 20, 40, 400
    // 8000, 16000, 160000

    // // set Motion Magic settings
    var motionMagicConfigsRight = intakeMotorConfiguration.MotionMagic;
    motionMagicConfigsRight.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
    motionMagicConfigsRight.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsRight.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)

    IntakeMotor.getConfigurator().apply(intakeMotorConfiguration);
   // CoralRightMotor.getConfigurator().apply(CoralRightMotorConfiguration);
    }

    public Angle getLeftEncoderPosition() {
        var rotorPosSignal = IntakeMotor.getRotorPosition();
        var rotorPos = rotorPosSignal.getValue();
        return rotorPos;
      }
        public void start()
        {
            IntakeMotor.set(intakespeed);
            System.out.println("intake Speed: " + intakespeed);

        }

        public void reverse() {
            IntakeMotor.set(-intakespeed);
        }

        public void stop() {
            IntakeMotor.set( 0 );
           // CoralRightMotor.set( 0 );
            //extramotor.set(0);
          }
        
        public void increaseintakespeed() {
            intakespeed += 0.05;
            System.out.println("intake Speed: " + intakespeed);
          }
          public void decreaseintakespeed() {
            intakespeed -= 0.05;
            System.out.println("intake Speed: " + intakespeed);
        
          }
          public void log() {
            // SmartDashboard.putData("CoralMotor", CoralMotor);
            //System.out.println("Value: " + getEnterSensorRange());
          }
          
          /** Call log method every loop. */
          @Override
          public void periodic() {
            //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
            log();
          }
    }


