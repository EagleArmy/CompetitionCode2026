// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class ShooterSubsystem extends SubsystemBase {

    private static final TalonFX ShooterMotor = new TalonFX(ShooterConstants.ShooterMotorID);
    private static final TalonFX ShooterMotor2 = new TalonFX(ShooterConstants.ShooterMotorID2);
    public static boolean shooterON = false;
    //shootermotor2 is canID 2 how convenient

        public static double shooterSpeed = ShooterConstants.shooterConstantSpeed;
        private final DutyCycleOut m_CycleOut = new DutyCycleOut(0);
        private final VoltageOut m_sysIdControl = new VoltageOut(0);

        private final SysIdRoutine m_sysIdRoutine =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,         // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic voltage to 4 to prevent brownout
                    null,          // Use default timeout (10 s)
                                        // Log state with Phoenix SignalLogger class
                    state -> SignalLogger.writeString("state", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    volts -> ShooterMotor2.setControl(m_sysIdControl.withOutput(volts)),
                    null,
                    this
                )
            );
           
            public ShooterSubsystem() 
            { 
                ShooterMotor.setControl(new Follower(ShooterMotor2.getDeviceID(), MotorAlignmentValue.Opposed));
               // shooterSpeed = ShooterConstants.shooterConstantSpeed;
        
               addChild("ShooterMotor2", ShooterMotor2);
        
               var ShooterMotor2Configuration = new TalonFXConfiguration();
               ShooterMotor2Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
               ShooterMotor2Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
               ShooterMotor2.getConfigurator().apply( ShooterMotor2Configuration );

            //    ShooterMotor2.setControl(new Follower(ShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
               
        
                       var ShooterMotorConfiguration = new TalonFXConfiguration();
               ShooterMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
               ShooterMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
               ShooterMotor.getConfigurator().apply( ShooterMotorConfiguration );
        
            var slot0ConfigsRight = ShooterMotor2Configuration.Slot0;
            slot0ConfigsRight.kS = 0.36568; // Add 0.25 V output to overcome static friction
            slot0ConfigsRight.kV = 0.12303; // A velocity target of 1 rps results in 0.12 V output
            slot0ConfigsRight.kA = 0.020527; // An acceleration of 1 rps/s requires 0.01 V output
            slot0ConfigsRight.kP = 0.21348; // A position error of 2.5 rotations results in 12 V output   //4.8 originally 
            slot0ConfigsRight.kI = 0; // no output for integrated error
            slot0ConfigsRight.kD = 0; // A velocity error of 1 rps results in 0.1 V output
        
            var slot0Configs = ShooterMotorConfiguration.Slot0;
            slot0Configs.kS = 0.36568; // Add 0.25 V output to overcome static friction
            slot0Configs.kV = 0.12303; // A velocity target of 1 rps results in 0.12 V output
            slot0Configs.kA = 0.020527; // An acceleration of 1 rps/s requires 0.01 V output
            slot0Configs.kP = 0.21348; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
            slot0Configs.kI = 0; // no output for integrated error
            slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output
        
        
            // // set Motion Magic settings
            var motionMagicConfigsRight = ShooterMotor2Configuration.MotionMagic;
            motionMagicConfigsRight.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
            motionMagicConfigsRight.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
            motionMagicConfigsRight.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
        
              // // set Motion Magic settings
            var motionMagicConfigsLeft = ShooterMotorConfiguration.MotionMagic;
            motionMagicConfigsLeft.MotionMagicCruiseVelocity = 20; // Target cruise velocity of 80 rps
            motionMagicConfigsLeft.MotionMagicAcceleration = 40; // Target acceleration of 160 rps/s (0.5 seconds)
            motionMagicConfigsLeft.MotionMagicJerk = 400; // Target jerk of 1600 rps/s/s (0.1 seconds)
        
            // 20, 40, 400
            // 8000, 16000, 160000
            ShooterMotor2.getConfigurator().apply(ShooterMotor2Configuration);
            ShooterMotor.getConfigurator().apply(ShooterMotorConfiguration);
        
            }
        
            public void start()
            {
                ShooterMotor.set(-shooterSpeed); 
                ShooterMotor2.set(shooterSpeed); //1 needs to be reversed
                System.out.println("shooterSpeed: " + shooterSpeed);
                //ADD A PRINT
            }
        
            // public void set(double speed, double speed2){
            //     ShooterMotor.set(-speed);
            //     ShooterMotor2.set(speed);
            // }

        
            public void reverse() 
            {
                ShooterMotor.set(shooterSpeed);
                ShooterMotor2.set(-shooterSpeed);
            }
        
            public void stop()
            {
                ShooterMotor.set(0);
                ShooterMotor2.set( 0 );
            }

            public void increaseshooterSpeed() 
            {
                shooterSpeed += 0.02;
                System.out.println("shooterSpeed: " + shooterSpeed);
            }

            public void decreaseshooterSpeed() 
            {
                shooterSpeed -= 0.02;
                System.out.println("shooterSpeed: " + shooterSpeed);
            }
        
            public void setShooterSpeed(double newSpeed) {
                shooterSpeed = newSpeed;
                System.out.println("NEW SPEED" + shooterSpeed);
            }

            public static void setShooterSpeedManually(double newSpeed)
            {
                ShooterMotor.set(newSpeed);
                ShooterMotor2.set(-newSpeed);
            }
        
            //shooting from different areas
            // public static void shooterSpeedHub() {
            //     shooterSpeed = 0.55;
            // }

            // public static void shooterSpeedMid() {
            //         shooterSpeed = 0.65;
            //     }
                
            // public static void shooterSpeedTower() {
            //     shooterSpeed = 0.75;
            // }

            public Command joystickDriveCommand(DoubleSupplier output) {
        return run(() -> ShooterMotor2.setControl(m_CycleOut.withOutput(output.getAsDouble())));
    }
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
    

    @Override
    public void periodic() 
    {
      //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
      //log();
    }

    //Katelynn is bad at coding...so is Mark Ruder. 
    //                            - Michael Murphy    
    
    //okay Mark
}