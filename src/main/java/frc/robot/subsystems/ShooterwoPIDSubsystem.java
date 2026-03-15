// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.



package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
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
public class ShooterwoPIDSubsystem extends SubsystemBase {

    private static final TalonFX ShooterMotor = new TalonFX(ShooterConstants.ShooterMotorID);
    private static final TalonFX ShooterMotor2 = new TalonFX(ShooterConstants.ShooterMotorID2);
    public static boolean shooterON = false;
    //shootermotor2 is canID 2 how convenient

        public static double shooterSpeed;
            public ShooterwoPIDSubsystem() 
            {
                shooterSpeed = ShooterConstants.shooterSpeed;
        
               addChild("ShooterMotor2", ShooterMotor2);
        
               var ShooterMotor2Configuration = new TalonFXConfiguration();
               ShooterMotor2Configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
               ShooterMotor2Configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
               
                       var ShooterMotorConfiguration = new TalonFXConfiguration();
               ShooterMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
               ShooterMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
            }
        
            public void start()
            {
                ShooterMotor.set(-shooterSpeed); 
                ShooterMotor2.set(shooterSpeed); //2 needs to be reversed
                System.out.println("shooterSpeed: " + shooterSpeed);
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
                shooterON = false;
            }
            public void increaseshooterSpeed() 
            {
                shooterSpeed += 0.05;
                System.out.println("shooterSpeed: " + shooterSpeed);
            }

            public void decreaseshooterSpeed() 
            {
                shooterSpeed -= 0.05;
                System.out.println("shooterSpeed: " + shooterSpeed);
            }
        
            public void setShooterSpeed(double newSpeed) {
                shooterSpeed = newSpeed;
            }
        
            //shooting from different areas
            public static void shooterSpeedHub() {
                shooterSpeed = 0.55;
    }

    public static void shooterSpeedMid() {
            shooterSpeed = 0.65;
        }
        
    public static void shooterSpeedTower() {
        shooterSpeed = 0.75;
    }

    

    @Override
    public void periodic() 
    {
    //   SmartDashboard.putNumber("Extender Position", getEncoderPosition());
    //   log();
    }

    //Katelynn is bad at coding...so is Mark Ruder. 
    //                            - Michael Murphy    
    
    //okay bud
}