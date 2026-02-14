// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;

//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ElevatorConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ElevatorSubsystem extends SubsystemBase {
  
  private final TalonFX ElevatorLeftMotor = new TalonFX(ElevatorConstants.kElevatorLeftMotorID);
  //private final TalonFX ElevatorRightMotor = new TalonFX(ElevatorConstants.kElevatorRightMotorID);
  public double leftPos;
  public double rightPos;
  // private final MotionMagicVoltage m_LeftRequest = new MotionMagicVoltage(ElevatorConstants.kFirstPosition);
  // private final MotionMagicVoltage m_RightRequest = new MotionMagicVoltage(ElevatorConstants.kFirstPosition);
  public double switchPos;
  //private double LevelFourPosition;

  public ElevatorSubsystem() {
    // LevelFourPosition = ElevatorConstants.kFourthPosition;
    // ElevatorLeftMotor.setPosition( ElevatorConstants.kStartPosition );
    // ElevatorRightMotor.setPosition( ElevatorConstants.kStartPosition );
    //leftPos = getLeftEncoderPosition().baseUnitMagnitude();     //degrees
    //rightPos = getRightEncoderPosition().baseUnitMagnitude();   //degrees
    
    // Let's name everything on the LiveWindow
    addChild("ElevatorLeftMotor", ElevatorLeftMotor);
    //addChild("ElevatorLeftMotor", ElevatorRightMotor);

    var ElevatorLeftMotorConfiguration = new TalonFXConfiguration();
    ElevatorLeftMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    ElevatorLeftMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    ElevatorLeftMotor.getConfigurator().apply( ElevatorLeftMotorConfiguration );

    var ElevatorRightMotorConfiguration = new TalonFXConfiguration();
    ElevatorRightMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    ElevatorRightMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    //ElevatorRightMotor.getConfigurator().apply( ElevatorRightMotorConfiguration );

    // // set slot 0 gains
    var slot0ConfigsLeft = ElevatorLeftMotorConfiguration.Slot0;
    slot0ConfigsLeft.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0ConfigsLeft.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    slot0ConfigsLeft.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0ConfigsLeft.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
    slot0ConfigsLeft.kI = 0; // no output for integrated error
    slot0ConfigsLeft.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // var slot0ConfigsRight = ElevatorRightMotorConfiguration.Slot0;
    // slot0ConfigsRight.kS = 0.25; // Add 0.25 V output to overcome static friction
    // slot0ConfigsRight.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
    // slot0ConfigsRight.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    // slot0ConfigsRight.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    // slot0ConfigsRight.kI = 0; // no output for integrated error
    // slot0ConfigsRight.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    // // set Motion Magic settings
    var motionMagicConfigsLeft = ElevatorLeftMotorConfiguration.MotionMagic;
    motionMagicConfigsLeft.MotionMagicCruiseVelocity = 7000; // Target cruise velocity of 80 rps
    motionMagicConfigsLeft.MotionMagicAcceleration = 14000; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsLeft.MotionMagicJerk = 90000; // Target jerk of 1600 rps/s/s (0.1 seconds)

    // // set Motion Magic settings
    var motionMagicConfigsRight = ElevatorRightMotorConfiguration.MotionMagic;
    motionMagicConfigsRight.MotionMagicCruiseVelocity = 7000; // Target cruise velocity of 80 rps
    motionMagicConfigsRight.MotionMagicAcceleration = 14000; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigsRight.MotionMagicJerk = 90000; // Target jerk of 1600 rps/s/s (0.1 seconds
    
    ElevatorLeftMotor.getConfigurator().apply(ElevatorLeftMotorConfiguration);
    //ElevatorRightMotor.getConfigurator().apply(ElevatorRightMotorConfiguration);
    System.out.println("Starting Left Position: " + leftPos + "        Starting Right Position: " + rightPos);
  }


  // public void firstLevel() {
  //   ElevatorLeftMotor.setControl(m_LeftRequest.withPosition(ElevatorConstants.kFirstPosition));
  //   leftPos = getLeftEncoderPosition().baseUnitMagnitude();
  //   ElevatorRightMotor.setControl(m_RightRequest.withPosition(ElevatorConstants.kFirstPosition));
  //   rightPos = getRightEncoderPosition().baseUnitMagnitude();
  // }
  // public void secondLevel() {
  //   ElevatorLeftMotor.setControl(m_LeftRequest.withPosition(ElevatorConstants.kSecondPosition));
  //   leftPos = getLeftEncoderPosition().baseUnitMagnitude();
  //   ElevatorRightMotor.setControl(m_RightRequest.withPosition(ElevatorConstants.kSecondPosition));
  //   rightPos = getRightEncoderPosition().baseUnitMagnitude();
  // }
  // public void thirdLevel() {
  //   ElevatorLeftMotor.setControl(m_LeftRequest.withPosition(ElevatorConstants.kThirdPosition));
  //   leftPos = getLeftEncoderPosition().baseUnitMagnitude();
  //   ElevatorRightMotor.setControl(m_RightRequest.withPosition(ElevatorConstants.kThirdPosition));
  //   rightPos = getRightEncoderPosition().baseUnitMagnitude();
  // }
  // public void fourthLevel() {
  //   ElevatorLeftMotor.setControl(m_LeftRequest.withPosition(ElevatorConstants.kFourthPosition));
  //   leftPos = getLeftEncoderPosition().baseUnitMagnitude();
  //   ElevatorRightMotor.setControl(m_RightRequest.withPosition(ElevatorConstants.kFourthPosition));
  //   rightPos = getRightEncoderPosition().baseUnitMagnitude();
  // }

  // public void startingPos() {
  //   ElevatorLeftMotor.setControl(m_LeftRequest.withPosition(ElevatorConstants.kStartPosition));
  //   leftPos = getLeftEncoderPosition().baseUnitMagnitude();
  //   ElevatorRightMotor.setControl(m_RightRequest.withPosition(ElevatorConstants.kStartPosition));
  //   rightPos = getRightEncoderPosition().baseUnitMagnitude();
  // }
  

  // public void moveToPos() {
  //   ElevatorLeftMotor.setControl(m_LeftRequest.withPosition(switchPos));
  //   leftPos = getLeftEncoderPosition().baseUnitMagnitude();
  //   ElevatorRightMotor.setControl(m_RightRequest.withPosition(switchPos));
  //   rightPos = getRightEncoderPosition().baseUnitMagnitude();
  // }


  public void moveUp() {
    ElevatorLeftMotor.set(-0.2);
    //ElevatorRightMotor.set(0.2);
  }
  public void moveDown() {
    ElevatorLeftMotor.set(0.15);
    //ElevatorRightMotor.set(-0.15);
  }
  public void manualstop() {
    ElevatorLeftMotor.set(0);
    //ElevatorRightMotor.set(0);
    //System.out.println("Left position: " + getleftPos());
    //System.out.println("Right Position!!!: " + getrightPos());
  }


  // public double getleftPos() {
  //   return leftPos;
  // }
  // public double getrightPos() {
  //   return rightPos;
  // }


  // public Angle getLeftEncoderPosition() {
  //   var rotorPosSignal = ElevatorLeftMotor.getRotorPosition();
  //   var rotorPos = rotorPosSignal.getValue();
  //   return rotorPos;
  //}
  // public Angle getRightEncoderPosition() {
  //   var rotorPosSignal = ElevatorRightMotor.getRotorPosition();
  //   var rotorPos = rotorPosSignal.getValue();
  //   return rotorPos;
  // }


  // public void increaseLFourPos() {
  //   System.out.println("L4 Elevator Pos: " + getLFourPos()) ;
  //   LevelFourPosition += 0.1;
  // }
  // public void decreaseLFourPos() {
  //   System.out.println("L4 Elevator Pos: " + getLFourPos()) ;
  //   LevelFourPosition -= 0.1;
  // }





  // public void setLeftEncoderPosition( Angle newLeftPosition) {
  //   leftMotorPosition = newLeftPosition;
  // }
  // public void setRightEncoderPosition( Angle newRightPosition) {
  //   rightMotorPosition = newRightPosition;
  // }



  public void log() {
    // System.out.println("Current Left Position: " + leftPos + "        Current Right Position: " + rightPos);
    // SmartDashboard.putData("ElevatorMotor", ElevatorMotor);
  }
  
  /** Call log method every loop. */
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
    log();
  }

}