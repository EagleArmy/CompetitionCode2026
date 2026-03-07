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
public class ShooterSubsystem extends SubsystemBase {

    private static final TalonFX ShooterMotor = new TalonFX(ShooterConstants.ShooterMotorID);
    private static final TalonFX ShooterMotor2 = new TalonFX(ShooterConstants.ShooterMotorID2);
    public static boolean shooterON = false;
    //shootermotor2 is canID 2 how convenient

        public static double shooterSpeed;
            public ShooterSubsystem() 
            {
                shooterSpeed = ShooterConstants.shooterSpeed;
        
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
            slot0ConfigsRight.kS = 0.25; // Add 0.25 V output to overcome static friction
            slot0ConfigsRight.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
            slot0ConfigsRight.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
            slot0ConfigsRight.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
            slot0ConfigsRight.kI = 0; // no output for integrated error
            slot0ConfigsRight.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
            var slot0Configs = ShooterMotorConfiguration.Slot0;
            slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
            slot0Configs.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
            slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
            slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
            slot0Configs.kI = 0; // no output for integrated error
            slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
        
        
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
                ShooterMotor.set(shooterSpeed); 
                ShooterMotor2.set(-shooterSpeed); //2 needs to be reversed
                System.out.println("shooterSpeed: " + shooterSpeed);
            }
        
            // public void set(double speed, double speed2){
            //     ShooterMotor.set(-speed);
            //     ShooterMotor2.set(speed);
            // }

        
            public void reverse() 
            {
                ShooterMotor.set(-shooterSpeed);
                ShooterMotor2.set(shooterSpeed);
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
      //SmartDashboard.putNumber("Extender Position", getEncoderPosition());
      //log();
    }

    //Katelynn is bad at coding...so is Mark Ruder. 
    //                            - Michael Murphy    
    
    //okay bud
}


//worthless yams slop below


// package frc.robot.subsystems;

// import static edu.wpi.first.units.Units.Amps;
// import static edu.wpi.first.units.Units.Degrees;
// import static edu.wpi.first.units.Units.DegreesPerSecond;
// import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
// import static edu.wpi.first.units.Units.Feet;
// import static edu.wpi.first.units.Units.Pounds;
// import static edu.wpi.first.units.Units.Second;
// import static edu.wpi.first.units.Units.Seconds;
// import static edu.wpi.first.units.Units.Volts;
// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.RPM;
// import com.ctre.phoenix6.hardware.TalonFX;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ShooterConstants;
// import yams.gearing.GearBox;
// import yams.gearing.MechanismGearing;
// import yams.mechanisms.SmartMechanism;
// import yams.mechanisms.config.Config;
// import yams.mechanisms.velocity.;
// import yams.motorcontrollers.SmartMotorController;
// import yams.motorcontrollers.SmartMotorControllerConfig;
// import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
// import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
// import yams.motorcontrollers.local.SparkWrapper;
// import yams.motorcontrollers.remote.TalonFXWrapper;

// public class ShooterSubsystem extends SubsystemBase {

//   private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
//   .withControlMode(ControlMode.CLOSED_LOOP)
//   // Feedback Constants (PID Constants)
//   .withClosedLoopController(1, 0, 0)
//   .withSimClosedLoopController(1, 0, 0)
//   // Feedforward Constants
//   .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
//   .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
//   // Telemetry name and verbosity level
//   .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
//   // Gearing from the motor rotor to final shaft.
//   // In this example GearBox.fromReductionStages(3,4) is the same as GearBox.fromStages("3:1","4:1") which corresponds to the gearbox attached to your motor.
//   // You could also use .withGearing(12) which does the same thing.
//   .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//   // Motor properties to prevent over currenting.
//   .withMotorInverted(false)
//   .withIdleMode(MotorMode.COAST)
//   .withStatorCurrentLimit(Amps.of(40));

//   // Vendor motor controller object
//   private TalonFX shooterMotor = new TalonFX(ShooterConstants.ShooterMotorID);

//   // Create our SmartMotorController from our Spark and config with the NEO.
//   private SmartMotorController encoderController = new TalonFXWrapper(shooterMotor, DCMotor.getFalcon500(1), smcConfig);
//  private final Config shooterConfig = new Config(encoderController)
//   // Diameter of the .
//   .withDiameter(Inches.of(4))
//   // Mass of the .
//   .withMass(Pounds.of(1))
//   // Maximum speed of the shooter.
//   .withUpperSoftLimit(RPM.of(1000));
//   // Telemetry name and verbosity for the arm.
//   //.withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

//   // Shooter Mechanism
//   private  shooter = new (shooterConfig);

//   /**
//    * Gets the current velocity of the shooter.
//    *
//    * @return Shooter velocity.
//    */
//   public AngularVelocity getVelocity() {return shooter.getSpeed();}

//   /**
//    * Set the shooter velocity.
//    *
//    * @param speed Speed to set.
//    * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
//    */
//   public Command setVelocity(AngularVelocity speed) {return shooter.run(speed);}
  
//   /**
//    * Set the shooter velocity setpoint.
//    *
//    * @param speed Speed to set
//    */
//   public void setVelocitySetpoint(AngularVelocity speed) {shooter.setMechanismVelocitySetpoint(speed);}
  
//   /**
//    * Set the shooter velocity.
//    *
//    * @param speed Speed to set.
//    * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
//    */
//   public Command setVelocityShooter(AngularVelocity speed) {return shooter.run(speed);}

//   /**
//    * Set the dutycycle of the shooter.
//    *
//    * @param dutyCycle DutyCycle to set.
//    * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
//    */
//   public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

//   /** Creates a new ExampleSubsystem. */
//   public ShooterSubsystem() {}

//   /**
//    * Example command factory method.
//    *
//    * @return a command
//    */
//   public Command exampleMethodCommand() {
//     // Inline construction of command goes here.
//     // Subsystem::RunOnce implicitly requires `this` subsystem.
//     return runOnce(
//         () -> {
//           /* one-time action goes here */
//         });
//   }

//   /**
//    * An example method querying a boolean state of the subsystem (for example, a digital sensor).
//    *
//    * @return value of some boolean subsystem state, such as a digital sensor.
//    */
//   public boolean exampleCondition() {
//     // Query some boolean state, such as a digital sensor.
//     return false;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     shooter.updateTelemetry();
//   }

//   @Override
//   public void simulationPeriodic() {
//     // This method will be called once per scheduler run during simulation
//     shooter.simIterate();
//   }
// }

