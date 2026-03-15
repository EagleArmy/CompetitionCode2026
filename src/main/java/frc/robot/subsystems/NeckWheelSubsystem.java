package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableRegistry;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
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
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.NeckWheelConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NeckWheelSubsystem extends SubsystemBase {

    private final TalonFX NeckWheelMotor = new TalonFX(NeckWheelConstants.NeckWheelMotorID);
    //public final double NeckWheelspeed = NeckWheelConstants.NeckWheelSpeed;
    public static double NeckWheelspeed = NeckWheelConstants.NeckWheelSpeed;

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
                    volts -> NeckWheelMotor.setControl(m_sysIdControl.withOutput(volts)),
                    null,
                    this
                )
            );
    
        public NeckWheelSubsystem() 
        {
           addChild("NeckWheelMotor", NeckWheelMotor);
    
           var NeckWheelMotorConfiguration = new TalonFXConfiguration();
           NeckWheelMotorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
           NeckWheelMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
           NeckWheelMotor.getConfigurator().apply( NeckWheelMotorConfiguration );
    
        //   var slot0ConfigsLeft = NeckWheelMotorConfiguration.Slot0;
        // slot0ConfigsLeft.kS = 0.25; // Add 0.25 V output to overcome static friction
        // slot0ConfigsLeft.kV = 0.15; // A velocity target of 1 rps results in 0.12 V output
        // slot0ConfigsLeft.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        // slot0ConfigsLeft.kP = 4.8; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
        // slot0ConfigsLeft.kI = 0; // no output for integrated error
        // slot0ConfigsLeft.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var slot0ConfigsLeft = NeckWheelMotorConfiguration.Slot0;
        slot0ConfigsLeft.kS = 0.1717; // Add 0.25 V output to overcome static friction
        slot0ConfigsLeft.kV = 0.1178; // A velocity target of 1 rps results in 0.12 V output
        slot0ConfigsLeft.kA = 0.013901; // An acceleration of 1 rps/s requires 0.01 V output
        slot0ConfigsLeft.kP = 0.07933; // A position error of 2.5 rotations results in 12 V output   //4.8 originally
        slot0ConfigsLeft.kI = 0; // no output for integrated error
        slot0ConfigsLeft.kD = 0; // A velocity error of 1 rps results in 0.1 V output
    
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
            NeckWheelMotor.set(-NeckWheelspeed);
            System.out.println("NeckWheel Speed: " + NeckWheelspeed);
        }
    
        public void reverse() 
        {
            NeckWheelMotor.set(0.1);
        }
    
        public void stop()
        {
            NeckWheelMotor.set( 0 );
        }
    
        public static void setNeckWheelSpeed (double newSpeed){
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

    public Command joystickDriveCommand(DoubleSupplier output) {
        return run(() -> NeckWheelMotor.setControl(m_CycleOut.withOutput(output.getAsDouble())));
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

}
