package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;

public class UltraIntakeCommand extends Command {
        private NeckWheelSubsystem m_NeckWheelSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;

    // private double neckSpeed = 0.3;
    // private double shooterSpeed = 0.4;

    public UltraIntakeCommand(NeckWheelSubsystem subsystem1, ShooterSubsystem subsystem2, IntakeSubsystem subsystem3 ){
        m_NeckWheelSubsystem = subsystem1;
        m_ShooterSubsystem = subsystem2;
        m_IntakeSubsystem = subsystem3;
        addRequirements(m_NeckWheelSubsystem, m_ShooterSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        // System.out.println("Neck works with Shooter!");
        m_NeckWheelSubsystem.setNeckWheelSpeed(.3);
        m_NeckWheelSubsystem.start();
        m_ShooterSubsystem.setShooterSpeed(.4);
        m_ShooterSubsystem.start();

        // the first parameter is intake, the 2nd one is the hopper
        m_IntakeSubsystem.setIntakeHopperSpeed(IntakeConstants.intakeSpeed, IntakeConstants.hopperSpeed);
    }   

    @Override

    public void end(boolean interrupted) { m_IntakeSubsystem.stop(); 
    m_ShooterSubsystem.stop();
    m_NeckWheelSubsystem.stop();
    System.out.println("Intaking FINISHED!");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
