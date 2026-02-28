package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.NeckWheelConstants;
import frc.robot.Constants.ShooterConstants;


public class UltraShooterCommand extends Command {
    private NeckWheelSubsystem m_NeckWheelSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;
    private IntakeSubsystem m_IntakeSubsystem; 

    public UltraShooterCommand(NeckWheelSubsystem subsystem1, ShooterSubsystem subsystem2, IntakeSubsystem subsystem3 ){
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

        //speed of the neck wheel SHOULD be already set with the april tag
        m_NeckWheelSubsystem.setNeckWheelSpeed(NeckWheelConstants.NeckWheelSpeed);
        m_ShooterSubsystem.setShooterSpeed(ShooterConstants.shooterSpeed);
        m_IntakeSubsystem.setIntakeHopperSpeed(IntakeConstants.intakeSpeed, IntakeConstants.hopperSpeed); // the first parameter is intake, the 2nd one is the hopper

        m_NeckWheelSubsystem.start();
        m_ShooterSubsystem.start();
        m_IntakeSubsystem.start();
    }   

    @Override //what happens when the command is finished lol
    public void end(boolean interrupted) { 
        m_ShooterSubsystem.stop();
        m_IntakeSubsystem.stop();
        m_NeckWheelSubsystem.stop();
     System.out.println("Shooting FINISHED!");
     }
  
    @Override
    public boolean isFinished() {
        if (ShooterSubsystem.shooterON){
            return true;
        }
      return false;
    }
}
