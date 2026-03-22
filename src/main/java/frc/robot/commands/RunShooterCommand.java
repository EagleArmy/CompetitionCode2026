package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MiddleWheelSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;


public class RunShooterCommand extends Command{
    private NeckWheelSubsystem m_NeckWheelSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;
    Joystick driver = new Joystick(0);

    public RunShooterCommand(NeckWheelSubsystem subsystem1, IntakeSubsystem subsystem2, ShooterSubsystem subsystem3){
        m_NeckWheelSubsystem = subsystem1;
        m_IntakeSubsystem = subsystem2;
        m_ShooterSubsystem = subsystem3;
        addRequirements(m_NeckWheelSubsystem, m_IntakeSubsystem, m_ShooterSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        
       m_IntakeSubsystem.onlyHopper();
       m_IntakeSubsystem.onlyIntake(); //commented out in fear the intake stops working
       m_NeckWheelSubsystem.start();
    }
    //m_IntakeSubsystem.stop();

    @Override
    public void end(boolean interrupted) { m_NeckWheelSubsystem.stop(); m_ShooterSubsystem.stop(); 
         m_IntakeSubsystem.stop(); 
         System.out.println("SHOOTING FINISHED");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}

