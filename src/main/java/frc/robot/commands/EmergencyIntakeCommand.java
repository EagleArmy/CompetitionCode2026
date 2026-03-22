package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class EmergencyIntakeCommand extends Command{
    private IntakeSubsystem m_IntakeSubsystem;
    private NeckWheelSubsystem m_NeckWheelSubsystem;
    Joystick driver = new Joystick(0);

    public EmergencyIntakeCommand(IntakeSubsystem subsystem1, NeckWheelSubsystem subsystem2){
        m_IntakeSubsystem = subsystem1;
        m_NeckWheelSubsystem = subsystem2;
        addRequirements(m_IntakeSubsystem, m_NeckWheelSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        m_IntakeSubsystem.onlyIntake();
        //m_IntakeSubsystem.reverse();
        //reverse reverses BOTH intake and hopper at their default speeds rn
        m_IntakeSubsystem.setHopperSpeed(.2);
        m_NeckWheelSubsystem.reverse();
    }

    @Override
    public void end(boolean interrupted) { m_IntakeSubsystem.stop(); 
        m_NeckWheelSubsystem.stop(); 
        System.out.println("EXPEL FINISHED");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}


