package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class ReverseIntakeCommand extends Command{
    private IntakeSubsystem m_IntakeSubsystem;
    Joystick driver = new Joystick(0);

    public ReverseIntakeCommand(IntakeSubsystem subsystem1){
        m_IntakeSubsystem = subsystem1;
        addRequirements(m_IntakeSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        m_IntakeSubsystem.reverse();
    }

    @Override
    public void end(boolean interrupted) { m_IntakeSubsystem.stop(); 
        System.out.println("REVERSE FINISHED");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}


