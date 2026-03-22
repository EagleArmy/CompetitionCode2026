package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class IntakeOnlyCommand extends Command{
    private IntakeSubsystem m_IntakeSubsystem;
    Joystick driver = new Joystick(0);

    public IntakeOnlyCommand(IntakeSubsystem subsystem1){
        m_IntakeSubsystem = subsystem1;
        addRequirements(m_IntakeSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        m_IntakeSubsystem.onlyIntake();
    }

    @Override
    public void end(boolean interrupted) { m_IntakeSubsystem.stop(); 
        System.out.println("INTAKE FINISHED");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}


