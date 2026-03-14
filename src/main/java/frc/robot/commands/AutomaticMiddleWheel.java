package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MiddleWheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class AutomaticMiddleWheel extends Command{
    private MiddleWheelSubsystem m_MiddleWheelSubsystem;
    Joystick driver = new Joystick(0);

    public AutomaticMiddleWheel(MiddleWheelSubsystem subsystem1){
        m_MiddleWheelSubsystem = subsystem1;
        addRequirements(m_MiddleWheelSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        
        if(driver.getX() >= 0)
        {
            m_MiddleWheelSubsystem.start();
        } else {
            m_MiddleWheelSubsystem.reverse();
        }
    }

    @Override
    public void end(boolean interrupted) { m_MiddleWheelSubsystem.stop(); System.out.println("middle wheel FINISHED");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}

