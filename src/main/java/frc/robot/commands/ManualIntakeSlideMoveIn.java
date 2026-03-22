package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSlideSubsystem;


public class ManualIntakeSlideMoveIn extends Command{
    private IntakeSlideSubsystem m_IntakeSlideSubsystem;
    Joystick operator = new Joystick(1);

    public ManualIntakeSlideMoveIn(IntakeSlideSubsystem subsystem1){
        m_IntakeSlideSubsystem = subsystem1;
        addRequirements(m_IntakeSlideSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        m_IntakeSlideSubsystem.moveItIn();
    }

    @Override
    public void end(boolean interrupted) { m_IntakeSlideSubsystem.stop();
        System.out.println("MOVED IN");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}


