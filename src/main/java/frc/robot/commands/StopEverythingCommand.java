
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;

public class StopEverythingCommand extends Command {

      private NeckWheelSubsystem m_NeckWheelSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;

    public StopEverythingCommand(NeckWheelSubsystem subsystem1, ShooterSubsystem subsystem2, IntakeSubsystem subsystem3){
        m_NeckWheelSubsystem= subsystem1;
        m_ShooterSubsystem = subsystem2;
        m_IntakeSubsystem = subsystem3;
        addRequirements(m_NeckWheelSubsystem, m_ShooterSubsystem, m_IntakeSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        System.out.println("TOTAL STOP");

        m_NeckWheelSubsystem.stop();
        m_ShooterSubsystem.stop();
        m_IntakeSubsystem.stop();

    }

    @Override
    public void end(boolean interrupted) { }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}

