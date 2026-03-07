
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class StopEverythingCommand extends Command {

      private HopperSubsystem m_HopperSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;

    public StopEverythingCommand(HopperSubsystem subsystem1, ShooterSubsystem subsystem2, IntakeSubsystem subsystem3){
        m_HopperSubsystem = subsystem1;
        m_ShooterSubsystem = subsystem2;
        m_IntakeSubsystem = subsystem3;
        addRequirements(m_HopperSubsystem, m_ShooterSubsystem, m_IntakeSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        System.out.println("TOTAL STOP");

        m_HopperSubsystem.stop();
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

