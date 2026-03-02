package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;


public class HopperIntakeCommand extends Command{
    private HopperSubsystem m_HopperSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;

    public HopperIntakeCommand(HopperSubsystem subsystem1, IntakeSubsystem subsystem2){
        m_HopperSubsystem = subsystem1;
        m_IntakeSubsystem = subsystem2;
        addRequirements(m_HopperSubsystem, m_IntakeSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        System.out.println("Hopper moves with intake!");

        m_HopperSubsystem.start();
        m_IntakeSubsystem.start();
    }

    @Override
    public void end(boolean interrupted) { m_HopperSubsystem.stop(); System.out.println("Coral Desposit FINISHED");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
