package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class HopperShooterCommand extends Command{
    private HopperSubsystem m_HopperSubsystem;
    private ShooterSubsystem m_ShooterSubsystem;

    public HopperShooterCommand(HopperSubsystem subsystem1, ShooterSubsystem subsystem2){
        m_HopperSubsystem = subsystem1;
        m_ShooterSubsystem = subsystem2;
        addRequirements(m_HopperSubsystem, m_ShooterSubsystem);
    }

    @Override 
    public void initialize() {}

    @Override
    public void execute (){
        System.out.println("Hopper works with Shooter!");

        m_HopperSubsystem.start();
        m_ShooterSubsystem.set(.55);

    }

    @Override
    public void end(boolean interrupted) { m_HopperSubsystem.stop(); System.out.println("Shooting FINISHED");}
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
