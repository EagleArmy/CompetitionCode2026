// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// import frc.robot.commands.HopperIntakeCommand;
// import frc.robot.commands.HopperShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSlideSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MiddleWheelSubsystem;
// import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterwoPIDSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;
import frc.robot.commands.StopEverythingCommand;



public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    //top tuner constant is real robot, bottom is test
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
     //public final CommandSwerveDrivetrain drivetrain = TunerConstantsTestingRobot.createDrivetrain();
    
    public final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    //public final HopperSubsystem m_HopperSubsystem = new HopperSubsystem();
    public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    public final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
    public final MiddleWheelSubsystem m_MiddleWheelSubsystem = new MiddleWheelSubsystem();
    public final NeckWheelSubsystem m_NeckWheelSubsystem = new NeckWheelSubsystem();
    public final IntakeSlideSubsystem m_IntakeSlideSubsystem = new IntakeSlideSubsystem();
    public final ShooterwoPIDSubsystem m_ShooterwoPIDSubsystem = new ShooterwoPIDSubsystem();

    /* Path follower */
   private final SendableChooser<Command> autoChooser;

    public RobotContainer() {

        //reg subsystem
        // NamedCommands.registerCommand("intake", new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.start()) , 
        //         new InstantCommand(() -> m_NeckWheelSubsystem.reverse())
        //         ));
        // NamedCommands.registerCommand("intake off", new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.stop()) , 
        //         new InstantCommand(() -> m_NeckWheelSubsystem.stop())
        //         ));
        // NamedCommands.registerCommand("shoot",new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.onlyHopper()) , 
        //         new InstantCommand(() ->m_NeckWheelSubsystem.start()),
        //         new InstantCommand(() -> m_ShooterSubsystem.start())
        //         ));
        // NamedCommands.registerCommand("shooter off", new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.stop()) , 
        //         new InstantCommand(() ->m_NeckWheelSubsystem.stop()),
        //         new InstantCommand(() -> m_ShooterSubsystem.stop())
        //         ));
        // NamedCommands.registerCommand("middle wheel start", new InstantCommand(() -> m_MiddleWheelSubsystem.start()));
        // NamedCommands.registerCommand("middle wheel start", new InstantCommand(()-> m_MiddleWheelSubsystem.stop()));        
        // //yams subsystem + command 
        // NamedCommands.registerCommand("move out intake", m_IntakeSlideSubsystem.moveToHeightCommand(Meters.convertFrom(6, Inches)));
        // NamedCommands.registerCommand("move in intake", m_IntakeSlideSubsystem.moveToHeightCommand(Meters.convertFrom(0, Inches)));
        
        // //limelight commands
        // NamedCommands.registerCommand("limelight align", new ParallelCommandGroup(new InstantCommand(() -> drivetrain.applyRequest(() -> forwardStraight
        //         .withRotationalRate(-m_LimelightSubsystem.getHubTx("limelight")/Constants.VisionProfile.hubProportionalTx)
        //         .withVelocityX(0) // Reduced speed for fine adjustments
        //         .withVelocityY(driver.getLeftY()))
        //     ), new InstantCommand(() -> LimelightSubsystem.setLimelightPipeline("limelight", 1))));
        // NamedCommands.registerCommand("limelight far align", new ParallelCommandGroup(new InstantCommand(() -> drivetrain.applyRequest(() -> forwardStraight
        //         .withRotationalRate(-m_LimelightSubsystem.getHubTx("limelight")/Constants.VisionProfile.hubProportionalTx)
        //         .withVelocityX(0) // Reduced speed for fine adjustments
        //         .withVelocityY(driver.getLeftY()))
        //     ), new InstantCommand(() -> LimelightSubsystem.setLimelightPipeline("limelight", 1))));



        autoChooser = AutoBuilder.buildAutoChooser("Right Path");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();

        // Warmup PathPlanner to avoid Java pauses
        FollowPathCommand.warmupCommand().schedule();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );  


        driver.povUp().whileTrue(drivetrain.applyRequest(() ->
             forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );

         driver.povDown().whileTrue(drivetrain.applyRequest(() ->
             forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //m_IntakeSlideSubsystem.setDefaultCommand(new InstantCommand(() -> m_IntakeSlideSubsystem.setVelocity(0.10)));

        //doing whatever/testing/being at the build team's beck and call button bindings

        // driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driver.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        // ));

        //press shooter!
        driver.rightBumper().onTrue(new InstantCommand(() -> m_ShooterwoPIDSubsystem.start()));
        //neck wheel moves
        driver.rightTrigger().onTrue(new ParallelCommandGroup(
                new InstantCommand(() -> m_IntakeSubsystem.onlyHopper()) , 
                new InstantCommand(() -> m_NeckWheelSubsystem.start())));
        driver.a().onTrue(new ParallelCommandGroup(
                new InstantCommand(() -> m_IntakeSubsystem.stop()) , 
                new InstantCommand(() -> m_NeckWheelSubsystem.stop()),
                new InstantCommand(() -> m_ShooterwoPIDSubsystem.stop())));
        //ultraShooterCommand should be repurposed for stopping everything

        //start intake
         driver.y().onTrue(new ParallelCommandGroup(
                new InstantCommand(() -> m_IntakeSubsystem.start()) , 
                new InstantCommand(() -> m_NeckWheelSubsystem.reverse())
                ));    
        //stop intake        
        driver.x().onTrue(new ParallelCommandGroup(
                new InstantCommand(() -> m_IntakeSubsystem.stop()) , 
                new InstantCommand(() -> m_NeckWheelSubsystem.stop())
                ));

        //hjopper and intake dont run at the same itme annymore
        //its just hopper and neckwheel

        driver.povUp().onTrue(
            m_IntakeSlideSubsystem.moveToHeightCommand(-(Meters.convertFrom(6, Inches))).andThen(m_IntakeSlideSubsystem.stopCommand()));
        driver.povDown().onTrue(
            m_IntakeSlideSubsystem.moveBackToZeroCommand(0).andThen(m_IntakeSlideSubsystem.stopCommand()));
        // driver.povUp().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );
        // driver.povDown().whileTrue(drivetrain.applyRequest(() ->
        //     forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // driver.y().onTrue(new InstantCommand( () -> m_ShooterSubsystem.start()));
        // driver.a().onTrue(new InstantCommand( () -> m_ShooterSubsystem.stop()));

        // driver.povUp().onTrue(new InstantCommand(() -> m_IntakeSubsystem.start()));
        // driver.povDown().onTrue(new InstantCommand(() -> m_IntakeSubsystem.stop()));
        // driver.povLeft().onTrue(new InstantCommand(() -> m_IntakeSubsystem.reverse()));

        //testing speeds for mid wheel
        // driver.x().onTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.start())); 
        // driver.b().onTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.stop()));
        // driver.y().onTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.increasetestingspeed()));
        // driver.a().onTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.decreasetestingspeed()));
        // driver.rightBumper().onTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.reverse()));

        //neck wheel thing; when you hold the button it runs, when you let go it stops 
        //driver.leftTrigger().whileFalse(new InstantCommand( () -> m_NeckWheelSubsystem.stop()));

        //COMPETITION BUTTON BINDINGS

        // driver.rightTrigger().whileTrue(
        //      drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-driver.getLeftY() * MaxSpeed/10) // Drive forward with negative Y (forward)
        //             .withVelocityY(driver.getLeftX() * MaxSpeed/10) // Drive left with negative X (left)
        //             .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // driver.rightBumper().debounce(0.2).whileTrue(
        //     drivetrain.applyRequest(() -> forwardStraight
        //         .withRotationalRate(-m_LimelightSubsystem.getHubTx("limelight")/Constants.VisionProfile.hubProportionalTx)
        //         .withVelocityX(m_LimelightSubsystem.getHubTA("limelight")) // Reduced speed for fine adjustments
        //         .withVelocityY(driver.getLeftY())
        //     )
        // );

        // driver.y().onTrue(new InstantCommand(() -> m_IntakeSubsystem.start()));
        // driver.x().onTrue(new InstantCommand(() -> m_IntakeSubsystem.stop()));

        //this is just the intake moving 
        // driver.y().onTrue(new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.start()) , 
        //         new InstantCommand(() -> m_NeckWheelSubsystem.reverse())
        //         ));            
        // driver.x().onTrue(new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.stop()) , 
        //         new InstantCommand(() -> m_NeckWheelSubsystem.stop())
        //         ));


        // driver.leftTrigger().whileTrue(new InstantCommand(() -> m_MiddleWheelSubsystem.reverse()));
         //driver.leftBumper().whileTrue(new InstantCommand( () -> m_MiddleWheelSubsystem.start()));
        // driver.leftBumper().and(driver.leftTrigger()).whileFalse(new InstantCommand(() -> m_MiddleWheelSubsystem.stop()));
        // //driver.leftBumper().whileFalse(new InstantCommand( () -> m_MiddleWheelSubsystem.stop()));
        // driver.povRight().onTrue( new InstantCommand( () -> m_ElevatorSubsystem.setVelocity(0)));

        // operator.b().onTrue(m_ElevatorSubsystem.setHeightCommand(Meters.convertFrom(33, Inches)));
        // operator.a().onTrue(m_ElevatorSubsystem.setHeightCommand(0));

        //shooter on!
        
        // operator.rightTrigger().whileTrue(new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.onlyHopper()) , 
        //         new InstantCommand(() ->m_NeckWheelSubsystem.start()),
        //         new InstantCommand(() -> m_ShooterSubsystem.start())));
        // operator.rightTrigger().whileFalse(new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.stop()) , 
        //         new InstantCommand(() ->m_NeckWheelSubsystem.stop()),
        //         new InstantCommand(() -> m_ShooterSubsystem.stop())));

        // operator.x().onTrue(new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.setIntakeHopperSpeed(0.7, 0.9)) , 
        //         new InstantCommand(() -> m_ShooterSubsystem.setShooterSpeed(0.6))));

        // driver.leftBumper().onTrue(m_IntakeSlideSubsystem.moveToHeightCommand(
        //     0.15));
        // driver.a().onTrue(m_IntakeSlideSubsystem.stopCommand());
        // driver.rightBumper().onTrue(m_IntakeSlideSubsystem.moveToHeightCommand(0));

        
         // Reset the field-centric heading on left bumper press.
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
