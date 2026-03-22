// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// import frc.robot.commands.HopperIntakeCommand;
// import frc.robot.commands.HopperShooterCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstantsTestingRobot;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSlideSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MiddleWheelSubsystem;
// import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.NeckWheelSubsystem;
import frc.robot.commands.AutomaticMiddleWheel;
import frc.robot.commands.EmergencyIntakeCommand;
import frc.robot.commands.IntakeOnlyCommand;
import frc.robot.commands.ManualIntakeSlideMove;
import frc.robot.commands.ManualIntakeSlideMoveIn;
import frc.robot.commands.ReverseIntakeCommand;
import frc.robot.commands.RunShooterCommand;



public class RobotContainer {
    //private boolean shootingSpot = LimelightSubsystem.TAmove("limelight");
    private double shooterSpeed = ShooterSubsystem.shooterSpeed;
    private double neckSpeed = NeckWheelSubsystem.NeckWheelspeed;
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
    public final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
    public final LimelightSubsystem m_LimelightSubsystem = new LimelightSubsystem();
    public final MiddleWheelSubsystem m_MiddleWheelSubsystem = new MiddleWheelSubsystem();
    public final NeckWheelSubsystem m_NeckWheelSubsystem = new NeckWheelSubsystem();
    public final IntakeSlideSubsystem m_IntakeSlideSubsystem = new IntakeSlideSubsystem();

    public boolean isHubActive() { //i literally ripped this from wpilib
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){
            return false;
        }
        if(DriverStation.isAutonomousEnabled()){
            return true;
        }

        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        if(gameData.isEmpty()){
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
        
            default -> {
                return true;
            }
        }

        boolean shift1Active = switch(alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            return true;  
        }
        else if (matchTime > 105){
            return shift1Active;
        } else if (matchTime > 80) {
            return !shift1Active;
        } else if (matchTime > 55) {
            return shift1Active; 
        } else if (matchTime > 30) {
            return shift1Active;
        } else {
            return true;
        }
    }

    /* Path follower */
    //Eddie is a . (Verified) 
   private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //CameraServer.startAutomaticCapture();
        //suspicion the camera causes too much stress to the rio so the swerve disconnects?

        //reg subsystem
        NamedCommands.registerCommand("intake", new ParallelCommandGroup(
                new InstantCommand(() -> m_IntakeSubsystem.onlyIntake())
                ));
        NamedCommands.registerCommand("intake off", new ParallelCommandGroup(
                new InstantCommand(() -> m_IntakeSubsystem.stop()) , 
                new InstantCommand(() -> m_NeckWheelSubsystem.stop())
                ));
        NamedCommands.registerCommand("shoot",new ParallelCommandGroup(
                new InstantCommand(() -> m_IntakeSubsystem.onlyHopper()), 
                new InstantCommand(() -> m_IntakeSubsystem.setIntakeSpeed(.3)),
                new InstantCommand(() ->m_NeckWheelSubsystem.start())
                ));
        // NamedCommands.registerCommand("shooter off", new ParallelCommandGroup(
        //         new InstantCommand(() -> m_IntakeSubsystem.stop()) , 
        //         new InstantCommand(() ->m_NeckWheelSubsystem.stop()),
        //         new InstantCommand(() -> m_ShooterSubsystem.stop())
        //         ));

        NamedCommands.registerCommand("STOP EVERYTHING", new ParallelCommandGroup(
                new InstantCommand(() -> m_IntakeSubsystem.stop()), 
                new InstantCommand(() -> m_NeckWheelSubsystem.stop()),
                new InstantCommand(() -> m_ShooterSubsystem.stop())
                ));
        // NamedCommands.registerCommand("middle wheel start", new InstantCommand(() -> m_MiddleWheelSubsystem.start()));
        // NamedCommands.registerCommand("middle wheel stop", new InstantCommand(()-> m_MiddleWheelSubsystem.stop()));   
        
        NamedCommands.registerCommand("revUp", new InstantCommand( () -> m_ShooterSubsystem.start()));
        //yams subsystem + command 
        NamedCommands.registerCommand("intake move out"
            ,m_IntakeSlideSubsystem.moveToHeightCommand(-(Meters.convertFrom(5, Inches))).andThen(m_IntakeSlideSubsystem.stopCommand()));
        NamedCommands.registerCommand("intake move in",
         m_IntakeSlideSubsystem.moveToHeightCommand(Meters.convertFrom(0, Inches)));
        
        //limelight commands
        NamedCommands.registerCommand("limelight align", new ParallelCommandGroup(new InstantCommand(() -> drivetrain.applyRequest(() -> forwardStraight
                .withRotationalRate(-m_LimelightSubsystem.getHubTx("limelight")/Constants.VisionProfile.hubProportionalTx)
                .withVelocityX(0) // Reduced speed for fine adjustments
                .withVelocityY(driver.getLeftY()))
            ), new InstantCommand(() -> LimelightSubsystem.setLimelightPipeline("limelight", 0))));
        // NamedCommands.registerCommand("limelight far align", new ParallelCommandGroup(new InstantCommand(() -> drivetrain.applyRequest(() -> forwardStraight
        //         .withRotationalRate(-m_LimelightSubsystem.getHubTx("limelight")/Constants.VisionProfile.hubProportionalTx)
        //         .withVelocityX(0) // Reduced speed for fine adjustments
        //         .withVelocityY(driver.getLeftY()))
        //     ), new InstantCommand(() -> LimelightSubsystem.setLimelightPipeline("limelight", 0))));

        autoChooser = AutoBuilder.buildAutoChooser("move back and shoot and stop");
        
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        SmartDashboard.putNumber("SHOOTER SPEED", shooterSpeed);
        SmartDashboard.putNumber("NECKWHEEL SPEED", neckSpeed);
        SmartDashboard.putNumber("LIMELIGHT DISTANCE", LimelightHelpers.getTA("limelight"));
        SmartDashboard.getNumber("Match Time", DriverStation.getMatchTime()); //moved these 2 underneath 3-20-26 8:49am; didnt deploy it yet
        SmartDashboard.getBoolean("HUB ACTIVE?", isHubActive());
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

        //m_IntakeSubsystem.setDefaultCommand(new IntakeCommand(m_IntakeSubsystem));
        


        // driver.povUp().whileTrue(drivetrain.applyRequest(() ->
        //      forwardStraight.withVelocityX(0.5).withVelocityY(0))
        // );

        //  driver.povDown().whileTrue(drivetrain.applyRequest(() ->
        //      forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // driver.leftBumper().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driver.leftBumper().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driver.rightBumper().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driver.rightBumper().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // driver.leftTrigger().onTrue(Commands.runOnce(SignalLogger::start));
        // driver.rightTrigger().onTrue(Commands.runOnce(SignalLogger::stop));
        
        //DRIVER
        driver.rightBumper().whileTrue(
             drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed/10) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed/10) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //this is just the intake moving         
        driver.a().whileTrue(
                drivetrain.applyRequest(() -> forwardStraight
                    .withRotationalRate(m_LimelightSubsystem.getHubTx("limelight")/10.5) //this rotates it
                    //.withVelocityX(m_LimelightSubsystem.getHubTA("limelight")/10.5) // Reduced speed for fine adjustments 
                    //this moves it closer to the april tag
                    .withVelocityY(-driver.getLeftY()) //manually driving left and right
                )
            );

        driver.leftTrigger().whileTrue(new AutomaticMiddleWheel(m_MiddleWheelSubsystem));
        driver.rightTrigger().whileTrue(new RunShooterCommand(m_NeckWheelSubsystem, m_IntakeSubsystem, m_ShooterSubsystem));

        // //  reset the field centric
        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driver.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        driver.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        //OPERATOR

        operator.leftBumper().onTrue(
            m_IntakeSlideSubsystem.moveToHeightCommand(-(Meters.convertFrom(5, Inches))).andThen(m_IntakeSlideSubsystem.stopCommand()));
        operator.rightBumper().onTrue(
            m_IntakeSlideSubsystem.moveBackToZeroCommand(0.01).andThen(m_IntakeSlideSubsystem.stopCommand()));

        operator.leftTrigger().whileTrue(new EmergencyIntakeCommand(m_IntakeSubsystem, m_NeckWheelSubsystem));
        operator.rightTrigger().whileTrue(new IntakeOnlyCommand(m_IntakeSubsystem));
        //these down here are for manually moving the intake out if we get jammed
        operator.povLeft().whileTrue(new ManualIntakeSlideMove(m_IntakeSlideSubsystem));
        operator.povRight().whileTrue(new ManualIntakeSlideMoveIn(m_IntakeSlideSubsystem));

        //adding x and leftTrig since intake keeps getting jammed
        operator.x().whileTrue(new ReverseIntakeCommand(m_IntakeSubsystem));
        
        operator.povDown().onTrue(m_IntakeSlideSubsystem.stopCommand());

        operator.a().onTrue(new ParallelCommandGroup(
                new InstantCommand(() -> m_IntakeSubsystem.stop()), 
                new InstantCommand(() -> m_NeckWheelSubsystem.stop()),
                new InstantCommand(() -> m_ShooterSubsystem.stop())
                ));
        
        operator.b().onTrue(new InstantCommand(() -> m_NeckWheelSubsystem.neckSpit(.85)));
        operator.y().onTrue(new InstantCommand(() -> m_ShooterSubsystem.start()));

        operator.povUp().onTrue(new InstantCommand(() -> m_ShooterSubsystem.increaseshooterSpeed()));
        // operator.povUp().onTrue( new InstantCommand(() -> m_ShooterSubsystem.setShooterSpeed(0.63)));
        // operator.povDown().onTrue( new InstantCommand(() -> m_ShooterSubsystem.setShooterSpeed(0.48)));
        operator.povDown().onTrue(new InstantCommand(() -> m_ShooterSubsystem.decreaseshooterSpeed()));
        // operator.povLeft().onTrue(new InstantCommand(() -> m_NeckWheelSubsystem.increasetestingspeed()));
        // operator.povRight().onTrue(new InstantCommand(() -> m_NeckWheelSubsystem.decreasetestingspeed()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */

        // Edit made by JS - 2026-03-19 1440
        //return autoChooser.getSelected();
        return new InstantCommand(() -> {});
    }
}
