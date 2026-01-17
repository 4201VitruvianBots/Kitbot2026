// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ResetGyro;
import frc.robot.commands.SetClimbSpeed;
import frc.robot.commands.SetIntakeShooterSpeeds;
import frc.robot.commands.autos.EightPieceMiddle;
import frc.robot.commands.autos.EightPieceRight;
import frc.robot.constants.INTAKESHOOTER.INTAKE_SPEED_PERCENT;
import frc.robot.constants.USB;
import frc.robot.constants.CLIMBER.CLIMB_SPEED_PERCENT;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeShooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.Telemetry;
import frc.team4201.lib.simulation.FieldSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private CommandSwerveDrivetrain m_swerveDrive = TunerConstants.createDrivetrain();
  private IntakeShooter m_intakeShooter = new IntakeShooter();
  private Climber m_climber = new Climber();
  

  private final Telemetry m_telemetry = new Telemetry();
  private final FieldSim m_fieldSim = new FieldSim();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.driver_xBoxController);
      
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 2.42; // kSpedAt12Volts desired top speed
  
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  /* Setting up bindings for necessary control of the swerve drive platform */
  
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.3)
          .withRotationalDeadband(MaxAngularRate * 0.1); // Add a 10% deadband

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    intializeSubsystems();
    initAutoChooser();
    
    SmartDashboard.putData(new ResetGyro(m_swerveDrive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    
    //intake
    m_driverController.leftTrigger().whileTrue(
      new SetIntakeShooterSpeeds(m_intakeShooter, INTAKE_SPEED_PERCENT.INTAKE, INTAKE_SPEED_PERCENT.KICKER_INTAKE));

    //shoot
    m_driverController.rightTrigger().whileTrue(
      new SetIntakeShooterSpeeds(m_intakeShooter, INTAKE_SPEED_PERCENT.SHOOT, INTAKE_SPEED_PERCENT.KICKER_OUTAKE));

    //climb up
    m_driverController.povUp().whileTrue(new SetClimbSpeed(m_climber, CLIMB_SPEED_PERCENT.UP));
    
    //climb down
    m_driverController.povDown().whileTrue(new SetClimbSpeed(m_climber, CLIMB_SPEED_PERCENT.DOWN));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_autoChooser.getSelected();
  }
  private void intializeSubsystems(){
    
    // Set Subsystem DefaultCommands
    m_swerveDrive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        m_swerveDrive.applyRequest(
            () -> {
              var rotationRate = -m_driverController.getRightX() * MaxAngularRate;
              // // if heading target
              // if (m_swerveDrive.isTrackingState()) {
              //   rotationRate = m_swerveDrive.calculateRotationToTarget();
              // }
              drive
                  .withVelocityX(
                      -m_driverController.getLeftY()
                          * MaxSpeed) // Drive forward with negative Y (forward)
                  .withVelocityY(
                      -m_driverController.getLeftX()
                          * MaxSpeed) // Drive left with negative X (left)
                  .withRotationalRate(
                      rotationRate); // Drive counterclockwise with negative X (left)
              return drive;
            }));

    m_telemetry.registerFieldSim(m_fieldSim);
    m_swerveDrive.registerTelemetry(m_telemetry::telemeterize);
  }

  private void initAutoChooser() {
    SmartDashboard.putData("Auto Mode", m_autoChooser);
    m_autoChooser.setDefaultOption("Do Nothing", new WaitCommand(0));

    m_autoChooser.addOption("EightPieceMiddle", new EightPieceMiddle(m_swerveDrive, m_intakeShooter));
    m_autoChooser.addOption("EightPieceRight", new EightPieceRight(m_swerveDrive, m_intakeShooter));
  }
}
