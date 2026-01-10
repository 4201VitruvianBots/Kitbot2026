// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.auto.Autos;
import frc.robot.commands.intakeShooter.SetIntakeShooterSpeeds;
import frc.robot.constants.USB;
import frc.robot.constants.INTAKESHOOTER.INTAKE_SPEED_PERCENT;
import frc.robot.subsystems.IntakeShooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private IntakeShooter m_intakeShooter;

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(USB.driver_xBoxController);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    initializeSubsystems();
    // Configure the trigger bindings
    configureBindings();
  }

  public void initializeSubsystems() {
    m_intakeShooter = new IntakeShooter();
  }

  public void initAutoChooser() {
    SmartDashboard.putData("Auto Mode", m_chooser);
    m_chooser.setDefaultOption("Do Nothing", new WaitCommand(0));
  }

  private void configureBindings() {
  
    //intake
    m_driverController.leftTrigger().whileTrue(
      new SetIntakeShooterSpeeds(m_intakeShooter, INTAKE_SPEED_PERCENT.INTAKE, INTAKE_SPEED_PERCENT.INTAKE));

    //shoot
    m_driverController.rightTrigger().whileTrue(
      new SetIntakeShooterSpeeds(m_intakeShooter, INTAKE_SPEED_PERCENT.INTAKE, INTAKE_SPEED_PERCENT.SHOOT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
