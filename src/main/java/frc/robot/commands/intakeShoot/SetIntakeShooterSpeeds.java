// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeShoot;

import frc.robot.constants.INTAKESHOOTER.INTAKE_SPEED_PERCENT;
import frc.robot.subsystems.IntakeShooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetIntakeShooterSpeeds extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final IntakeShooter m_intakeShooter;
  private final INTAKE_SPEED_PERCENT m_shootPercent;
  private final INTAKE_SPEED_PERCENT m_kickerPercent;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   * @param shootPercent The motor output percentage of the intake and shooter
   * @param kickerPercent The motor output percentage of the kicker
   */
  public SetIntakeShooterSpeeds(IntakeShooter intake, INTAKE_SPEED_PERCENT shootPercent, INTAKE_SPEED_PERCENT kickerPercent) {
    m_intakeShooter = intake;
    m_shootPercent = shootPercent;
    m_kickerPercent = kickerPercent;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeShooter.setMotorSpeeds(m_shootPercent.get(), m_kickerPercent.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!DriverStation.isAutonomous()) m_intakeShooter.setMotorSpeeds(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
