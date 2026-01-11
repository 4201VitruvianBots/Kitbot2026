// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeShooter;

import frc.robot.constants.INTAKESHOOTER;
import frc.robot.constants.INTAKESHOOTER.INTAKE_SPEED_PERCENT;
import frc.robot.subsystems.IntakeShooter;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetIntakeShooterSpeeds extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final IntakeShooter m_intakeShooter;
  private final INTAKE_SPEED_PERCENT m_intakePercent;
  private final INTAKE_SPEED_PERCENT m_shootPercent;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetIntakeShooterSpeeds(IntakeShooter intake, INTAKE_SPEED_PERCENT intakeMotor, INTAKE_SPEED_PERCENT flywheelMotor) {
    m_intakeShooter = intake;
    m_intakePercent = intakeMotor;
    m_shootPercent = flywheelMotor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intakeShooter.setMotorSpeeds(m_intakePercent.get(), m_shootPercent.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
