// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FIELD;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Controls;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
private final CommandSwerveDrivetrain m_SwerveDrivetrain;
private final Vision m_vision;

  Translation2d m_goal = new Translation2d();

// private final PIDController m_PidController = 
//   new PIDController(DRIVE.kTeleP_Theta, DRIVE.kTeleI_Theta, DRIVE.kTeleD_Theta);
//TODO: ask Gavin if we're doing this this year
private final DoubleSupplier m_throttleInput;
private final DoubleSupplier m_turnInput; 

  /** Creates a new AutoAlign. */
  public AutoAlign(
    CommandSwerveDrivetrain commandSwerveDrivetrain,
    Vision vision,
    DoubleSupplier throttleInput,
    DoubleSupplier turnInput) {
      m_SwerveDrivetrain = commandSwerveDrivetrain;
      m_vision = vision;
      m_throttleInput = throttleInput;
      m_turnInput = turnInput;
      // m_PidController.setTolerance(Units.degreesToRadians(2));
      // m_PidController.enableContinuousInput(-Math.PI, Math.PI);
      //TODO: ask Gavin if we're doing this this year
    addRequirements(m_SwerveDrivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_PidController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Controls.isBlueAlliance()){
      m_goal = FIELD.blueHub;
    } 
    else {
      m_goal = FIELD.redHub;
    }
    var setPoint = m_SwerveDrivetrain.getState().Pose.getTranslation().minus(m_goal);
    // var turnRate = 
    //   m_PidController.calculate(
    //     m_SwerveDrivetrain.getState().Pose.getRotation().getRadians(),
    //     setPoint.getAngle().getRadians());
    // m_SwerveDrivetrain.setChassisSpeedControl(
    //     new ChassisSpeeds(
    //       m_throttleInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
    //       m_turnInput.getAsDouble() * DRIVE.kMaxSpeedMetersPerSecond,
    //       turnRate));
    //TODO: ask Gavin if we're doing PID controller still
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
