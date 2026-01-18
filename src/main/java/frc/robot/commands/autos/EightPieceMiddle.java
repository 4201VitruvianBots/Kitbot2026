// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetClimbSpeed;
import frc.robot.commands.SetIntakeShooterSpeeds;
import frc.robot.constants.CLIMBER.CLIMB_SPEED_PERCENT;
import frc.robot.constants.INTAKESHOOTER.INTAKE_SPEED_PERCENT;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeShooter;

public class EightPieceMiddle extends SequentialCommandGroup {
  public EightPieceMiddle(CommandSwerveDrivetrain swerveDrive, IntakeShooter intakeShooter, Climber climber) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("EightPieceMiddlePath1");
      var m_path2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("EightPieceMiddlePath2");

      addCommands(
          m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new SetIntakeShooterSpeeds(intakeShooter, INTAKE_SPEED_PERCENT.SHOOT, INTAKE_SPEED_PERCENT.KICKER_OUTAKE)
              .withTimeout(9),
          new ParallelCommandGroup(
            m_path2.andThen(() -> swerveDrive.setControl(stopRequest)),
            new SetClimbSpeed(climber, CLIMB_SPEED_PERCENT.UP).withTimeout(2.7)
          ),
          new SetClimbSpeed(climber, CLIMB_SPEED_PERCENT.DOWN).withTimeout(2.7)
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for EightPieceMiddle", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}