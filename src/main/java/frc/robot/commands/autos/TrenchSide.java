// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetIntakeShooterSpeeds;
import frc.robot.constants.INTAKESHOOTER.INTAKE_SPEED_PERCENT;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeShooter;
import frc.team4201.lib.command.Auto;
import frc.team4201.lib.utils.TrajectoryUtils;

public class TrenchSide extends Auto {
  public TrenchSide(CommandSwerveDrivetrain swerveDrive, IntakeShooter intakeShooter, Climber climber, BooleanSupplier flipToRight) {
    try {
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      TrajectoryUtils trajectoryUtils = swerveDrive.getTrajectoryUtils();

      var m_path1 = PathPlannerPath.fromPathFile("TrenchSidePath1");
      var m_path2 = PathPlannerPath.fromPathFile("TrenchSidePath2");
      var m_path3 = PathPlannerPath.fromPathFile("TrenchSidePath3");

      addCommands(
          getPathCommand(trajectoryUtils, m_path1, flipToRight).andThen(() -> swerveDrive.setControl(stopRequest)),
          new ParallelRaceGroup (
            getPathCommand(trajectoryUtils, m_path2, flipToRight).andThen(() -> swerveDrive.setControl(stopRequest)),
            new SetIntakeShooterSpeeds(intakeShooter, INTAKE_SPEED_PERCENT.INTAKE, INTAKE_SPEED_PERCENT.KICKER_INTAKE)
          ),
          getPathCommand(trajectoryUtils, m_path3, flipToRight).andThen(() -> swerveDrive.setControl(stopRequest)),
          new SetIntakeShooterSpeeds(intakeShooter, INTAKE_SPEED_PERCENT.SHOOT, INTAKE_SPEED_PERCENT.KICKER_OUTAKE).withTimeout(10)
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for TrenchSide", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
}