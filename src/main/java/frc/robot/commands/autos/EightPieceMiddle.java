// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.SetIntakeShooterSpeeds;
import frc.robot.constants.INTAKESHOOTER.INTAKE_SPEED_PERCENT;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeShooter;

public class EightPieceMiddle extends SequentialCommandGroup {
  public EightPieceMiddle(CommandSwerveDrivetrain swerveDrive, IntakeShooter intakeShooter) {
    try {

      PathPlannerPath path = PathPlannerPath.fromPathFile("EightPieceMiddlePath1");
      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var m_path1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("EightPieceMiddlePath1");

      addCommands(
              new InstantCommand(()->swerveDrive.resetPose(flippedStartingPosition(path))),
              m_path1.andThen(() -> swerveDrive.setControl(stopRequest)),
              new SetIntakeShooterSpeeds(intakeShooter, INTAKE_SPEED_PERCENT.INTAKE, INTAKE_SPEED_PERCENT.SHOOT),
              new WaitCommand(10)
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for EightPieceMiddle", e.getStackTrace());
      addCommands(new InstantCommand());
    }
  }
  private static Pose2d flippedStartingPosition(PathPlannerPath path) {
      boolean shouldFlip = DriverStation.getAlliance().orElse(Alliance.Blue).equals(DriverStation.Alliance.Red);
      if(shouldFlip) {
        return path.flipPath().getStartingHolonomicPose().get();
      } else {
        return path.getStartingHolonomicPose().get();
      }
  }
}
