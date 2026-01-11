// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ExampleAuto extends SequentialCommandGroup {
  public ExampleAuto(CommandSwerveDrivetrain swerveDrive) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile("StartToPiece1");

      var m_startToPiece1 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("StartToPiece1");
      var m_piece1ToBarge = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("Piece1ToBarge");
      var m_bargeToPiece2 = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("BargeToPiece2");
      var m_piece2ToBarge = swerveDrive.getTrajectoryUtils().generatePPHolonomicCommand("Piece2ToBarge");

      var stopRequest = new SwerveRequest.ApplyRobotSpeeds();

      var starting_pose = path.getStartingHolonomicPose().orElseThrow();

      addCommands(
          new InstantCommand(() -> swerveDrive.resetPose(starting_pose)),
          m_startToPiece1.andThen(() -> swerveDrive.setControl(stopRequest)),
          new WaitCommand(3),
          m_piece1ToBarge.andThen(() -> swerveDrive.setControl(stopRequest)),
          new WaitCommand(3),
          m_bargeToPiece2.andThen(() -> swerveDrive.setControl(stopRequest)),
          new WaitCommand(3),
          m_piece2ToBarge.andThen(() -> swerveDrive.setControl(stopRequest)),
          new WaitCommand(3)
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path for TwoAlgaeRight", e.getStackTrace());
      addCommands(new WaitCommand(0));
    }
  }
}
