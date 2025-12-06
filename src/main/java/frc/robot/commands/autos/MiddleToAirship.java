// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleToAirship extends SequentialCommandGroup {
  /** Creates a new MiddleToAirship. */
  public MiddleToAirship(CommandSwerveDrivetrain swervedrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    try {
        PathPlannerPath path = PathPlannerPath.fromPathFile("MiddleToAirship1");
        
        var m_MiddletoAirship1 = swervedrive.getTrajectoryUtils().generatePPHolonomicCommand("MiddleToArship1");
        var m_MiddletoAirship2 = swervedrive.getTrajectoryUtils().generatePPHolonomicCommand("MiddleToAirship2");
        
        var stoprequest = new SwerveRequest.ApplyRobotSpeeds();
        
        var starting_pose = path.getStartingHolonomicPose().orElseThrow();
        
       addCommands(
        new InstantCommand(() -> swervedrive.resetPose(starting_pose)),
        m_MiddletoAirship1.andThen(() -> swervedrive.setControl(stoprequest)),
        new WaitCommand(3),
        m_MiddletoAirship2.andThen(() -> swervedrive.setControl(stoprequest))
       );
    } catch (Exception e) {
        DriverStation.reportError("Failed to load path for MiddleToAirship", e.getStackTrace());
        addCommands(new WaitCommand(0));
    }
  }
}
