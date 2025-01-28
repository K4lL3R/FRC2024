// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Auton extends SequentialCommandGroup {
  /** Example static factory for an autonomous command. */
    public static boolean is4Note = false;
    public Auton(String pathName) {
      if (pathName.equals("4")) {
        is4Note = true;
      }
      Command autoPath = RobotContainer.s_Swerve.getAutonPath(pathName);

      addCommands(autoPath);
    }
}
