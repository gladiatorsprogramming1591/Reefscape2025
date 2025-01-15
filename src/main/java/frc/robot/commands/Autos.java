// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;

import com.pathplanner.lib.auto.NamedCommands;

public final class Autos {
  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem) {
    return driveSubsystem.driveArcade(driveSubsystem, () -> 0.5, () -> 0.0).withTimeout(1.0);
  }

  public static final Command spitAuto(CANDriveSubsystem driveSubsystem, CANRollerSubsystem rollerSubsystem) {
    return driveSubsystem.driveArcade(driveSubsystem, () -> 0.25, () -> 0.0).withTimeout(1.0).andThen(new RunCommand(() -> rollerSubsystem.runRoller(rollerSubsystem, () -> 0.44, () -> 0))); 
    
  }

  public void registerNamedCommands(){
    
  }
}
