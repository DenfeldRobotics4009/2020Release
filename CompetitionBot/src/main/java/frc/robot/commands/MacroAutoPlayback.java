/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;
import java.util.Scanner;
import java.io.FileNotFoundException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FrontIntake;
import frc.robot.subsystems.InnerIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class MacroAutoPlayback extends CommandBase {
  FrontIntake s_FrontIntake;
  InnerIntake s_InnerIntake;
  Shooter s_Shooter;
  Turret s_Turret;
  
  /**
   * Creates a new MacroAutoPlayback.
   */
  public MacroAutoPlayback(
      FrontIntake s_FrontIntake,
      InnerIntake s_InnerIntake,
      Shooter s_Shooter,
      Turret s_Turret) throws FileNotFoundException{

    addRequirements(s_FrontIntake, s_InnerIntake, s_Shooter, s_Turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
