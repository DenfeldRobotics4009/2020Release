/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerIntake;

/**
 * Phase one will activate when front sensor 
 * is active, and will stop when
 * the back sensor is inactive
 */

public class InnerProccess extends CommandBase {
  public InnerIntake s_Intake;
  /**
   * Creates a new IntakePhaseOne.
   */
  public InnerProccess(InnerIntake i_intake) {
    s_Intake = i_intake;
    addRequirements(s_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.SetInnerIntake(true);
    s_Intake.Start();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.SetInnerIntake(false);
    s_Intake.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !s_Intake.FrontSensorActive();
  }
}
