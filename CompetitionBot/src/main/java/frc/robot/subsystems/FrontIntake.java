/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FrontIntake extends SubsystemBase {
  /**
   * Creates a new FrontIntake.
   */
  private NetworkTable FrontIntake = NetworkTableInstance.getDefault().getTable("FrontIntake");

  public WPI_TalonSRX intakemotor;

  public FrontIntake() {
    intakemotor = new WPI_TalonSRX(Constants.IntakeMotorPort);

    intakemotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void frontIntake() {
    intakemotor.set(0.5);
  }

  public void stopIntake() {
    intakemotor.set(0);
  }

  public void frontOuttake() {
    intakemotor.set(-0.6);
  }
}
