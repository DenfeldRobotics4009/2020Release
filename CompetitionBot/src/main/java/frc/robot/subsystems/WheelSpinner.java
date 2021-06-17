/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class WheelSpinner extends SubsystemBase {
  /**
   * Creates a new WheelSpinner.
   */
  private final WPI_TalonSRX controllpanel;

  public WheelSpinner() {
    controllpanel = new WPI_TalonSRX(Constants.WOFMotorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * spins the wheel that spins the WOF to a speed you specify
   * @param speed how fast you want the motherf***** spinner to go
   */
  public void ManualSpin(double speed) {
    controllpanel.set(speed);

  }
}
