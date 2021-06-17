/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Libraries.PIDController;

public class Shooter extends SubsystemBase {
private double flywheelSpeed;
private NetworkTable Shooter = NetworkTableInstance.getDefault().getTable("Shooter");

private PIDController VControl;

private WPI_TalonSRX TopShooter = new WPI_TalonSRX(Constants.Shoot1MotorPort);
private WPI_TalonSRX BottomShooter = new WPI_TalonSRX(Constants.Shoot2MotorPort);

  /*
   * Creates a new Shooter.
   */
  public Shooter() {
    BottomShooter.configFactoryDefault();

    BottomShooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    BottomShooter.setSensorPhase(true);
    BottomShooter.configFeedbackNotContinuous(true, Constants.kTimeoutMs);
    BottomShooter.setInverted(true);

    TopShooter.setInverted(true);
    BottomShooter.setInverted(true);


    VControl = new PIDController(0.00001, 0, .00001, 0);

    VControl.setTarget(Constants.DesiredFlyWheelSpeed);
    VControl.setTolerance(0);

  }

  @Override 
  public void periodic() {

    flywheelSpeed = BottomShooter.getSelectedSensorVelocity();

    SmartDashboard.putNumber("FlyWheel Speed", flywheelSpeed);
   
    // This method will be called once per scheduler run
  }
  /**
  * Will trigger the robot to run both shooting motors
  */
 public void Shoot() { 
  TopShooter.set(-1);
  BottomShooter.set(-1);
 }

 public void ControlledShoot() {
   VControl.setInput(flywheelSpeed);
   TopShooter.set(-VControl.calculate(1,-1) -.88);
   BottomShooter.set(-VControl.calculate(1,-1) -.88);

 }
 public Boolean FastEnough() {
   return flywheelSpeed >= Constants.DesiredFlyWheelSpeed - 500;
 }
 public Boolean TooSlow() {
   return flywheelSpeed <= Constants.DesiredFlyWheelSpeed - 500;
 }
 /**
  * Will end the shooting
  */
 public void StopShoot() {
  TopShooter.set(0);
  BottomShooter.set(0);
 }

}
