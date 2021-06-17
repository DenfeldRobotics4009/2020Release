/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Constants;
import frc.robot.Libraries.PIDController;


/**
 * Drives the robot. We have 2 NEOS as motors, and the encoders go on the NEOS.
 * @author Nikolai
 */
public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */

  private NetworkTable DriveTrain = NetworkTableInstance.getDefault().getTable("DriveTrain");
  /*
  private NetworkTableEntry LeftGroup = DriveTrain.getEntry("l");
  private NetworkTableEntry RightGroup = DriveTrain.getEntry("r");
  */



  private CANSparkMax left1, right1, left2, right2;
  private CANEncoder leftE, rightE;
  private SpeedControllerGroup l, r;
  private DifferentialDrive drive;
  private AHRS navx;
  private CANPIDController pid1, pid2, pid3, pid4;
  private PIDController locater;

  private double p, i, d;
  public DriveTrain() {
    p = .1;
    i = 0;
    d = .075;
   locater = new PIDController(0.03, 0, 0.05, 0);
    
    left1 = new CANSparkMax(Constants.LeftDrive1MotorPort, MotorType.kBrushless);
    right1 = new CANSparkMax(Constants.RightDrive1MotorPort, MotorType.kBrushless);
    left2 = new CANSparkMax(Constants.leftDrive2MotorPort, MotorType.kBrushless);
    right2 = new CANSparkMax(Constants.RightDrive2MotorPort, MotorType.kBrushless);

    leftE = left1.getEncoder(EncoderType.kHallSensor, 42);
    rightE = right1.getEncoder(EncoderType.kHallSensor, 42);

    pid1 = left1.getPIDController();
    pid2 = left2.getPIDController();
    pid3 = right1.getPIDController();
    pid4 = right2.getPIDController();

    pid1.setP(p);
    pid1.setI(i);
    pid1.setD(d);
    pid1.setIZone(0);
    pid1.setFF(0);
    pid1.setOutputRange(-1, 1);

    pid2.setP(p);
    pid2.setI(i);
    pid2.setD(d);
    pid2.setIZone(0);
    pid2.setFF(0);
    pid2.setOutputRange(-1, 1);
    
    pid3.setP(p);
    pid3.setI(i);
    pid3.setD(d);
    pid3.setIZone(0);
    pid3.setFF(0);
    pid3.setOutputRange(-1, 1);   
    
    pid4.setP(p);
    pid4.setI(i);
    pid4.setD(d);
    pid4.setIZone(0);
    pid4.setFF(0);
    pid4.setOutputRange(-1, 1);

    left1.setOpenLoopRampRate(1);
    left2.setOpenLoopRampRate(1);
    right1.setOpenLoopRampRate(1);
    right2.setOpenLoopRampRate(1);


    l = new SpeedControllerGroup(left1, left2);
    r = new SpeedControllerGroup(right1, right2);

    leftE = left1.getEncoder(EncoderType.kHallSensor, Constants.DriveEncoderResolution);
    rightE = right1.getEncoder(EncoderType.kHallSensor, Constants.DriveEncoderResolution);
  

    drive = new DifferentialDrive(l, r);

    try {
      navx = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantating navX-MXP: " + ex.getMessage(), true);
    }

    navx.reset();

 
    
  }

  @Override
  public void periodic() {

  SmartDashboard.putNumber("Left Encoder", leftE.getPosition());
  SmartDashboard.putNumber("Right Encoder", rightE.getPosition());

    // This method will be called once per scheduler run
  }
  /**
   * Use this method to drive the robot with the input parameters.
   * @param forward the amount of forward/backward motion you want
   * @param twist the amount of twisting left/right you put in.
   */
  public void ArcadeDrive(double forward, double twist) {
    drive.arcadeDrive(forward, twist);
  }

  /**
   * @return the angle the gyro is facing from -180 to 180 degrees.
   */
  public double ReportAngle() {
    return navx.getYaw();
  }

  public double ReportLeftPosition() {
    return leftE.getPosition();
  }

  public double ReportRightPosition() {
    return rightE.getPosition();
  }

  public void SetSidePosition(double target, boolean right){
    locater.setTarget(target);
    locater.setTolerance(0.5);

    if(right) {
      locater.setInput(rightE.getPosition());
      r.set(locater.calculate(1, -1));
    }
    else {
      locater.setInput(leftE.getPosition());
      l.set(locater.calculate(1, -1));
    }
  }

  public Boolean SideOnTarget(double target, Boolean right){
    if(right){
      return Math.abs(rightE.getPosition() - target) < 1;
    }
    else {
      return Math.abs(leftE.getPosition() - target) < 1;
    }
  }
}
