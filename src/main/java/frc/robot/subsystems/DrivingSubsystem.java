/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DrivingSubsystem extends SubsystemBase {
  
  public static WPI_TalonFX frontLeftMotor = new WPI_TalonFX(Constants.kFrontLeft);
  public static WPI_TalonFX rearLeftMotor = new WPI_TalonFX(Constants.kRearLeft);
  public static WPI_TalonFX frontRightMotor = new WPI_TalonFX(Constants.kFrontRight);
  public static WPI_TalonFX rearRightMotor = new WPI_TalonFX(Constants.kRearRight);

  public static MecanumDrive drive; 

  private final float MIN_MOTOR_SPEED = 0.15f;
  private final float MIN_CONTROLLER_SPEED = 0.20f;
  private final float SMALLEST_RADIO = 0.1f;
  



  /**
   * Creates a new DriveTrain.
   */
  public DrivingSubsystem() {
    super();
    frontLeftMotor.setInverted(true);
    frontRightMotor.setInverted(true);
    drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
  }

  public void drive(double xAxisLeft, double yAxisLeft, double xAxisRight) {
    if(Math.abs(xAxisRight)<MIN_CONTROLLER_SPEED){
      xAxisRight = 0;
    }

    if(Math.abs(xAxisLeft)<MIN_CONTROLLER_SPEED){
      xAxisLeft = 0;
    }

    if(Math.abs(yAxisLeft)<MIN_CONTROLLER_SPEED){
      xAxisLeft = 0;
    }

    double forceLeft = yAxisLeft;
    double forceRight = yAxisLeft;

    double finalRatio = 1;


    if(finalRatio < SMALLEST_RADIO){
        finalRatio = SMALLEST_RADIO;
    }

    forceLeft += (xAxisRight/3)*2;
    forceRight += (xAxisRight/3)*2;

    double addFL = xAxisLeft;

    double addRL = -xAxisLeft;

    double addFR = -xAxisLeft;

    double addRR = xAxisLeft;


    forceLeft *= finalRatio;
    forceRight *= finalRatio;

    addFL *= finalRatio;
    addRL *= finalRatio;
    addFR *= finalRatio;
    addRR *= finalRatio;

    forceLeft *= 0.95f;
    forceRight *= 0.95f;

    addFL *= 0.95f;
    addRL *= 0.95f;
    addFR *= 0.95f;
    addRR *= 0.95f;

    if(Math.abs(yAxisLeft) > MIN_MOTOR_SPEED ||
      Math.abs(xAxisRight) > MIN_MOTOR_SPEED ||
      Math.abs(xAxisLeft) > MIN_MOTOR_SPEED) {
        frontLeftMotor.set(ControlMode.PercentOutput, (forceLeft + addFL));
        rearLeftMotor.set(ControlMode.PercentOutput, (forceLeft + addRL));

        frontRightMotor.set(ControlMode.PercentOutput, -(forceRight + addFR));
        rearRightMotor.set(ControlMode.PercentOutput, -(forceRight + addRR));
      } else {
        frontLeftMotor.set(ControlMode.PercentOutput, 0);
        rearLeftMotor.set(ControlMode.PercentOutput, 0);

        frontRightMotor.set(ControlMode.PercentOutput, 0);
        rearRightMotor.set(ControlMode.PercentOutput, 0);
      }

    // final double magnitude = Math.hypot(xSpeed, ySpeed);
    // final double robotAngle = Math.atan2(xSpeed, ySpeed) - Math.PI / 4.0;
    // final double rightX = strafing;

    // final double fld = magnitude * Math.cos(robotAngle) + rightX;
    // final double frd = magnitude * Math.sin(robotAngle) - rightX;
    // final double rld = magnitude * Math.sin(robotAngle) + rightX;
    // final double rrd = magnitude * Math.cos(robotAngle) - rightX;

    // frontLeftMotor.set(fld);
    // frontRightMotor.set(frd);
    // rearLeftMotor.set(rld);
    // rearRightMotor.set(rrd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
