
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.cameraserver.CameraServer;

public class ShooterSubsystem extends SubsystemBase {

  /** Creates a new Shooter. */

  // Motor Controllers
  private CANSparkMax LeftShooterMotor = new CANSparkMax(Constants.ShooterConstants.kLeftShooterMotorCanId,
      MotorType.kBrushless);
  private CANSparkMax RightShooterMotor = new CANSparkMax(Constants.ShooterConstants.kRightShooterMotorCanId,
      MotorType.kBrushless);

  // stores the speed of the Shooter motor
  private float LeftShooterSpeed = 0.70f;
  private float RightShooterSpeed = 0.70f;
  // private float lowSpeed = 0.65f;

  public ShooterSubsystem() {
    // Reset the motors
    LeftShooterMotor.restoreFactoryDefaults();
    RightShooterMotor.restoreFactoryDefaults();

    // Sets the right side motors to be inverted
    LeftShooterMotor.setInverted(false);
    RightShooterMotor.setInverted(true);

    // Sets idle mode of the motor controllers to brake mode
    LeftShooterMotor.setIdleMode(ShooterConstants.kLeftShooterIdleMode);
    RightShooterMotor.setIdleMode(ShooterConstants.kRightShooterIdleMode);

    CameraServer.startAutomaticCapture();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    int num = (int) (LeftShooterSpeed * 100);
    String percent = String.valueOf(num);
    SmartDashboard.putString("Shooter Motors Speed", percent + "%");
    // SmartDashboard.putNumber("Average Shooter Encoder Position",
    // getAverageShooterEncoderDistance());

  }

  public void startShooter() {
    LeftShooterMotor.set(LeftShooterSpeed);
    RightShooterMotor.set(RightShooterSpeed);
    // ShooterMotor2.set(ShooterRingSpeed);
  }

  public void stopShooter() {
    LeftShooterMotor.set(0.0);
    RightShooterMotor.set(0.0);
  }
}
