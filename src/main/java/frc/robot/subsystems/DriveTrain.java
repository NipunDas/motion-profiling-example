// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;

public class DriveTrain extends SubsystemBase {

  private WPI_TalonSRX leftTalon = new WPI_TalonSRX(Constants.leftDrivePort);
  private WPI_TalonSRX rightTalon = new WPI_TalonSRX(Constants.rightDrivePort);
  private WPI_TalonSRX leftSlave = new WPI_TalonSRX(Constants.leftSlavePort);
  private WPI_TalonSRX rightSlave = new WPI_TalonSRX(Constants.rightSlavePort);
  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private double kTrackWidth = 0.607;
  //private double kGearRatio = 1;
  private double kWheelRadius = 0.076;
  private double kTicksInRotation = 4096;
  public double maxVelocity = 2;
  public double maxAcceleration = 2;

  //PID Constants
  private double kS = 0, kV = 0, kA = 0, kP = 0, kI = 0, kD = 0;

  //PID Controllers
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  PIDController leftPIDController = new PIDController(kP, kI, kD);
  PIDController rightPIDController = new PIDController(kP, kI, kD);

  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidth);
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  private Pose2d position = new Pose2d();

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    config(leftTalon, false);
    config(rightTalon, true);
    config(leftSlave, false);
    config(rightSlave, true);
    navx.calibrate();
    navx.reset();
    resetPosition();
  }

  public void config(WPI_TalonSRX talon, boolean inverted) {
    talon.configFactoryDefault();
    talon.setSelectedSensorPosition(0, 0, 10);
    talon.setInverted(inverted);
  }

  public void resetNavx() {
    navx.reset();
  }

  public double getAngle() {
    return navx.getAngle();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-1 * navx.getAngle());
  }

  public void tankDrive(double leftPow, double rightPow) {
    leftTalon.set(ControlMode.PercentOutput, leftPow);
    rightTalon.set(ControlMode.PercentOutput, rightPow);
    //leftSlave.set(ControlMode.PercentOutput, leftPow);
    //rightSlave.set(ControlMode.PercentOutput, rightPow);
  }

  public void setVoltages(double leftVolt, double rightVolt) {
    leftTalon.set(ControlMode.PercentOutput, leftVolt / 12);
    rightTalon.set(ControlMode.PercentOutput, rightVolt / 12);
  }

  public void resetPosition() {
    leftTalon.setSelectedSensorPosition(0, 0, 10);
    rightTalon.setSelectedSensorPosition(0, 0, 10);
    navx.reset();
    odometry.resetPosition(new Pose2d(), new Rotation2d());
  }

  public Pose2d getPosition() {
    return position;
  }

  public double getDistance() {
    //robot distance in meters when driving forward
    return -1 * (leftTalon.getSelectedSensorPosition(0) + rightTalon.getSelectedSensorPosition(0)) * 0.5 * (2 * Math.PI * kWheelRadius) / kTicksInRotation;
  }

  public double getLeftDistance() {
    return -1 * leftTalon.getSelectedSensorPosition(0) * (2 * Math.PI * kWheelRadius) / kTicksInRotation;
  }

  public double getRightDistance() {
    return -1 * rightTalon.getSelectedSensorPosition(0) * (2 * Math.PI * kWheelRadius) / kTicksInRotation;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftTalon.getSensorCollection().getPulseWidthVelocity() * 10 * (2 * Math.PI * kWheelRadius) / kTicksInRotation,
      rightTalon.getSensorCollection().getPulseWidthVelocity() * 10 * (2 * Math.PI * kWheelRadius) / kTicksInRotation
    );
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPIDController;
  }

  public PIDController getRightPIDController() {
    return rightPIDController;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tankDrive(RobotContainer.getLeftJoy().getY(), RobotContainer.getRightJoy().getY());

    //updating robot position
    position = odometry.update(getHeading(),  getLeftDistance(), getRightDistance());
    SmartDashboard.putNumber("x coordinate: ", position.getX());
    SmartDashboard.putNumber("y coordinate: ", position.getY());
    SmartDashboard.putNumber("heading: ", position.getRotation().getDegrees());
    SmartDashboard.putNumber("left distance: ", getLeftDistance());
    SmartDashboard.putNumber("right distance: ", getRightDistance());
  }
}
