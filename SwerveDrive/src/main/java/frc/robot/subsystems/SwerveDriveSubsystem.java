// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class SwerveDriveSubsystem extends SubsystemBase {
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is
   * useful during initial testing of the robot.
   */
  public static final double maxVoltage = 12.0;
  // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
  // The formula for calculating the theoretical maximum velocity is:
  // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
  // pi
  // By default this value is setup for a Mk3 standard module using Falcon500s to
  // drive.
  // An example of this constant for a Mk4 L2 module with NEOs to drive is:
  // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
  // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight
   * line.
   */
  public static final double maxVelocity = 6380.0 / 60.0 *
      SdsModuleConfigurations.MK4I_L1.getDriveReduction() *
      SdsModuleConfigurations.MK4I_L1.getWheelDiameter() * Math.PI;
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also
  // replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = maxVelocity /
      Math.hypot(DriveConstants.trackwidth / 2.0, DriveConstants.wheelbase / 2.0);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DriveConstants.trackwidth / 2.0,
          DriveConstants.wheelbase / 2.0),
      // Front right
      new Translation2d(DriveConstants.trackwidth / 2.0,
          -DriveConstants.wheelbase / 2.0),
      // Back left
      new Translation2d(-DriveConstants.trackwidth / 2.0,
          DriveConstants.wheelbase / 2.0),
      // Back right
      new Translation2d(-DriveConstants.trackwidth / 2.0,
          -DriveConstants.wheelbase / 2.0));

  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 400); // NavX connected over MXP

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public SwerveDriveSubsystem() {
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        // This parameter is optional, but will allow you to see the current state of
        // the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk4iSwerveModuleHelper.GearRatio.L1,
        DriveConstants.FrontLeft.DriveMotorFL,
        DriveConstants.FrontLeft.SteerMotorFL,
        DriveConstants.FrontLeft.SteerEncoderFL,
        DriveConstants.FrontLeft.SteerOffsetFL);

    // We will do the same for the other modules
    frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        DriveConstants.FrontRight.DriveMotorFR,
        DriveConstants.FrontRight.SteerMotorFR,
        DriveConstants.FrontRight.SteerEncoderFR,
        DriveConstants.FrontRight.SteerOffsetFR);

    backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        DriveConstants.BackLeft.DriveMotorBL,
        DriveConstants.BackLeft.SteerMotorBL,
        DriveConstants.BackLeft.SteerEncoderBL,
        DriveConstants.BackLeft.SteerOffsetBL);

    backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        Mk4iSwerveModuleHelper.GearRatio.L1,
        DriveConstants.BackRight.DriveMotorBR,
        DriveConstants.BackRight.SteerMotorBR,
        DriveConstants.BackRight.SteerEncoderBR,
        DriveConstants.BackRight.SteerOffsetBR);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    // FIXME Uncomment if you are using a NavX
    m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {
    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }
    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.maxVelocity);

    frontLeftModule.set(
        states[0].speedMetersPerSecond / DriveConstants.maxVelocity * DriveConstants.maxVoltage,
        states[0].angle.getRadians());
    frontRightModule.set(
        states[1].speedMetersPerSecond / DriveConstants.maxVelocity * DriveConstants.maxVoltage,
        states[1].angle.getRadians());
    backLeftModule.set(
        states[2].speedMetersPerSecond / DriveConstants.maxVelocity * DriveConstants.maxVoltage,
        states[2].angle.getRadians());
    backRightModule.set(
        states[3].speedMetersPerSecond / DriveConstants.maxVelocity * DriveConstants.maxVoltage,
        states[3].angle.getRadians());
  }
}
