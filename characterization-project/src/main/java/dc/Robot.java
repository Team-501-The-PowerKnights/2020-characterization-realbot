/**
 * This is a very simple robot program that can be used to send telemetry to
 * the data_logger script to characterize your drivetrain. If you wish to use
 * your actual robot code, you only need to implement the simple logic in the
 * autonomousPeriodic function and change the NetworkTables update rate
 */

package dc;

import java.util.function.Supplier;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// WPI_Talon* imports are needed in case a user has a Pigeon on a Talon
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  static private double WHEEL_DIAMETER = 0.1524;
  static private double GEARING = 10.71;

  private Joystick stick;
  private DifferentialDrive drive;

  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftRearMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightRearMotor;

  private SpeedControllerGroup left;
  private SpeedControllerGroup right;

  private CANEncoder leftEncoder;
  private CANEncoder rightEncoder;

  private AHRS navx;

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  double priorAutospeed = 0;
  Number[] numberArray = new Number[10];

  @Override
  public void robotInit() {
    if (!isReal())
      SmartDashboard.putData(new SimEnabler());

    stick = new Joystick(0);

    leftFrontMotor = new CANSparkMax(11, MotorType.kBrushless);
    leftRearMotor = new CANSparkMax(12, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(13, MotorType.kBrushless);
    rightRearMotor = new CANSparkMax(14, MotorType.kBrushless);

    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftRearMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightRearMotor.setIdleMode(IdleMode.kBrake);

    left = new SpeedControllerGroup(leftFrontMotor, leftRearMotor);
    right = new SpeedControllerGroup(rightFrontMotor, rightRearMotor);

    left.setInverted(false);
    right.setInverted(true);

    leftEncoder = new CANEncoder(leftFrontMotor);

    rightEncoder = new CANEncoder(rightFrontMotor);

    //
    // Configure gyro
    //

    // Note that the angle from the NavX and all implementors of wpilib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    navx = new AHRS(SPI.Port.kMXP);
    gyroAngleRadians = () -> -(Math.toRadians(navx.getAngle()));

    //
    // Configure drivetrain movement
    //

    drive = new DifferentialDrive(left, right);

    drive.setDeadband(0);

    //
    // Configure encoder related functions -- getDistance and getrate should
    // return units and units/s
    //

    final double encoderConstant = (1 / GEARING) * WHEEL_DIAMETER * Math.PI;

    leftEncoderPosition = () -> leftEncoder.getPosition() * encoderConstant;
    leftEncoderRate = () -> leftEncoder.getVelocity() * encoderConstant / 60.;

    rightEncoderPosition = () -> rightEncoder.getPosition() * encoderConstant;
    rightEncoderRate = () -> rightEncoder.getVelocity() * encoderConstant / 60.;

    // Reset encoders
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void disabledInit() {
    System.out.println("Robot disabled");
    drive.tankDrive(0, 0);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void robotPeriodic() {
    // feedback for users, but not used by the control program
    SmartDashboard.putNumber("l_encoder_pos", leftEncoderPosition.get());
    SmartDashboard.putNumber("l_encoder_rate", leftEncoderRate.get());
    SmartDashboard.putNumber("r_encoder_pos", rightEncoderPosition.get());
    SmartDashboard.putNumber("r_encoder_rate", rightEncoderRate.get());
  }

  @Override
  public void teleopInit() {
    System.out.println("Robot in operator control mode");
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-stick.getY(), stick.getX());
  }

  @Override
  public void autonomousInit() {
    System.out.println("Robot in autonomous mode");
  }

  /**
   * If you wish to just use your own robot program to use with the data logging
   * program, you only need to copy/paste the logic below into your code and
   * ensure it gets called periodically in autonomous mode
   *
   * Additionally, you need to set NetworkTables update rate to 10ms using the
   * setUpdateRate call.
   */
  @Override
  public void autonomousPeriodic() {

    // Retrieve values to send back before telling the motors to do something
    final double now = Timer.getFPGATimestamp();

    final double leftPosition = leftEncoderPosition.get();
    final double leftRate = leftEncoderRate.get();

    final double rightPosition = rightEncoderPosition.get();
    final double rightRate = rightEncoderRate.get();

    final double battery = RobotController.getBatteryVoltage();

    final double leftMotorVolts = leftFrontMotor.getBusVoltage() * leftFrontMotor.getAppliedOutput();
    final double rightMotorVolts = rightFrontMotor.getBusVoltage() * rightFrontMotor.getAppliedOutput();

    // Retrieve the commanded speed from NetworkTables
    final double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    drive.tankDrive((rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed, false);

    // send telemetry data array back to NT
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    telemetryEntry.setNumberArray(numberArray);
  }
}