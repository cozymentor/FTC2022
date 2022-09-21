package org.firstinspires.ftc.teamcode.CoveMecanum;

import static java.util.Objects.requireNonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.CoveMecanum.MathUtil;

public class MecanumDrive {
    private static int instances;

     private final DcMotor m_frontLeftMotor;
     private final DcMotor m_rearLeftMotor;
     private final DcMotor m_frontRightMotor;
     private final DcMotor m_rearRightMotor;
     public static final double kDefaultDeadband = 0.02;
     public static final double kDefaultMaxOutput = 1.0;
     protected double m_deadband = kDefaultDeadband;
     protected double m_maxOutput = kDefaultMaxOutput;
     private boolean m_reported;

/**
* Wheel speeds for a mecanum drive.
*
* <p>Uses normalized voltage [-1.0..1.0].
*/
@SuppressWarnings("MemberName")
    public static class WheelSpeeds {
    public double frontLeft;
    public double frontRight;
    public double rearLeft;
    public double rearRight;

/** Constructs a WheelSpeeds with zeroes for all four speeds. */
public WheelSpeeds() {}

/**
* Constructs a WheelSpeeds.
*
* @param frontLeft The front left speed [-1.0..1.0].
* @param frontRight The front right speed [-1.0..1.0].
* @param rearLeft The rear left speed [-1.0..1.0].
* @param rearRight The rear right speed [-1.0..1.0].
*/
    public WheelSpeeds(double frontLeft, double frontRight, double rearLeft, double rearRight) {
      this.frontLeft = frontLeft;
      this.frontRight = frontRight;
      this.rearLeft = rearLeft;
      this.rearRight = rearRight;
    }
  }

/**
* Construct a MecanumDrive.
*
* <p>If a motor needs to be inverted, do so before passing it in.
*
* @param frontLeftMotor The motor on the front-left corner.
* @param rearLeftMotor The motor on the rear-left corner.
* @param frontRightMotor The motor on the front-right corner.
* @param rearRightMotor The motor on the rear-right corner.
*/
  public MecanumDrive(
      DcMotor frontLeftMotor,
      DcMotor rearLeftMotor,
      DcMotor frontRightMotor,
      DcMotor rearRightMotor) {
      requireNonNull(frontLeftMotor, "Front-left motor cannot be null");
      requireNonNull(rearLeftMotor, "Rear-left motor cannot be null");
      requireNonNull(frontRightMotor, "Front-right motor cannot be null");
      requireNonNull(rearRightMotor, "Rear-right motor cannot be null");

      m_frontLeftMotor = frontLeftMotor;
      m_rearLeftMotor = rearLeftMotor;
      m_frontRightMotor = frontRightMotor;
      m_rearRightMotor = rearRightMotor;

      instances++;

  }
    @SuppressWarnings("ParameterName")
    /**
     095   * Normalize all wheel speeds if the magnitude of any wheel is greater than 1.0.
     096   *
     097   * @param wheelSpeeds List of wheel speeds to normalize.
     098   */
  protected static void normalize(double[] wheelSpeeds) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        if (maxMagnitude > 1.0) {
            for (int i = 0; i < wheelSpeeds.length; i++) {
                wheelSpeeds[i] = wheelSpeeds[i] / maxMagnitude;
            }
        }
    }
/**
* Drive method for Mecanum platform.
*
* <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
* from its angle or rotation rate.
*
* @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Forward is positive.
* @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Right is positive.
* @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
*     positive.
* @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use this
*     to implement field-oriented controls.
*/
  @SuppressWarnings("ParameterName")
  public void driveCartesian(double ySpeed, double xSpeed, double zRotation, double gyroAngle) {

            ySpeed = MathUtil.applyDeadband(ySpeed, m_deadband);
            xSpeed = MathUtil.applyDeadband(xSpeed, m_deadband);

            WheelSpeeds speeds = driveCartesianIK(ySpeed, xSpeed, zRotation, gyroAngle);

            m_frontLeftMotor.setPower(speeds.frontLeft * m_maxOutput);
            m_frontRightMotor.setPower(speeds.frontRight * m_maxOutput);
            m_rearLeftMotor.setPower(speeds.rearLeft * m_maxOutput);
            m_rearRightMotor.setPower(speeds.rearRight * m_maxOutput);


          }

    /**
     213   * Cartesian inverse kinematics for Mecanum platform.
     214   *
     215   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
     216   * from its angle or rotation rate.
     217   *
     218   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Forward is positive.
     219   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Right is positive.
     220   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     221   *     positive.
     222   * @return Wheel speeds [-1.0..1.0].
     223   */
  @SuppressWarnings("ParameterName")
  public static WheelSpeeds driveCartesianIK(double ySpeed, double xSpeed, double zRotation) {
    return driveCartesianIK(ySpeed, xSpeed, zRotation, 0.0);
  }

  /**
 230   * Cartesian inverse kinematics for Mecanum platform.
 231   *
 232   * <p>Angles are measured clockwise from the positive X axis. The robot's speed is independent
 233   * from its angle or rotation rate.
 234   *
 235   * @param ySpeed The robot's speed along the Y axis [-1.0..1.0]. Forward is positive.
 236   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Right is positive.
 237   * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
 238   *     positive.
 239   * @param gyroAngle The current angle reading from the gyro in degrees around the Z axis. Use this
 240   *     to implement field-oriented controls.
 241   * @return Wheel speeds [-1.0..1.0].
 242   */
  @SuppressWarnings("ParameterName")
  public static WheelSpeeds driveCartesianIK(
      double ySpeed, double xSpeed, double zRotation, double gyroAngle) {
        ySpeed = MathUtil.clamp(ySpeed, -1.0, 1.0);
            xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);

            // Compensate for gyro angle.
            Vector2d input = new Vector2d(ySpeed, xSpeed);
            input.rotate(-gyroAngle);

            double[] wheelSpeeds = new double[4];
            wheelSpeeds[0] = input.x + input.y + zRotation;
            wheelSpeeds[1] = input.x - input.y - zRotation;
            wheelSpeeds[2] = input.x - input.y + zRotation;
            wheelSpeeds[3] = input.x + input.y - zRotation;

            normalize(wheelSpeeds);

            return new WheelSpeeds(
                        wheelSpeeds[0],
                        wheelSpeeds[1],
                        wheelSpeeds[2],
                        wheelSpeeds[3]);
          }
}
