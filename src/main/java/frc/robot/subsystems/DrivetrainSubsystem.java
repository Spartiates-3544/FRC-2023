package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import frc.robot.Constants.DriveConstants;

public class DrivetrainSubsystem extends SubsystemBase{
    private final WPI_TalonFX left1;
    private final WPI_TalonFX left2;
    private final WPI_TalonFX right1;
    private final WPI_TalonFX right2;
    private final MotorControllerGroup left;
    private final MotorControllerGroup right;
    private final DifferentialDrive drive;
    private final WPI_PigeonIMU gyro;
    private final DifferentialDriveOdometry odometry;
    private final Field2d field;
    private final SlewRateLimiter ramp;
    private boolean rampEnable;
    

    public DrivetrainSubsystem() {
        left1 = new WPI_TalonFX(DriveConstants.left1Port);
        left2 = new WPI_TalonFX(DriveConstants.left2Port);
        right1 = new WPI_TalonFX(DriveConstants.right1Port);
        right2 = new WPI_TalonFX(DriveConstants.right2Port);
        gyro = new WPI_PigeonIMU(DriveConstants.gyroPort);
        configMotors();

        left = new MotorControllerGroup(left1, left2);
        right = new MotorControllerGroup(right1, right2);

        drive = new DifferentialDrive(left, right);

        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), getLeftSideMeters(), getRightSideMeters());

        field = new Field2d();

        ramp = new SlewRateLimiter(DriveConstants.ramp);
        rampEnable = false;
        SmartDashboard.putData("Field", field);
    }

    private void configMotors() {
        left1.configFactoryDefault(0);
        left2.configFactoryDefault(0);
        right1.configFactoryDefault(0);
        right2.configFactoryDefault(0);

        left1.setNeutralMode(NeutralMode.Coast);
        left2.setNeutralMode(NeutralMode.Coast);
        right1.setNeutralMode(NeutralMode.Coast);
        right2.setNeutralMode(NeutralMode.Coast);

        right1.setInverted(true);
        right2.setInverted(true);
        left1.setInverted(false);
        left2.setInverted(false);
        
        left1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        left2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        right1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        right2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    //TODO Check if odometry is accurate and working, also check for Field2d
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getLeftSideMeters(), getRightSideMeters());
        field.setRobotPose(odometry.getPoseMeters());
        
        SmartDashboard.putData("Gyro angle", gyro);
        /*
        SmartDashboard.putNumber("Left drive meters", getLeftSideMeters());
        SmartDashboard.putNumber("Right drive meters", getRightSideMeters());
        */
        SmartDashboard.putNumber("Left drive velocity", getLeftSideVelocityMetersPerSecond());
        SmartDashboard.putNumber("Right drive velocity", getRightSideVelocityMetersPerSecond());
        SmartDashboard.putNumber("Left side output", left1.getMotorOutputVoltage());
        SmartDashboard.putNumber("Right side output", right1.getMotorOutputVoltage());

    }

    public void enableRamp(boolean enable) {
        rampEnable = enable;
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        if (rampEnable) {
            drive.arcadeDrive(ramp.calculate(xSpeed), zRotation);
        }
        if (!rampEnable) {
            drive.arcadeDrive(xSpeed, zRotation);
        }
    }

    public void resetEncoders() {
        left1.setSelectedSensorPosition(0);
        left2.setSelectedSensorPosition(0);
        right1.setSelectedSensorPosition(0);
        right2.setSelectedSensorPosition(0);
    }
    /*
    public double getLeftSideMeters() {
        return ( (left1.getSelectedSensorPosition() + left2.getSelectedSensorPosition()) / 2 ) * DriveConstants.kEncoderDistancePerPulseMeters; 
    }

    public double getRightSideMeters() {
        return ( (right1.getSelectedSensorPosition() + right2.getSelectedSensorPosition()) / 2 ) * DriveConstants.kEncoderDistancePerPulseMeters; 
    }
    */

    public double getLeftSideMeters() {
        return (((left1.getSelectedSensorPosition() / 2048) / 6) * 0.478778720407);
    }

    public double getRightSideMeters() {
        return (((right1.getSelectedSensorPosition() / 2048) / 6) * 0.478778720407);
    }

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /*
    public double getLeftSideVelocityMetersPerSecond() {
        double avgTicksPerSecond = ( (left1.getSelectedSensorVelocity() + left2.getSelectedSensorVelocity()) / 2 ) * 10; 
        return avgTicksPerSecond * DriveConstants.kEncoderDistancePerPulseMeters;
    }

    public double getRightSideVelocityMetersPerSecond() {
        double avgTicksPerSecond = ( (right1.getSelectedSensorVelocity() + right2.getSelectedSensorVelocity()) / 2 ) * 10; 
        return avgTicksPerSecond * DriveConstants.kEncoderDistancePerPulseMeters;
    }
    */

    public double getLeftSideVelocityMetersPerSecond() {
        return ((((left1.getSelectedSensorVelocity() * 10) / 2048) / 6) / 0.478778720407);
    }

    public double getRightSideVelocityMetersPerSecond() {
        return ((((right1.getSelectedSensorVelocity() * 10) / 2048) / 6) / 0.478778720407);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftSideVelocityMetersPerSecond(), getRightSideVelocityMetersPerSecond());
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void followVelocites(double left, double right) {
        
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        left.setVoltage(leftVolts);
        right.setVoltage(rightVolts);
        drive.feed();
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        gyro.reset();
        odometry.resetPosition(gyro.getRotation2d(), getLeftSideMeters(), getRightSideMeters(), pose);
    }

}
