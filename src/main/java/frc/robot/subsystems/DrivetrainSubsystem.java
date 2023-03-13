package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
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

    public void periodic() {
        SmartDashboard.putData("Angle gyro", gyro);
    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
        drive.feed();
    }

    public void resetEncoders() {
        left1.setSelectedSensorPosition(0);
        left2.setSelectedSensorPosition(0);
        right1.setSelectedSensorPosition(0);
        right2.setSelectedSensorPosition(0);
    }

    public double getAverageLeftEncoders() {
        return (left1.getSelectedSensorPosition() + left2.getSelectedSensorPosition()) / 2; 
    }

    public double getAverageRightEncoders() {
        return (right1.getSelectedSensorPosition() + right2.getSelectedSensorPosition()) / 2; 
    }

    public double getAverageEncoders() {
        return (getAverageLeftEncoders() + getAverageRightEncoders()) / 2;
    }

    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

}
