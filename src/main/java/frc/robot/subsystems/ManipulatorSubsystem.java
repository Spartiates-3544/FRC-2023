package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ManipulatorSubsystem extends SubsystemBase{
    private final WPI_TalonFX manipulator;


    public ManipulatorSubsystem() {
        manipulator = new WPI_TalonFX(Constants.ManipulatorConstants.manipulatorPort);
        configMotors();
    }

    private void configMotors() {
        manipulator.configFactoryDefault();

        manipulator.setNeutralMode(NeutralMode.Brake);
        manipulator.setInverted(false);
        manipulator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        manipulator.config_kP(0, Constants.ManipulatorConstants.velocityP);
        manipulator.config_kI(0, Constants.ManipulatorConstants.velocityI);
        manipulator.config_kD(0, Constants.ManipulatorConstants.velocityD);
        manipulator.config_kF(0, Constants.ManipulatorConstants.velocityF);
    }

    public void periodic() {
        SmartDashboard.putNumber("Manipulator velocity", manipulator.getSelectedSensorVelocity());
    }

    public void setPourcentage(double pourcentage) {
        manipulator.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    public void setVelocity(double targetVelocity) {
        //Values in sensor units/100ms
        manipulator.set(TalonFXControlMode.Velocity, targetVelocity);
    }

    public double getOutputCurrent() {
        return manipulator.getStatorCurrent();
    }

}
