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
    }

    public void periodic() {
        SmartDashboard.putNumber("Manipulator velocity", manipulator.getSelectedSensorVelocity());
    }

    public void setPourcentage(double pourcentage) {
        manipulator.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    public double getOutputCurrent() {
        return manipulator.getStatorCurrent();
    }

}
