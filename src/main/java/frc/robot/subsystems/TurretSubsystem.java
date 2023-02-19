package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase{
    private WPI_TalonFX turret;
    private DigitalInput calibrationSwitch;

    public TurretSubsystem() {
        turret = new WPI_TalonFX(TurretConstants.turretPort);
        calibrationSwitch = new DigitalInput(TurretConstants.calSwitchChannel);
        configMotors();
    }

    public void periodic() {
        SmartDashboard.putNumber("Turret encoder", turret.getSelectedSensorPosition());
    }

    private void configMotors() {
        turret.configFactoryDefault();
        turret.setNeutralMode(NeutralMode.Brake);
        turret.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TurretConstants.pidIdx, 30);
        turret.setInverted(false);
        turret.configNeutralDeadband(TurretConstants.pourcentDeadband);

        turret.selectProfileSlot(TurretConstants.slotIdx, 0);
        turret.config_kF(TurretConstants.slotIdx, TurretConstants.turretF);
        turret.config_kP(TurretConstants.slotIdx, TurretConstants.turretP);
        turret.config_kI(TurretConstants.slotIdx, TurretConstants.turretI);
        turret.config_IntegralZone(TurretConstants.slotIdx, TurretConstants.IZone);
        turret.config_kD(TurretConstants.slotIdx, TurretConstants.turretD);

        turret.configMotionAcceleration(TurretConstants.turretAccel);
        turret.configMotionCruiseVelocity(TurretConstants.turretCruise);
    }


    public void setTurret(double pourcentage) {
        turret.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    public void setMagicSetpoint(double setpoint) {
        turret.set(TalonFXControlMode.MotionMagic, setpoint);
    }

    public void resetEncoder() {
        turret.setSelectedSensorPosition(0);
    }

    public boolean getCalibrationSwitch() {
        return calibrationSwitch.get();
    }

    public double getEncoder() {
        return turret.getSelectedSensorPosition();
    }
}
