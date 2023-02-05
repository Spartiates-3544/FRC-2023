package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

public class TurretSubsystem extends SubsystemBase{
    private WPI_TalonFX turret;

    public TurretSubsystem() {
        turret = new WPI_TalonFX(TurretConstants.turretPort);
        configMotors();
    }

    private void configMotors() {
        turret.configFactoryDefault();
        turret.setNeutralMode(NeutralMode.Brake);
        turret.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, TurretConstants.pidIdx, 0);
        turret.setInverted(false);

        turret.config_kF(TurretConstants.slotIdx, TurretConstants.turretF);
        turret.config_kP(TurretConstants.slotIdx, TurretConstants.turretP);
        turret.config_kI(TurretConstants.slotIdx, TurretConstants.turretI);
        turret.config_kD(TurretConstants.slotIdx, TurretConstants.turretD);

        turret.configMotionAcceleration(TurretConstants.turretAccel);
        turret.configMotionCruiseVelocity(TurretConstants.turretCruise);
    }


    public void setTurret(double pourcentage) {
        turret.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    public void setMagicSetpoint(int setpoint) {
        turret.set(TalonFXControlMode.MotionMagic, setpoint);
    }

    public void resetEncoder() {
        turret.setSelectedSensorPosition(0);
    }
}
