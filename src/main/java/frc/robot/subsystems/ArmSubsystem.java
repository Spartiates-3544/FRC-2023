package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX stage1;
    private final WPI_TalonFX stage2;

    public ArmSubsystem() {
        stage1 = new WPI_TalonFX(ArmConstants.stage1Port);
        stage2 = new WPI_TalonFX(ArmConstants.stage2Port);
        configMotors();
    }

    private void configMotors() {
        stage1.configFactoryDefault();
        stage2.configFactoryDefault();

        stage1.setNeutralMode(NeutralMode.Brake);
        stage2.setNeutralMode(NeutralMode.Brake);

        //Config sensor for primary pid
        stage1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.slotIdx, 0);
        stage2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, ArmConstants.slotIdx, 0);

        //TODO
        //Gains for motionmagic
        stage1.selectProfileSlot(0, 0);
        stage1.config_kF(ArmConstants.slotIdx, ArmConstants.stage1F);
        stage1.config_kP(ArmConstants.slotIdx, ArmConstants.stage1P);
        stage1.config_kI(ArmConstants.slotIdx, ArmConstants.stage1I);
        stage1.config_kD(ArmConstants.slotIdx, ArmConstants.stage1D);
        stage1.configMotionCruiseVelocity(ArmConstants.stage1Cruise);
        stage1.configMotionAcceleration(ArmConstants.stage1Accel);

        stage2.config_kF(ArmConstants.slotIdx, ArmConstants.stage2F);
        stage2.config_kP(ArmConstants.slotIdx, ArmConstants.stage2P);
        stage2.config_kI(ArmConstants.slotIdx, ArmConstants.stage2I);
        stage2.config_kD(ArmConstants.slotIdx, ArmConstants.stage2D);
        stage2.configMotionCruiseVelocity(ArmConstants.stage2Cruise);
        stage2.configMotionAcceleration(ArmConstants.stage2Accel);
        //Soft limits
        stage1.configForwardSoftLimitThreshold(ArmConstants.stage1FwdLimit);
        stage1.configReverseSoftLimitThreshold(ArmConstants.stage1RevLimit);
        stage1.configForwardSoftLimitEnable(true);
        stage1.configReverseSoftLimitEnable(true);

        stage2.configForwardSoftLimitThreshold(ArmConstants.stage2FwdLimit);
        stage2.configReverseSoftLimitThreshold(ArmConstants.stage2RevLimit);
        stage2.configForwardSoftLimitEnable(true);
        stage2.configReverseSoftLimitEnable(true);

        //Positive value = Arm goes up
        stage1.setInverted(ArmConstants.stage1Invert);
        stage2.setInverted(ArmConstants.stage2Invert);

    }

    public void setStage1Pourcentage(double pourcentage) {
        stage1.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    public void setStage2Pourcentage(double pourcentage) {
        stage2.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    public void setStage1Pos(double pos) {
        stage1.set(TalonFXControlMode.MotionMagic, pos);
    }

    public void setStage2Pos(double pos) {
        stage2.set(TalonFXControlMode.MotionMagic, pos);
    }

    //Sample command
    public CommandBase setArmGrabConeHP() {
        return this.runOnce(() -> {
            setStage1Pos(ArmConstants.ArmPickupConeHP.stage1Pos); 
            setStage2Pos(ArmConstants.ArmPickupConeHP.stage2Pos);});
    }
}
