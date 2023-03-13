package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final WPI_TalonFX stage1;
    private final WPI_TalonFX stage2;
    private final WPI_CANCoder CANCoder1;
    private final WPI_CANCoder CANCoder2;

    public ArmSubsystem() {
        stage1 = new WPI_TalonFX(ArmConstants.stage1Port);
        stage2 = new WPI_TalonFX(ArmConstants.stage2Port);
        CANCoder1 = new WPI_CANCoder(Constants.ArmConstants.CANCoder1Port);
        CANCoder2 = new WPI_CANCoder(Constants.ArmConstants.CANCoder2Port);

        configMotors();
        configEncoders();
    }

    public void periodic() {
        //SmartDashboard.putNumber("Stage 1 CANcoder Absolute", CANCoder1.getAbsolutePosition());
        SmartDashboard.putNumber("Stage 2 CANcoder Absolute", CANCoder2.getAbsolutePosition());
        /*
        SmartDashboard.putNumber("Stage 2 CANCoder Relative", CANCoder2.getPosition());
        SmartDashboard.putNumber("Stage 2 Falcon", stage2.getSelectedSensorPosition());
        SmartDashboard.putNumber("Stage 1 output", stage1.get());
        SmartDashboard.putNumber("Stage 2 output", stage2.get());
        */

        SmartDashboard.putNumber("1 Pourcentage", stage1.getMotorOutputPercent());
        SmartDashboard.putNumber("2 Pourcentage", stage2.getMotorOutputPercent());

        SmartDashboard.putNumber("Encodeur 1", stage1.getSelectedSensorPosition());
        SmartDashboard.putNumber("Encodeur 2", stage2.getSelectedSensorPosition());

        //Set Relative to Absolute for Motion Magic (??)
        CANCoder1.setPositionToAbsolute(30);
        CANCoder2.setPositionToAbsolute(30);
    }

    private void configEncoders() {
        CANCoder1.configFactoryDefault();
        CANCoder2.configFactoryDefault();

        CANCoder1.configSensorDirection(true, 30);
        CANCoder2.configSensorDirection(false, 30);


        CANCoder1.configFeedbackCoefficient(0.06591797, "deg", SensorTimeBase.PerSecond);
        CANCoder1.configMagnetOffset(40);

        CANCoder2.configMagnetOffset(-253);
    }

    private void configMotors() {
        stage1.configFactoryDefault();
        stage2.configFactoryDefault();

        stage1.setNeutralMode(NeutralMode.Coast);
        stage2.setNeutralMode(NeutralMode.Coast);

        stage1.configNeutralDeadband(ArmConstants.pourcentageDeadband);
        stage2.configNeutralDeadband(ArmConstants.pourcentageDeadband);

        stage1.setSensorPhase(true);
        stage2.setSensorPhase(true);

        //Config sensor for primary pid
        //stage1.configSelectedFeedbackSensor(RemoteSensorSource.CANCoder, ArmConstants.slotIdx, 30);
        //stage2.configSelectedFeedbackSensor(RemoteSensorSource.CANCoder, ArmConstants.slotIdx, 30);
        stage1.configRemoteFeedbackFilter(CANCoder1.getDeviceID(), RemoteSensorSource.CANCoder, 0);
        stage2.configRemoteFeedbackFilter(CANCoder2.getDeviceID(), RemoteSensorSource.CANCoder, 1);
        stage1.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, ArmConstants.slotIdx, 30);
        stage2.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor1, ArmConstants.slotIdx, 30);


        //Gains for motionmagic
        stage1.selectProfileSlot(0, 0);
        stage1.config_kF(ArmConstants.slotIdx, ArmConstants.stage1F);
        stage1.config_kP(ArmConstants.slotIdx, ArmConstants.stage1P);
        stage1.config_kI(ArmConstants.slotIdx, ArmConstants.stage1I);
        stage1.config_IntegralZone(ArmConstants.slotIdx, ArmConstants.IZone1);
        stage1.config_kD(ArmConstants.slotIdx, ArmConstants.stage1D);
        stage1.configMotionCruiseVelocity(ArmConstants.stage1Cruise);
        stage1.configMotionAcceleration(ArmConstants.stage1Accel);

        stage2.selectProfileSlot(0, 0);
        stage2.config_kF(ArmConstants.slotIdx, ArmConstants.stage2F);
        stage2.config_kP(ArmConstants.slotIdx, ArmConstants.stage2P);
        stage2.config_kI(ArmConstants.slotIdx, ArmConstants.stage2I);
        stage2.config_IntegralZone(ArmConstants.slotIdx, ArmConstants.IZone2);
        stage2.config_kD(ArmConstants.slotIdx, ArmConstants.stage2D);
        stage2.configMotionCruiseVelocity(ArmConstants.stage2Cruise);
        stage2.configMotionAcceleration(ArmConstants.stage2Accel);
        //Soft limits
        /*/
        stage1.configForwardSoftLimitThreshold(ArmConstants.stage1FwdLimit);
        stage1.configReverseSoftLimitThreshold(ArmConstants.stage1RevLimit);
        stage1.configForwardSoftLimitEnable(true);
        stage1.configReverseSoftLimitEnable(true);

        stage2.configForwardSoftLimitThreshold(ArmConstants.stage2FwdLimit);
        stage2.configReverseSoftLimitThreshold(ArmConstants.stage2RevLimit);
        stage2.configForwardSoftLimitEnable(true);
        stage2.configReverseSoftLimitEnable(true);
        */

        //Positive value = Arm goes up
        stage1.setInverted(ArmConstants.stage1Invert);
        stage2.setInverted(ArmConstants.stage2Invert);

    }

    public void setStage1Pourcentage(double pourcentage) {
        stage1.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    public double getStage1AngleRadians() {
        int kMeasuredAngleHorizontal = 154;
        double radians = Math.toRadians(CANCoder1.getAbsolutePosition() - kMeasuredAngleHorizontal);
        return radians;
    }

    public double getStage2AngleRadians() {
        int kMeasuredAngleHorizontal = 91;
        double radians = Math.toRadians(CANCoder2.getAbsolutePosition() - kMeasuredAngleHorizontal);
        return radians;
    }

    public void setStage2Pourcentage(double pourcentage) {
        stage2.set(TalonFXControlMode.PercentOutput, pourcentage);
    }

    public void setStage1Pos(double pos) {
        double feedforward = ArmConstants.stage1StallPourcentage1 * Math.cos(getStage1AngleRadians() + (ArmConstants.stage1StallPourcentage2 - ArmConstants.stage1StallPourcentage1) * Math.cos(getStage2AngleRadians()));
        SmartDashboard.putNumber("Stage1 Ff", feedforward);
        stage1.set(TalonFXControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward, feedforward);

    }

    public void setStage2Pos(double pos) {
        double feedforward = Math.cos(getStage2AngleRadians()) * ArmConstants.stage2StallPourcentage;
        SmartDashboard.putNumber("Stage2 Ff", feedforward);
        stage2.set(TalonFXControlMode.MotionMagic, pos, DemandType.ArbitraryFeedForward, feedforward);
    }

    /*
    public boolean stage1AtSetpoint() {
        return Utilities.inRange(stage1.getActiveTrajectoryPosition() - 50, stage1.getActiveTrajectoryPosition() + 50, stage1.getSelectedSensorPosition());
    }

    public boolean stage2AtSetpoint() {
        return Utilities.inRange(stage2.getActiveTrajectoryPosition() - 50, stage2.getActiveTrajectoryPosition() + 50, stage2.getSelectedSensorPosition());
    }
    */

    public double getStage1Encoder() {
        return stage1.getSelectedSensorPosition();
    }

    public double getStage2Encoder() {
        return stage2.getSelectedSensorPosition();
    }

    public void resetStage1Encoder() {
        stage1.setSelectedSensorPosition(0);
    }

    public void resetStage2Encoder() {
        stage2.setSelectedSensorPosition(0);
    }

    /*
    //Sample command
    public CommandBase setArmGrabConeHP() {
        return this.runOnce(() -> {
            setStage1Pos(ArmConstants.ArmPickupConeHP.stage1Pos); 
            setStage2Pos(ArmConstants.ArmPickupConeHP.stage2Pos);});
    }
    */
}
