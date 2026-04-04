package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Libs.LimelightHelpers;
import frc.robot.Util.Dashboard.HardwareFaultTracker;
import frc.robot.Util.Math.ShooterRegression;

public class Shooter implements Subsystem{
    private final TalonFX shooter = new TalonFX(HardwareConstants.SHOOTER_CAN, HardwareConstants.CANIVORE);
    private final TalonFX shooter2 = new TalonFX(HardwareConstants.SHOOTER_2_CAN, HardwareConstants.CANIVORE);
    private final SparkFlex kicker = new SparkFlex(HardwareConstants.KICKER_CAN, MotorType.kBrushless);
    private final SparkFlex liveFloor = new SparkFlex(HardwareConstants.LIVEFLOOR_CAN, MotorType.kBrushless);
    private final RelativeEncoder kickerRelativeEncoder = kicker.getEncoder();
    private final RelativeEncoder liveFloorRelativeEncoder = liveFloor.getEncoder();
    private final StaticBrake staticBrakeRequest = new StaticBrake();

    private final StatusSignal<AngularVelocity> shooterVelo = shooter.getVelocity();

    private final VelocityVoltage shooterVeloRequest = new VelocityVoltage(0.0)
        .withOverrideBrakeDurNeutral(true)
        .withSlot(0);
    
    private final Alert shooterAlert = new Alert("Shooter Alert", AlertType.kError);
    private final Alert shooter2Alert = new Alert("Shooter 2 Alert", AlertType.kError);
    private final Alert kickerAlert = new Alert("Kicker Alert", AlertType.kError);
    private final Alert liveFloorAlert = new Alert("Live Floor Alert", AlertType.kError);

    private static SysIdRoutine shooterRoutine, kickerRoutine;
    
    // NT Logging 
    private final DoublePublisher kickerVeloPub, shooterVeloPub, shooterSetpointPub, kickerSetPointPub, liveFloorVeloPub, liveFloorSetpointPub;

    //Datalog
    private final DoubleLogEntry shooterVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "shooter/Velo", "RPS");
    
    private final DoubleLogEntry kickerVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Kicker/Velo", "RPM");

    private final DoubleLogEntry liveFloorVeloLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "LiveFloor/Velo", "RPM");

    private final DoubleLogEntry kickerSetPointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Kicker/Setpoint", "RPM");

    private final DoubleLogEntry shooterSetPointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Setpoint", "RPS");
    
    private final DoubleLogEntry liveFLoorSetpointLog = 
        new DoubleLogEntry(DataLogManager.getLog(), "LiveFloor/Setpoint", "RPM");

    private final DoubleLogEntry kickerVoltageLog =
        new DoubleLogEntry(DataLogManager.getLog(), "Kicker/Voltage", "Volts");

    private final DoubleLogEntry kickerPositionLog =
        new DoubleLogEntry(DataLogManager.getLog(), "Kicker/Position", "Rotations");
    


    public Shooter() {
        shooter.getConfigurator().apply(ShooterConstants.SHOOTER_CONFIG);
        kicker.configure(ShooterConstants.KICKER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        liveFloor.configure(ShooterConstants.LIVEFLOOR_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooter2.getConfigurator().apply(ShooterConstants.SHOOTER_CONFIG);
        shooter2.setControl(new Follower(HardwareConstants.SHOOTER_CAN, MotorAlignmentValue.Opposed)); 

        new LimelightHelpers();

        NetworkTable LL = NetworkTableInstance.getDefault().getTable("LLfront");
        LL.getDoubleTopic("LL distance").publish();

        if (BuildConstants.PUBLISH_EVERYTHING){
            NetworkTable nt = NetworkTableInstance.getDefault().getTable("Shooter");
            kickerVeloPub = nt.getDoubleTopic("kickerVelo").publish();
            shooterVeloPub = nt.getDoubleTopic("shooterVelo").publish();
            liveFloorVeloPub = nt.getDoubleTopic("liveFloorVelo").publish();
            kickerSetPointPub = nt.getDoubleTopic("kickerSetpoint").publish();
            shooterSetpointPub = nt.getDoubleTopic("shooterSetpoint").publish();
            liveFloorSetpointPub = nt.getDoubleTopic("liveFloorSetpoint").publish();
        } else {
            kickerVeloPub = null;
            shooterVeloPub = null;
            liveFloorVeloPub = null;
            kickerSetPointPub = null;
            shooterSetpointPub = null;
            liveFloorSetpointPub = null;
        }
    }


    public void setShooterSpeed(double velo) {
        velo = MathUtil.clamp(velo, 0.0, 100); 

        shooter.setControl(
            shooterVeloRequest.withVelocity(velo).withSlot(0)
            );
        shooterSetPointLog.append(velo);

        if (BuildConstants.PUBLISH_EVERYTHING) {
            shooterSetpointPub.set(velo);
        }
    }

    /**
     * 
     * @param velo rpm
     */
    public void setKickerSpeed(double speed) {
        //kickerController.setSetpoint(speed, ControlType.kVelocity);
        kicker.set(speed);

        kickerSetPointLog.append(speed);

        if (BuildConstants.PUBLISH_EVERYTHING){
            kickerSetPointPub.set(speed);
        }
    }

    public void setliveFloorSpeed(double speed) {
        liveFloor.set(speed); 

        liveFLoorSetpointLog.append(speed);

        if(BuildConstants.PUBLISH_EVERYTHING){
            liveFloorSetpointPub.set(speed);
        }
    }

    public void stopShooterSub() {
        shooter.setControl(staticBrakeRequest);
        kicker.disable();
        liveFloor.disable();
    }

    private void setShooterVoltage(double voltage){
        shooter.setVoltage(voltage);
    }

    private void setKickerVoltage(double voltage) {
        kicker.setVoltage(voltage);
    }

    public double getKickerVelocity() {
        return kickerRelativeEncoder.getVelocity();
    }

    public double getShooterVelocity() {
        return shooterVelo.getValueAsDouble();
    }

    public boolean setVelocityFromLimelight() {
        if (LimelightHelpers.getTargetCount(AprilTagConstants.LL_4_FRONT) < 1) {
        shooter.stopMotor();
        return false;
        }

        double ty = LimelightHelpers.getTY(AprilTagConstants.LL_4_FRONT);
        double distance = ShooterRegression.getShooterDistance(ty);
        double targetVel = ShooterRegression.getVelocityRotationsPerSec(distance);

        double targetRPS = targetVel;

        shooter.setControl(shooterVeloRequest.withVelocity(targetRPS));

        // Ready when within ±10 RPS of target
        double currentRPS = shooter.getVelocity().getValueAsDouble();
        return Math.abs(currentRPS - targetRPS) < 10.0;
    }


    public static SysIdRoutine getShooterRoutine(Shooter shooter) {
        System.out.println("Starting CTRE SignalLogger due to getShooterRoutine");
        SignalLogger.start();

        if(shooterRoutine == null){
        shooterRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.50).per(Second),
            Volts.of(6), null, state -> SignalLogger.writeString("SysID_Shooter", state.toString())
        ), new SysIdRoutine.Mechanism((Voltage v) -> {
            shooter.setShooterVoltage(v.in(Volts));
        }, null, shooter) 
        );
    }
        return shooterRoutine;
    }

    public static SysIdRoutine getKickerRoutine(Shooter shooter) {
        System.out.println("Starting URCL due to getKickerRoutine");
        DataLogManager.start();
        URCL.start();

        if (kickerRoutine == null){
        kickerRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(1.0).per(Second),
                Volts.of(6.0),
                Seconds.of(10)), 
            new SysIdRoutine.Mechanism((voltage) -> {
                    shooter.setKickerVoltage(voltage.in(Volts));
                },
                null, shooter));
        }
        return kickerRoutine;
    }

    public void log() {
        shooterVelo.refresh();
        kickerRelativeEncoder.getVelocity();
        liveFloorRelativeEncoder.getVelocity();

        shooterVeloLog.append(shooterVelo.getValueAsDouble());
        kickerVeloLog.append(kickerRelativeEncoder.getVelocity());
        liveFloorVeloLog.append(liveFloorRelativeEncoder.getVelocity());

        kickerVoltageLog.append(kicker.getAppliedOutput());
        kickerPositionLog.append(kickerRelativeEncoder.getPosition());

        if (BuildConstants.PUBLISH_EVERYTHING) {
            shooterVeloPub.set(shooterVelo.getValueAsDouble());
            kickerVeloPub.set(kickerRelativeEncoder.getVelocity());
            liveFloorVeloPub.set(liveFloorRelativeEncoder.getVelocity());
        }
    }

    public void checkHardware() {
        HardwareFaultTracker.checkFault(shooterAlert, !shooter.isAlive() || !shooter.isConnected());
        HardwareFaultTracker.checkFault(shooter2Alert, !shooter2.isAlive() || !shooter2.isConnected());
        HardwareFaultTracker.checkFault(kickerAlert, kicker.hasActiveFault() || kicker.hasActiveWarning());
        HardwareFaultTracker.checkFault(liveFloorAlert, liveFloor.hasActiveFault() || liveFloor.hasActiveWarning());
    }
}
