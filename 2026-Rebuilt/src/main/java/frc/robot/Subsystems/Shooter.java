package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter implements Subsystem{
    private final TalonFX shooter = new TalonFX(HardwareConstants.SHOOTER_CAN, HardwareConstants.CANIVORE);
    private final TalonFX shooter2 = new TalonFX(HardwareConstants.SHOOTER_2_CAN, HardwareConstants.CANIVORE);
    private final SparkFlex kicker = new SparkFlex(HardwareConstants.KICKER_CAN, MotorType.kBrushless);
    private final RelativeEncoder kickerRelativeEncoder = kicker.getEncoder();
    
    private final StaticBrake staticBrakeRequest = new StaticBrake();

    private final StatusSignal<AngularVelocity> shooterVelo = shooter.getVelocity();

    private final VelocityVoltage shooterVeloRequest = new VelocityVoltage(0.0)
        .withOverrideBrakeDurNeutral(true)
        .withSlot(0);

    private static SysIdRoutine shooterRoutine;
    
        // NT Logging 
        private final DoublePublisher kickerVeloPub, shooterVeloPub, shooterSetpointPub, kickerSetPointPub;
    
        //Datalog
        private final DoubleLogEntry shooterVeloLog = 
            new DoubleLogEntry(DataLogManager.getLog(), "shooter/Velo", "RPS");
        
        private final DoubleLogEntry kickerVeloLog = 
            new DoubleLogEntry(DataLogManager.getLog(), "Kicker/Velo", "RPS");
    
        private final DoubleLogEntry kickerSetPointLog = 
            new DoubleLogEntry(DataLogManager.getLog(), "Kicker/Setpoint", "RPS");
    
        private final DoubleLogEntry shooterSetPointLog = 
            new DoubleLogEntry(DataLogManager.getLog(), "Shooter/Setpoint", "RPS");
    
    
        public Shooter() {
            shooter.getConfigurator().apply(ShooterConstants.SHOOTER_CONFIG);
            kicker.configure(ShooterConstants.KICKER_CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            shooter2.getConfigurator().apply(ShooterConstants.SHOOTER_CONFIG);
            shooter2.setControl(new Follower(HardwareConstants.SHOOTER_CAN, MotorAlignmentValue.Opposed)); //TODO
    
            if (BuildConstants.PUBLISH_EVERYTHING){
                NetworkTable nt = NetworkTableInstance.getDefault().getTable("Shooter");
                kickerVeloPub = nt.getDoubleTopic("kickerVelo").publish();
                shooterVeloPub = nt.getDoubleTopic("shooterVelo").publish();
                kickerSetPointPub = nt.getDoubleTopic("kickerSetpoint").publish();
                shooterSetpointPub = nt.getDoubleTopic("shooterSetpoint").publish();
            } else {
                kickerVeloPub = null;
                shooterVeloPub = null;
            }
        }
    
    
        public void setShooterSpeed(double velo) {
            velo = MathUtil.clamp(velo, 0.0, 100); //TODO
    
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
         * @param velo (percent: -100 to 100)
         */
        public void setKickerSpeed(double velo) {
            kicker.set(velo);
    
            kickerSetPointLog.append(velo);
    
            if (BuildConstants.PUBLISH_EVERYTHING){
                kickerSetPointPub.set(velo);
            }
        }
    
        public void stopShooterSub() {
            shooter.setControl(staticBrakeRequest);
            kicker.stopMotor();
        }
    
        private void setVoltage(double voltage){
            shooter.setVoltage(voltage);
        }
    
        public static SysIdRoutine getShooterRoutine(Shooter shooter) {
            System.out.println("Starting CTRE SignalLogger due to getModuleDriveRoutine");
            SignalLogger.start();
    
            if(shooterRoutine == null){
            shooterRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(0.50).per(Second),
                Volts.of(6), null, state -> SignalLogger.writeString("SysID_Shooter", state.toString())
            ), new SysIdRoutine.Mechanism((Voltage v) -> {
                shooter.setVoltage(v.in(Volts));
            }, null, shooter) 
            );
        }
        return shooterRoutine;
    }



    public void log() {
        shooterVelo.refresh();
        kickerRelativeEncoder.getVelocity();

        shooterVeloLog.append(shooterVelo.getValueAsDouble());
        kickerVeloLog.append(kickerRelativeEncoder.getVelocity());

        if (BuildConstants.PUBLISH_EVERYTHING) {
            shooterVeloPub.set(shooterVelo.getValueAsDouble());
            kickerVeloPub.set(kickerRelativeEncoder.getVelocity());
        }
    }
}
