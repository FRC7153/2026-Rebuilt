package frc.robot.Subsystems.Vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Libs.LimelightHelpers;
import frc.robot.Util.Dashboard.HardwareFaultTracker;

public class Limelight implements Subsystem{
    public static enum Version {
        LIMELIGHT_3G(true, false),
        LIMELIGHT_4(true, false);
    /** Whether the LL has a integrated IMU */
    public final boolean integratedIMU;

    /** Whether the LL has fans for cooling */
    public final boolean hasFans;

    private Version(boolean integratedIMU, boolean hasFans) {
        this.integratedIMU = integratedIMU;
        this.hasFans = hasFans;
    }
}

    protected final String cameraName;
    protected final Version version;


    //NT
    private final DoubleArraySubscriber statsSub;
    private final DoubleSubscriber heartbeatSub;
    private final DoublePublisher pipelinePub;

    //Alert
    private final Alert notConnectedAlert;
    

    // Stats 
    private double lastHeartbeat = -1.0;
    private double lastHeartbeatTS = 0.0;
    private boolean alive = false;

    //Logging 
    private final DoubleLogEntry fpsLog, cpuTempLog, ramLog, tempLog;
    private final BooleanLogEntry isAliveLog;


    /**
     * 
     * @param name Host Camera ID
     * @param version
     */
    public Limelight(String name, Version version) {
        cameraName = name;
        this.version = version;

        notConnectedAlert = new Alert(String.format("Limelight %s is not alive", name), AlertType.kWarning);

        //NT Topics 
        NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable(name);
        
        heartbeatSub = cameraTable.getDoubleTopic("hb").subscribe(-1.0);
        statsSub = cameraTable.getDoubleArrayTopic("hw").subscribe(new double[4]);

        pipelinePub = cameraTable.getDoubleTopic("pipeline").publish();

        // Init Logging
        String logName = String.format("Limelight/%s", name);

        fpsLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "fps");
        cpuTempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "cpuTemp", "f");
        ramLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "ram", "%");
        tempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "temp", "f");
        isAliveLog = new BooleanLogEntry(DataLogManager.getLog(), logName + "isAlive");
    } 

    public void setPipeline(int pipeline) {
        pipelinePub.set(pipeline);
    } 

    // Logs diagnostics data
    public void log() {
        //get stats 
        double[] stats = statsSub.get();

        if(alive && stats.length == 4) {
            fpsLog.append(stats[0]);
            cpuTempLog.append(stats[1]);
            ramLog.append(stats[2]);
            tempLog.append(stats[3]);
        }
    }

    /**
     * Checks if this limellight is alive 
     */
    public void checkHardware() {

        //Check heartbeat
        double newHeartbeat = heartbeatSub.get();

        if (newHeartbeat == -1.0) {
            // no heartbeat 
            alive = false; 
        } else if (newHeartbeat != lastHeartbeat) {
            lastHeartbeat = newHeartbeat;
            lastHeartbeatTS = Timer.getFPGATimestamp();
            alive = true;
        } else if (Timer.getFPGATimestamp() - lastHeartbeatTS > 1.5) {
            // heartbeat timeout 
            alive = false;
        }


        HardwareFaultTracker.checkFault(notConnectedAlert, !alive);
        isAliveLog.append(alive);
    }

    /**
     * 
     * @return if the LL is returning a heartbeat
     */
    public boolean isAlive() {
        return alive;
    }

    /**
     * 
     * @return the version of the LL
     */
    public Version getVersion() {
        return version;
    }








    

}
