package frc.robot.Subsystems.Swerve;

import com.ctre.phoenix6.SignalLogger;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SwerveSysID {
  private static SysIdRoutine moduleDriveRoutine, moduleSteerRoutine;

  /** Gets routine for module drive motor SysId characterization. This will start the CTRE SignalLogger */
  public static SysIdRoutine getModuleDriveRoutine(SwerveDrive drive) {
    System.out.println("Starting CTRE SignalLogger due to getModuleDriveRoutine call");
    SignalLogger.start();

    if (moduleDriveRoutine == null) {
      // No cached routine, instantiate it
      moduleDriveRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, (State state) -> {
          // Log state
          SignalLogger.writeString("drive-sysid-state", state.toString());
        }), 
        new SysIdRoutine.Mechanism((Voltage v) -> {
          // Apply voltages
          for (int m = 0; m < 4; m++) {
            drive.modules[m].setVoltageRequest(v.in(Volts));
          }
        }, null, drive)
      );
    }

    return moduleDriveRoutine;
  }

   /** Gets routine for module steer motor SysId characterization. This will start the URCL
   * logger. It will only characterize the front left module.
   */
  public static SysIdRoutine getModuleSteerRoutine(SwerveDrive drive) {
    System.out.println("Starting CTRE SignalLogger due to getModuleSteerRoutine call");
    SignalLogger.start();

    if (moduleSteerRoutine == null) {
      // No cached routine, instantiate it
      moduleSteerRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, (State state) -> {
          // Log state
          SignalLogger.writeString("steer-sysid-state", state.toString());
        }), 
        new SysIdRoutine.Mechanism((Voltage v) -> {
          // Apply voltages
          drive.modules[0].setSteerVoltage(v.in(Volts));
        }, null, drive)
      );
    }

    return moduleSteerRoutine;
  }
}
