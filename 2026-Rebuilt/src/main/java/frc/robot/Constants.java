package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Constants {
    public class BuildConstants {
        public static final boolean PUBLISH_EVERYTHING = true;
        public static final boolean INCLUDE_TEST_AUTOS = true;

        public static final AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);//TODO 
    }
    public class HardwareConstants {
        // Swerve Hardware CAN IDs
        public final static int FR_DRIVE_CAN = 1;
        public final static int FR_STEER_CAN = 2;
        public final static int FL_DRIVE_CAN = 3;
        public final static int FL_STEER_CAN = 4;
        public final static int RL_DRIVE_CAN = 5;
        public final static int RL_STEER_CAN = 6;
        public final static int RR_DRIVE_CAN = 7;
        public final static int RR_STEER_CAN = 8;

        // CANCoder CAN IDs
        public final static int FR_CANCODER_CAN = 9;
        public final static int FL_CANCODER_CAN = 10;
        public final static int RL_CANCODER_CAN = 11;
        public final static int RR_CANCODER_CAN = 12;

        // Pigeon 2.0
        public final static int PIGEON_CAN_ID = 20;
        // PDH Can ID
        public static final int PDH_CAN = 21;

        // CAN Busses
        public final static CANBus CANIVORE = new CANBus("CANivore");
        public final static CANBus RIO_CAN = new CANBus("rio");
    }
}
