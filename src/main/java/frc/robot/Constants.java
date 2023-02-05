package frc.robot;

public final class Constants {
    public static final class DriveConstants {
        public static final int left1Port = 0;
        public static final int left2Port = 1;
        public static final int right1Port = 2;
        public static final int right2Port = 3;
        public static final int controllerPort = 0;
    }

    public static final class ArmConstants {
        public static final int stage1Port = 4;
        public static final int stage2Port = 5;
        public static final int slotIdx = 0;

        public static final double stage1F = 0;
        public static final double stage1P = 0;
        public static final double stage1I = 0;
        public static final double stage1D = 0;
        public static final double stage1Accel = 0;
        public static final double stage1Cruise = 0;
        public static final double stage1FwdLimit = 0;
        public static final double stage1RevLimit = 0;
        public static final boolean stage1Invert = false;

        public static final double stage2F = 0;
        public static final double stage2P = 0;
        public static final double stage2I = 0;
        public static final double stage2D = 0;
        public static final double stage2Accel = 0;
        public static final double stage2Cruise = 0;
        public static final double stage2FwdLimit = 0;
        public static final double stage2RevLimit = 0;
        public static final boolean stage2Invert = true;

        public static final class ArmPickupConeHP {
            public static final double stage1Pos = 0;
            public static final double stage2Pos = 0;
        }
    }

    public static final class TurretConstants {
        public enum Apriltags {
            REDRIGHT1,
            REDMIDDLE2,
            REDLEFT3,
            BLUERIGHT6,
            BLUEMIDDLE7,
            BLUELEFT8,
            BLUEDOUBLE4,
            REDDOUBLE5
        }

        public enum Armpositions {
            FRONT,
            BACK
        }

        public static final class ArmPositions {
            public static final int front = 0;
            public static final int back = 0;
        }

        public static final int turretPort = 6;
        public static final int pidIdx = 0;

        public static final int limelightTurretP = 0;
        public static final int limelightTurretI = 0;
        public static final int limelightTurretD = 0;

        public static final int turretF = 0;
        public static final int turretP = 0;
        public static final int turretI = 0;
        public static final int turretD = 0;
        public static final int turretAccel = 0;
        public static final int turretCruise = 0;

        public static final int slotIdx = 0;

    }

}

