package frc.robot;


public final class Constants {
    public static final class DriveConstants {
        public static final int left1Port = 0;
        public static final int left2Port = 1;
        public static final int right1Port = 2;
        public static final int right2Port = 3;
        public static final int controllerPort = 0;
        public static final int controller2Port = 1;
        public static final int gyroPort = 10;
    }

    public static final class ArmConstants {
        public static final int stage1Port = 4;
        public static final int stage2Port = 5;
        public static final int slotIdx = 0;

        public static final double stage1StallPourcentage1 = 0.0528;
        public static final double stage1StallPourcentage2 = 0.0381;

        public static final double stage2StallPourcentage = 0.0449;
        

        public static final double stage1F = 0;
        public static final double stage1P = 1;
        public static final double stage1I = 0;
        public static final double stage1D = 0.030;
        public static final double stage1Accel = 80;
        public static final double stage1Cruise = 150;
        public static final double stage1FwdLimit = 0;
        public static final double stage1RevLimit = 0;
        public static final boolean stage1Invert = true;

        public static final double stage2F = 0;
        public static final double stage2P = 0.9;
        public static final double stage2I = 0;
        public static final double stage2D = 0.045;
        public static final double stage2Accel = 15;
        public static final double stage2Cruise = 25;
        public static final double stage2FwdLimit = 0;
        public static final double stage2RevLimit = 0;
        public static final boolean stage2Invert = true;

        public static final double pourcentageDeadband = 0;
        public static final double IZone1 = 10;
		public static final int CANCoder1Port = 9;
        public static final int CANCoder2Port = 8;
        public static final double IZone2 = 10;

        public static final class ArmPickupConeHP {
            public static final double stage1Pos = 0;
            public static final double stage2Pos = 0;
        }

        public static final class ArmPositions {
            public static final int stowed1 = 0;
            public static final int stowed2 = 0;

            public static final int ground1 = 0;
            public static final int ground2 = 0;

            public static final int high1 = 0;
            public static final int high2 = 0;

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

        public static final class TurretPositions {
            public static final int front = 0;
            public static final int back = 20000;
        }

        public static final int turretPort = 6;
        public static final int pidIdx = 0;

        public static final double limelightTurretP = 0.03
        ;
        public static final double limelightTurretI = 0;
        public static final double limelightTurretD = 0;

        public static final double turretF = 0.05465;
        //public static final double turretF = 0.75;
        public static final double turretP = 0.3;
        //public static final double turretP = 1.1;
        public static final double turretI = 0.0105;
        public static final double turretD = 0.1;
        public static final int turretAccel = 24000;
        public static final int turretCruise = 15000;

        public static final double pourcentDeadband = 0.01;

        public static final int slotIdx = 0;
        public static final double IZone = 500;
        public static final int calSwitchChannel = 0;

    }

    public static final class ManipulatorConstants {
        //TODO
        public static final int manipulatorPort = 7;

        public static final double velocityP = 0;
        public static final double velocityI = 0;
        public static final double velocityD = 0;
        public static final double velocityF = 0;

        public static final double cubePickupVelocity = 0;
    }

}

