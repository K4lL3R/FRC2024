package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double heightOfAprilTag = 7.0;
    

    public static final class Climb {
        public static final int encoderPort = 7;
        //Comp
        // public static final double mappingOffset = 0.410;
        // public static final double mappingHeight = 9.0;
        //Practice
        public static final double mappingOffset = 0.222;
        public static final double mappingHeight = 9.0;

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.00005;
        public static final double maxVel = 24000;//was 6000
        public static final double maxAccel = 90000;

        public static final class GlobalSetpoints {
            public static final double elevatorUpPos = 49;
            public static final double midPos = 28;
            //mid 24
            public static final double elevatorDefaultPos = 0;
            public static final double ampElevator = 30;    
        }

        public enum Position {
            Up,
            Mid,
            Down,
            Amp
        }
    }

    public static final class Wrists {
        public static final double hardStopDegOffset = 0.507;

        public static final class ShooterConst {
            public static final int canID = 15;
            public static final int encoderPort = 0;
            public static final IdleMode idleMode = IdleMode.kBrake;

            public static final double shooterSetPoint = -8;
            public static final double climbLock = 0;//3.8
            public static final double downSetPoint = -55.7;//-58 -21 -23.5
            public static final double farShots = -25.2;//-27.3
            public static final double autoFar = -17.8;

            public enum ShooterMode {
                ClimbLock,
                Shooting,
                Down,
                FarShots, 
                AutoFar
            }
        }
        //-6.3 ticks = 63 deg
        //-40.8 = 0

        public static final class Arm {
            public static final int canID = 19;
            public static final int encoderPort = 1;
            public static final IdleMode idleMode = IdleMode.kBrake;

            public static final double feedSetPoint = 0;
            public static final double stowSetPoint = -1.67;
            public static final double scoreSetPoint = -28;

            public enum ArmMode {
                Feed,
                Stow,
                Score
            }
        }

        public static final class Intake {
            public static final int canID = 20;
            public static final int encoderPort = 2;
            public static final IdleMode idleMode = IdleMode.kBrake;

            public static final double intakeSetPoint = -29;//wasa 28.15
            public static final double stowSetPoint = -4.47;
            public static final double feedSetPoint = 0;

            public enum IntakeMode {
                Down,
                Stow,
                Feed
            }
        }

        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kF = 0.00008;
        public static final double maxVel = 24000;
        public static final double maxAccel = 24000;
    }

    public static final class Intake {
        public static final int canID = 9;
        public static final double intake_cone_outtake_cube = 1;//0.8
        public static final double intake_cube_outtake_cone = -1;//-0.5
    }

    public static final class LEDs {
        public enum Colors {
            Green,
            Orange
        }
        public enum AnimationTypes {  
            ColorFlow,
            Fire,
            Larson,
            Rainbow,
            RgbFade,
            SingleFade,
            Strobe,
            Twinkle,
            TwinkleOff,
            SetAll
        }
    }
}