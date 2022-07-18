// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class DriveConstants {

        public static final class FrontLeft {
            // This is the ID of the drive motor
            public static final int DriveMotorFL = 0;
            // This is the ID of the steer motor
            public static final int SteerMotorFL = 0;
            // This is the ID of the steer encoder
            public static final int SteerEncoderFL = 0;
            // This is how much the steer encoder is offset from true zero (In our case,
            // zero is facing straight forward)
            public static final double SteerOffsetFL = 0;

        }

        public static final class BackRight {
            public static final int DriveMotorBR = 0;
            public static final int SteerMotorBR = 0;
            public static final int SteerEncoderBR = 0;
            public static final double SteerOffsetBR = 0;

        }

        public static final class BackLeft {
            public static final int DriveMotorBL = 0;
            public static final int SteerMotorBL = 0;
            public static final int SteerEncoderBL = 0;
            public static final double SteerOffsetBL = 0;

        }

        public static final class FrontRight {
            public static final int DriveMotorFR = 0;
            public static final int SteerMotorFR = 0;
            public static final int SteerEncoderFR = 0;
            public static final double SteerOffsetFR = 0;

        }

        public static final double trackwidth = 0;
        public static final double wheelbase = 0;
        public static final double maxVelocity = 0;
        public static final double maxVoltage = 0;
        public static final double maxAngularVelocity = 0;

    }

    public static final class IOConstants {
        public static final class JoySContants {

            public static final int joyS_ID = 0;
            public static final int xTranslation= 0;
            public static final int yTranslation = 1;
            public static final int rotation = 5;

        }
    }
}
