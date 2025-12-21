// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    // CAN IDs
    // Drive

    // Front Right
    public static final int FRONT_RIGHT_STEER = 1;
    public static final int FRONT_RIGHT_ENCODER = 2;
    public static final int FRONT_RIGHT_DRIVE = 3;

    // Back Right
    public static final int BACK_RIGHT_STEER = 4;
    public static final int BACK_RIGHT_ENCODER = 5;
    public static final int BACK_RIGHT_DRIVE = 6;

    // Back Left
    public static final int BACK_LEFT_STEER = 7;
    public static final int BACK_LEFT_ENCODER = 8;
    public static final int BACK_LEFT_DRIVE = 9;

    // Front Left
    public static final int FRONT_LEFT_STEER = 10;
    public static final int FRONT_LEFT_ENCODER = 11;
    public static final int FRONT_LEFT_DRIVE = 12;

    public static final int PIGEON_IMU = 13;
    // Mechanisms

    // Elevator
    public static final int ELEVATOR_ID = 23;

    // Arm
    public static final int ARM_PIVOT_ID = 25;
    public static final int ARM_INTAKE_ID = 18;
    public static final int ARM_ENCODER_ID = 24;
}
