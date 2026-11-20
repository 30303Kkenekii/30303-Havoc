package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretConfig {
    public static boolean ENABLED = true;

    // --- Right Sensor Calibration (180° Baseline: -90° to +90°) ---
    public static double CENTER_RIGHT = 1.623;
    public static double SCALE_RIGHT = -92.69;

    // --- Left Sensor Calibration (-90° to -180°) ---
    public static double V_LEFT_AT_NEG90 = 0.940; // Captured at -90°
    public static double SCALE_LEFT = -91.56;     // Solved to hit -180° with 0.0° drift

    // --- Travel Limits ---
    public static double MIN_ANGLE_DEG = -175.0;
    public static double MAX_ANGLE_DEG = 130.0;

    // --- Servo Directions ---
    public static boolean INVERT_LEFT = false;
    public static boolean INVERT_RIGHT = true;

    // --- Closed Loop PID & Friction ---
    public static double kP = .0088;
    public static double kI = 0;
    public static double kD = 0;
    public static double kS = .085;
    public static double DEADBAND_DEG = 1.5;
    public static double MAX_POWER = 1.00;

    // --- Driver Controls ---
    public static double TRIM_STEP_DEG = 1.0;
    public static double TOLERANCE_DEG = 2.0;
}