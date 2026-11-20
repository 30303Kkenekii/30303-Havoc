package org.firstinspires.ftc.teamcode.configurables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterConfig {
    // --- Driver Controls ---
    public static int TRIM_STEP = 10;

    // --- Startup Target (before TeleOp/Auto sets one) ---
    public static double DEFAULT_TARGET_TPS = 2220.0;

    // --- Curve Clamps (applied to the distance equation output) ---
    public static double MIN_TPS = 1500.0;
    public static double MAX_TPS = 3000.0;

    // --- Absolute Safety Clamps (applied to the final target, trims included) ---
    public static double ABS_MIN_TPS = 1500.0;
    public static double ABS_MAX_TPS = 3000.0;

    // --- 4th-Degree Polynomial Coefficients ---
    // y = -0.00065959x^4 + 0.168091x^3 - 15.5185x^2 + 622.75902x - 7111.16613
    public static double COEFF_A = -0.00065959;
    public static double COEFF_B = 0.168091;
    public static double COEFF_C = -15.5185;
    public static double COEFF_D = 622.75902;
    public static double COEFF_E = -7111.16613;

    // --- Tuned Velocity PIDF Coefficients ---
    public static double P = 25.0;
    public static double I = 0.0;
    public static double D = 0.0;
    public static double F = 12.5;

    // --- Transfer Gate Servo Positions ---
    public static double GATE_CLOSED = 0.50;
    public static double GATE_OPEN = 0.65;
}