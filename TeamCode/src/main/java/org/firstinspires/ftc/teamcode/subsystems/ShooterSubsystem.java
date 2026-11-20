package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Config
public class ShooterSubsystem {
    public static int TRIM_STEP = 10;

    // --- Hard Safety Clamps ---
    public static double MIN_TPS = 2070.0;
    public static double MAX_TPS = 2510.0;

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

    public static double GATE_CLOSED = 0.50;
    public static double GATE_OPEN = 0.65;

    private final DcMotorEx shooter;
    private final Servo transferGate;

    private double targetVelocity = 2220.0;
    private boolean flywheelEnabled = false;
    private boolean gateOpen = false;

    private double lastP, lastI, lastD, lastF;

    public ShooterSubsystem(HardwareMap hwMap) {
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        transferGate = hwMap.get(Servo.class, "transferGate");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        applyPidCoefficients();
        closeGate();
    }

    /**
     * Calculates base flywheel velocity from distance using the 4th-degree polynomial.
     * Output is strictly clamped between MIN_TPS (2070) and MAX_TPS (2510).
     */
    public static double calculateVelocityFromDistance(double distanceInches) {
        double x = distanceInches;
        // Horner's form: highly efficient & numerically stable
        double rawTps = ((((COEFF_A * x + COEFF_B) * x + COEFF_C) * x + COEFF_D) * x) + COEFF_E;
        return Range.clip(rawTps, MIN_TPS, MAX_TPS);
    }

    public void applyPidCoefficients() {
        shooter.setVelocityPIDFCoefficients(P, I, D, F);
        lastP = P;
        lastI = I;
        lastD = D;
        lastF = F;
    }

    public void spinUp() { flywheelEnabled = true; }
    public void spinDown() { flywheelEnabled = false; }
    public boolean isFlywheelOn() { return flywheelEnabled; }

    // In ShooterSubsystem.java:
    public void setTargetVelocity(double tps) {
        // Allows trims to drop below 2070 or above 2510 without being blocked!
        this.targetVelocity = Range.clip(tps, 1800.0, 2750.0);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getCurrentVelocity() { return shooter.getVelocity(); }

    public void openGate() {
        transferGate.setPosition(GATE_OPEN);
        gateOpen = true;
    }

    public void closeGate() {
        transferGate.setPosition(GATE_CLOSED);
        gateOpen = false;
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    public void update() {
        if (P != lastP || I != lastI || D != lastD || F != lastF) {
            applyPidCoefficients();
        }

        if (flywheelEnabled) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setVelocity(0);
        }
    }
}