package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.configurables.ShooterConfig;

public class ShooterSubsystem {
    private final DcMotorEx shooter;
    private final Servo transferGate;

    private double targetVelocity = ShooterConfig.DEFAULT_TARGET_TPS;
    private boolean flywheelEnabled = false;
    private boolean gateOpen = false;

    private double lastP, lastI, lastD, lastF;

    /** Closes the transfer gate (this is the shooter's "init state"). The flywheel stays off until spinUp(). */
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
     * Calculates base flywheel velocity from distance using the 4th-degree polynomial,
     * clamped between MIN_TPS and MAX_TPS.
     */
    public static double calculateVelocityFromDistance(double distanceInches) {
        double x = distanceInches;
        // Horner's form: efficient & numerically stable
        double rawTps = ((((ShooterConfig.COEFF_A * x + ShooterConfig.COEFF_B) * x + ShooterConfig.COEFF_C) * x
                + ShooterConfig.COEFF_D) * x) + ShooterConfig.COEFF_E;
        return Range.clip(rawTps, ShooterConfig.MIN_TPS, ShooterConfig.MAX_TPS);
    }

    public void applyPidCoefficients() {
        shooter.setVelocityPIDFCoefficients(ShooterConfig.P, ShooterConfig.I, ShooterConfig.D, ShooterConfig.F);
        lastP = ShooterConfig.P;
        lastI = ShooterConfig.I;
        lastD = ShooterConfig.D;
        lastF = ShooterConfig.F;
    }

    public void spinUp() { flywheelEnabled = true; }
    public void spinDown() { flywheelEnabled = false; }
    public boolean isFlywheelOn() { return flywheelEnabled; }

    /** Sets the target (trims allowed to go outside MIN/MAX_TPS, but never outside the ABS clamps). */
    public void setTargetVelocity(double tps) {
        this.targetVelocity = Range.clip(tps, ShooterConfig.ABS_MIN_TPS, ShooterConfig.ABS_MAX_TPS);
    }

    public double getTargetVelocity() { return targetVelocity; }
    public double getCurrentVelocity() { return shooter.getVelocity(); }

    public void openGate() {
        transferGate.setPosition(ShooterConfig.GATE_OPEN);
        gateOpen = true;
    }

    public void closeGate() {
        transferGate.setPosition(ShooterConfig.GATE_CLOSED);
        gateOpen = false;
    }

    public boolean isGateOpen() {
        return gateOpen;
    }

    public void update() {
        if (ShooterConfig.P != lastP || ShooterConfig.I != lastI
                || ShooterConfig.D != lastD || ShooterConfig.F != lastF) {
            applyPidCoefficients();
        }

        if (flywheelEnabled) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setVelocity(0);
        }
    }
}