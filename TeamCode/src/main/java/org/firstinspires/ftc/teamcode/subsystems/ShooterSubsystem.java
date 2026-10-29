package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class ShooterSubsystem {
    public static double DEFAULT_TPS = 2220;
    public static int TRIM_STEP = 10;

    public static double GATE_CLOSED = 0.50;
    public static double GATE_OPEN = 0.65;

    public static boolean TURRET_ENABLED = false; // Kept false until wired

    private final DcMotorEx shooter;
    private final Servo transferGate;

    private double targetVelocity = DEFAULT_TPS;
    private boolean flywheelEnabled = false;
    private boolean gateOpen = false;

    public ShooterSubsystem(HardwareMap hwMap) {
        shooter = hwMap.get(DcMotorEx.class, "shooter");
        transferGate = hwMap.get(Servo.class, "transferGate");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        closeGate();
    }

    public void spinUp() { flywheelEnabled = true; }
    public void spinDown() { flywheelEnabled = false; }
    public boolean isFlywheelOn() { return flywheelEnabled; }

    public void setTargetVelocity(double tps) { this.targetVelocity = Math.max(0, tps); }
    public void trimTargetVelocity(double delta) { setTargetVelocity(this.targetVelocity + delta); }
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

    /**
     * Applies motor outputs. Completely independent of other subsystems.
     */
    public void update() {
        if (flywheelEnabled) {
            shooter.setVelocity(targetVelocity);
        } else {
            shooter.setVelocity(0);
        }
    }
}