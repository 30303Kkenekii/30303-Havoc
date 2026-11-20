package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class IntakeSubsystem {
    // --- Servo Presets: Intake Pivot ---
    public static double PIVOT_UP = .8;
    public static double PIVOT_DOWN = .61;

    // --- Servo Presets: Flower Pivot ---
    public static double FLOWER_UP = .25;
    public static double FLOWER_DOWN = .7;

    // --- Flower Arm Speed (Tune this in Dashboard!) ---
    // Position units per second.
    // Stroke is 0.60 (0.80 - 0.20). At 1.2 speed, it moves in 0.50 seconds.
    // Lower = slower & gentler (e.g. 0.8), Higher = faster (e.g. 2.0).
    public static double FLOWER_SPEED = .4;

    // --- Motor Powers ---
    public static double INTAKE_POWER = 1.00;
    public static double SHOOTING_FEED_POWER = .75;
    public static double EJECT_POWER = -1.00;

    // --- Auto Beam Timings ---
    public static double BEAM_CONFIRM_SEC = 0.40;
    public static double GATE_OPEN_DELAY_SEC = 0.60;

    private final DcMotor frontMotor;
    private final DcMotor backMotor;
    private final Servo pivot;
    private final Servo flowerPivot;
    private final DigitalChannel beamBreak;

    private double currentPower = 0.0;
    private boolean flowerIsDown = false;

    // Flower rate-limiting tracking
    private double targetFlowerPos = FLOWER_UP;
    private double currentFlowerPos = FLOWER_UP;
    private final ElapsedTime flowerTimer = new ElapsedTime();

    private final ElapsedTime beamTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
    private boolean sequenceActive = false;
    private boolean readyToOpenGate = false;

    public IntakeSubsystem(HardwareMap hwMap) {
        frontMotor = hwMap.get(DcMotor.class, "intakeMotorFront");
        backMotor = hwMap.get(DcMotor.class, "intakeMotorBack");
        pivot = hwMap.get(Servo.class, "intakePivot");
        flowerPivot = hwMap.get(Servo.class, "flowerPivot");
        beamBreak = hwMap.get(DigitalChannel.class, "beamBreak");

        frontMotor.setDirection(DcMotor.Direction.REVERSE);
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        // Initial setup
        pivotDown();
        flowerPivot.setPosition(FLOWER_UP);
        targetFlowerPos = FLOWER_UP;
        currentFlowerPos = FLOWER_UP;
        flowerTimer.reset();
    }

    public void setPower(double power) {
        this.currentPower = power;
        frontMotor.setPower(power);
        backMotor.setPower(power);
    }

    public void intake() { setPower(INTAKE_POWER); }
    public void intakeShooting() { setPower(SHOOTING_FEED_POWER); }
    public void eject() { setPower(EJECT_POWER); }
    public void stop() { setPower(0); }

    public double getPower() { return currentPower; }
    public boolean isIntaking() { return currentPower > 0.05; }

    public void pivotUp() { pivot.setPosition(PIVOT_UP); }
    public void pivotDown() { pivot.setPosition(PIVOT_DOWN); }
    public void setPivotPosition(double pos) { pivot.setPosition(pos); }

    // --- Flower Arm Control (Smooth Slew) ---
    public void flowerUp() {
        targetFlowerPos = FLOWER_UP;
        flowerIsDown = false;
    }

    public void flowerDown() {
        targetFlowerPos = FLOWER_DOWN;
        flowerIsDown = true;
    }

    public void toggleFlower() {
        if (flowerIsDown) flowerUp(); else flowerDown();
    }

    public void setFlowerPosition(double pos) {
        targetFlowerPos = pos;
    }

    public boolean isFlowerDown() { return flowerIsDown; }
    public boolean isBeamBroken() { return beamBreak.getState(); }
    public boolean shouldOpenGate() { return readyToOpenGate; }

    public boolean isConfirmedFull() {
        return sequenceActive;
    }

    /**
     * Updates beam-break automation and smoothly glides the flower arm
     */
    public void update(boolean isShooting) {
        // -----------------------------------------------------------
        // 1. Smooth Flower Arm Motion (Rate Limiter)
        // -----------------------------------------------------------
        double dt = flowerTimer.seconds();
        flowerTimer.reset();
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        if (Math.abs(targetFlowerPos - currentFlowerPos) > 1e-4) {
            double maxStep = FLOWER_SPEED * dt;
            double error = targetFlowerPos - currentFlowerPos;

            if (Math.abs(error) <= maxStep) {
                currentFlowerPos = targetFlowerPos;
            } else {
                currentFlowerPos += Math.signum(error) * maxStep;
            }
            flowerPivot.setPosition(currentFlowerPos);
        }

        // -----------------------------------------------------------
        // 2. Beam-Break Sequence
        // -----------------------------------------------------------
        if (isShooting) {
            beamTimer.reset();
            return;
        }

        boolean beam = isBeamBroken();
        if (!beam && !sequenceActive) {
            beamTimer.reset();
        }

        // Stage 1: Beam broken long enough -> lift pivot & stop motors
        if (!sequenceActive && beam && beamTimer.seconds() >= BEAM_CONFIRM_SEC) {
            pivotUp();
            stop();
            sequenceActive = true;
            readyToOpenGate = false;
            gateTimer.reset();
        }

        // Stage 2: Wait delay -> signal transfer gate can open
        if (sequenceActive && !readyToOpenGate && gateTimer.seconds() >= GATE_OPEN_DELAY_SEC) {
            readyToOpenGate = true;
        }

        // Stage 3: Beam cleared -> re-arm
        if (readyToOpenGate && !beam) {
            resetAutoSequence();
        }
    }

    public void resetAutoSequence() {
        sequenceActive = false;
        readyToOpenGate = false;
        beamTimer.reset();
    }
}