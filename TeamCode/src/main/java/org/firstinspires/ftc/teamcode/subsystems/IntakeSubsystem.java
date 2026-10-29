package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class IntakeSubsystem {
    public static double PIVOT_UP = 0.87;
    public static double PIVOT_DOWN = 0.69;

    public static double FLOWER_UP = 0.20;
    public static double FLOWER_DOWN = 0.80;

    public static double INTAKE_POWER = 1.00;
    public static double EJECT_POWER = -1.00;

    // Increased from 0.35s to 0.45s to prevent false triggers
    public static double BEAM_CONFIRM_SEC = 0.4;
    public static double GATE_OPEN_DELAY_SEC = 0.60;

    private final DcMotor frontMotor;
    private final DcMotor backMotor;
    private final Servo pivot;
    private final Servo flowerPivot;
    private final DigitalChannel beamBreak;

    private double currentPower = 0.0;
    private boolean flowerIsDown = false;
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

        pivotDown();
        flowerUp();
    }

    public void setPower(double power) {
        this.currentPower = power;
        frontMotor.setPower(power);
        backMotor.setPower(power);
    }

    public void intake() { setPower(INTAKE_POWER); }
    public void eject() { setPower(EJECT_POWER); }
    public void stop() { setPower(0); }

    public double getPower() { return currentPower; }
    public boolean isIntaking() { return currentPower > 0.05; }

    public void pivotUp() { pivot.setPosition(PIVOT_UP); }
    public void pivotDown() { pivot.setPosition(PIVOT_DOWN); }
    public void setPivotPosition(double pos) { pivot.setPosition(pos); }

    public void flowerUp() {
        flowerPivot.setPosition(FLOWER_UP);
        flowerIsDown = false;
    }

    public void flowerDown() {
        flowerPivot.setPosition(FLOWER_DOWN);
        flowerIsDown = true;
    }

    public void toggleFlower() {
        if (flowerIsDown) flowerUp(); else flowerDown();
    }

    public boolean isFlowerDown() { return flowerIsDown; }
    public boolean isBeamBroken() { return beamBreak.getState(); }
    public boolean shouldOpenGate() { return readyToOpenGate; }

    public boolean isConfirmedFull() {
        return sequenceActive;
    }

    public void update(boolean isShooting) {
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