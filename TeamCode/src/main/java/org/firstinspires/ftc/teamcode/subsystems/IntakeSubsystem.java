package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.configurables.IntakeConfig;

public class IntakeSubsystem {
    private final DcMotor frontMotor;
    private final DcMotor backMotor;
    private final Servo flowerPivot;
    private final DigitalChannel beamBreak;

    private double currentPower = 0.0;
    private boolean flowerIsDown = false;
    private boolean raiseFlowerWhenFull = true; // Auto turns this off to keep the flower down

    private final ElapsedTime beamTimer = new ElapsedTime();
    private final ElapsedTime gateTimer = new ElapsedTime();
    private boolean sequenceActive = false;
    private boolean readyToOpenGate = false;

    /** Moves the flower arm to its UP position (this is the intake's "init state"). */
    public IntakeSubsystem(HardwareMap hwMap) {
        frontMotor = hwMap.get(DcMotor.class, "intakeMotorFront");
        backMotor = hwMap.get(DcMotor.class, "intakeMotorBack");
        flowerPivot = hwMap.get(Servo.class, "flowerPivot");
        beamBreak = hwMap.get(DigitalChannel.class, "beamBreak");

        frontMotor.setDirection(DcMotor.Direction.REVERSE);
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        flowerUp();
    }

    // --- Intake Motors ---
    public void setPower(double power) {
        this.currentPower = power;
        frontMotor.setPower(power);
        backMotor.setPower(power);
    }

    public void intake() { setPower(IntakeConfig.INTAKE_POWER); }
    public void intakeShooting() { setPower(IntakeConfig.SHOOTING_FEED_POWER); }
    public void eject() { setPower(IntakeConfig.EJECT_POWER); }
    public void stop() { setPower(0); }

    public double getPower() { return currentPower; }
    public boolean isIntaking() { return currentPower > 0.05; }

    // --- Flower Arm ---
    public void flowerUp() {
        flowerPivot.setPosition(IntakeConfig.FLOWER_UP);
        flowerIsDown = false;
    }

    public void flowerDown() {
        flowerPivot.setPosition(IntakeConfig.FLOWER_DOWN);
        flowerIsDown = true;
    }

    public void toggleFlower() {
        if (flowerIsDown) flowerUp(); else flowerDown();
    }

    public boolean isFlowerDown() { return flowerIsDown; }

    /** Per-OpMode switch: false = a full intake only stops the motors and never moves the flower. */
    public void setRaiseFlowerWhenFull(boolean raise) { this.raiseFlowerWhenFull = raise; }

    /**
     * Intake is done (4 balls in, or an Auto timeout): stop the motors and raise the
     * flower if it was down. If it was already up, it stays up.
     */
    public void finishIntaking() {
        stop();
        if (IntakeConfig.FLOWER_UP_WHEN_FULL && raiseFlowerWhenFull && flowerIsDown) {
            flowerUp();
        }
    }

    // --- Beam Break ---
    public boolean isBeamBroken() { return beamBreak.getState(); }
    public boolean shouldOpenGate() { return readyToOpenGate; }

    public boolean isConfirmedFull() {
        return sequenceActive;
    }

    /**
     * Updates the beam-break automation. Call every loop.
     */
    public void update(boolean isShooting) {
        if (isShooting) {
            beamTimer.reset();
            return;
        }

        boolean beam = isBeamBroken();
        if (!beam && !sequenceActive) {
            beamTimer.reset();
        }

        // Stage 1: Beam broken long enough -> stop intake (+ flower up if it was down)
        if (!sequenceActive && beam && beamTimer.seconds() >= IntakeConfig.BEAM_CONFIRM_SEC) {
            finishIntaking();
            sequenceActive = true;
            readyToOpenGate = false;
            gateTimer.reset();
        }

        // Stage 2: Wait delay -> signal transfer gate can open
        if (sequenceActive && !readyToOpenGate && gateTimer.seconds() >= IntakeConfig.GATE_OPEN_DELAY_SEC) {
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