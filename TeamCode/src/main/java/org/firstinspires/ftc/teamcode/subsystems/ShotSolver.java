package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ShotSolver {
    // --- Dashboard Tunables ---
    public static double SHOOT_FEED_TIME = 0.60;     // Feed duration into flywheel
    public static double GATE_CLOSE_DELAY_SEC = 0.20; // Wait for the gate to close before intaking again
    public static double RECOVERY_DELAY_SEC = 0.60;  // Delay before resuming intake on recovery

    public enum State { IDLE, FEEDING, CLOSING_GATE, RECOVERING }
    private State state = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    private final IntakeSubsystem intake;
    private final ShooterSubsystem shooter;

    public ShotSolver(IntakeSubsystem intake, ShooterSubsystem shooter) {
        this.intake = intake;
        this.shooter = shooter;
    }

    /**
     * Starts the shoot sequence (Right Trigger).
     */
    public void startShoot() {
        if (state == State.IDLE) {
            state = State.FEEDING;
            timer.reset();
            intake.intakeShooting();
            shooter.openGate();
        }
    }

    /**
     * Triggers the false-positive reset (Button A).
     */
    public void startRecovery() {
        if (state != State.FEEDING && state != State.CLOSING_GATE) {
            shooter.closeGate();
            state = State.RECOVERING;
            timer.reset();
        }
    }

    public boolean isShooting() {
        return state == State.FEEDING || state == State.CLOSING_GATE;
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    public State getState() {
        return state;
    }

    /**
     * Advances the non-blocking state machine.
     */
    public void update() {
        switch (state) {
            case FEEDING:
                intake.intakeShooting();
                if (timer.seconds() >= SHOOT_FEED_TIME) {
                    intake.stop();
                    shooter.closeGate();
                    state = State.CLOSING_GATE;
                    timer.reset();
                }
                break;

            case CLOSING_GATE:
                intake.stop();
                if (timer.seconds() >= GATE_CLOSE_DELAY_SEC) {
                    resumeIntaking();
                }
                break;

            case RECOVERING:
                if (timer.seconds() >= RECOVERY_DELAY_SEC) {
                    resumeIntaking();
                }
                break;

            case IDLE:
            default:
                break;
        }
    }

    private void resumeIntaking() {
        intake.intake();
        intake.resetAutoSequence();
        state = State.IDLE;
    }
}