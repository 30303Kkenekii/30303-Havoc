package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ShotSolver {
    // --- Dashboard Tunables ---
    public static double SHOOT_FEED_TIME = 0.60;   // Feed duration into flywheel
    public static double SHOOT_LOWER_DELAY = 0.20; // Delay before lowering intake pivot
    public static double RECOVERY_DELAY_SEC = 0.60; // Delay before lowering & resuming on recovery

    public enum State { IDLE, FEEDING, WAITING_TO_LOWER, RECOVERING }
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
        if (state != State.FEEDING && state != State.WAITING_TO_LOWER) {
            shooter.closeGate();
            state = State.RECOVERING;
            timer.reset();
        }
    }

    public boolean isShooting() {
        return state == State.FEEDING || state == State.WAITING_TO_LOWER;
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
                    state = State.WAITING_TO_LOWER;
                    timer.reset();
                }
                break;

            case WAITING_TO_LOWER:
                intake.stop();
                if (timer.seconds() >= SHOOT_LOWER_DELAY) {
                    intake.pivotDown();
                    intake.intake();
                    intake.resetAutoSequence();
                    state = State.IDLE;
                }
                break;

            case RECOVERING:
                if (timer.seconds() >= RECOVERY_DELAY_SEC) {
                    intake.pivotDown();
                    intake.intake();
                    intake.resetAutoSequence();
                    state = State.IDLE;
                }
                break;

            case IDLE:
            default:
                break;
        }
    }
}