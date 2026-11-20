package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretSubsystem {

    private final CRServo turretLeft;
    private final CRServo turretRight;
    private final AnalogInput turretLeftAnalog;
    private final AnalogInput turretRightAnalog;

    private double targetAngleDeg = 0.0;
    private double currentAngleDeg = 0.0;
    private double currentPower = 0.0;

    private boolean latchedToLeft = false;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    public TurretSubsystem(HardwareMap hwMap) {
        turretLeft = hwMap.get(CRServo.class, "turretLeft");
        turretRight = hwMap.get(CRServo.class, "turretRight");
        turretLeftAnalog = hwMap.get(AnalogInput.class, "turretLeftAnalog");
        turretRightAnalog = hwMap.get(AnalogInput.class, "turretRightAnalog");

        timer.reset();
        stop();
    }

    public void setTargetAngle(double degrees) {
        this.targetAngleDeg = Range.clip(degrees, TurretConfig.MIN_ANGLE_DEG, TurretConfig.MAX_ANGLE_DEG);
    }

    public void trimTarget(double deltaDegrees) {
        setTargetAngle(this.targetAngleDeg + deltaDegrees);
    }

    public double getTargetAngle() {
        return targetAngleDeg;
    }

    public double getCurrentAngle() {
        return currentAngleDeg;
    }

    public double getPower() {
        return currentPower;
    }

    public boolean isLatchedToLeft() {
        return latchedToLeft;
    }

    public boolean isAtTarget() {
        return Math.abs(targetAngleDeg - currentAngleDeg) <= TurretConfig.TOLERANCE_DEG;
    }

    public void stop() {
        turretLeft.setPower(0);
        turretRight.setPower(0);
        currentPower = 0.0;
        integralSum = 0.0;
    }

    private double readFusedAngle() {
        double vLeft = turretLeftAnalog.getVoltage();
        double vRight = turretRightAnalog.getVoltage();

        double angleFromRight = (vRight - TurretConfig.CENTER_RIGHT) * TurretConfig.SCALE_RIGHT;
        double angleFromLeft = -90.0 + (vLeft - TurretConfig.V_LEFT_AT_NEG90) * TurretConfig.SCALE_LEFT;

        // Clean latching handoff
        if (!latchedToLeft) {
            if (angleFromRight < -135.0) latchedToLeft = true;
        } else {
            if (angleFromLeft > -125.0) latchedToLeft = false;
        }

        return latchedToLeft ? angleFromLeft : angleFromRight;
    }

    public void update() {
        currentAngleDeg = readFusedAngle();

        if (!TurretConfig.ENABLED) {
            stop();
            return;
        }

        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0 || dt > 0.1) dt = 0.02;

        double clampedTarget = Range.clip(targetAngleDeg, TurretConfig.MIN_ANGLE_DEG, TurretConfig.MAX_ANGLE_DEG);
        double error = clampedTarget - currentAngleDeg;

        // PID
        double pTerm = TurretConfig.kP * error;

        integralSum += error * dt;
        double maxInt = 0.2 / (TurretConfig.kI > 1e-6 ? TurretConfig.kI : 1.0);
        integralSum = Range.clip(integralSum, -maxInt, maxInt);
        double iTerm = TurretConfig.kI * integralSum;

        double dTerm = TurretConfig.kD * ((error - lastError) / dt);
        lastError = error;

        double output = pTerm + iTerm + dTerm;

        // Friction feedforward (kS)
        if (Math.abs(error) > TurretConfig.DEADBAND_DEG) {
            output += Math.signum(error) * TurretConfig.kS;
        } else {
            output = 0.0;
        }

        output = Range.clip(output, -TurretConfig.MAX_POWER, TurretConfig.MAX_POWER);

        if (currentAngleDeg <= TurretConfig.MIN_ANGLE_DEG && output < 0) output = 0;
        if (currentAngleDeg >= TurretConfig.MAX_ANGLE_DEG && output > 0) output = 0;

        currentPower = output;

        double leftPwr = TurretConfig.INVERT_LEFT ? -output : output;
        double rightPwr = TurretConfig.INVERT_RIGHT ? -output : output;

        turretLeft.setPower(leftPwr);
        turretRight.setPower(rightPwr);
    }
}