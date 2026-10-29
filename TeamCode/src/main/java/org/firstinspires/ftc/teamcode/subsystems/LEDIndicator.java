package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.opmodes.BaseTeleOp.Alliance;

@Config
public class LEDIndicator {
    public static double COLOR_OFF = 0.00;
    public static double COLOR_RED = 0.29;      // Red Alliance
    public static double COLOR_GREEN = 0.52;    // Shooting
    public static double COLOR_BLUE = 0.62;     // Blue Alliance
    public static double COLOR_PURPLE = 0.70;   // Confirmed Full

    // 0.5s ON, 0.5s OFF (Clean 1 Hz blink)
    public static double BLINK_INTERVAL_SEC = 0.50;

    private final Servo indicator;
    private final ElapsedTime blinkTimer = new ElapsedTime();

    public LEDIndicator(HardwareMap hwMap) {
        indicator = hwMap.get(Servo.class, "LEDIndicator");
        setIdle();
    }

    public void setPosition(double pos) {
        indicator.setPosition(pos);
    }

    public void setIdle() {
        setPosition(COLOR_OFF);
    }

    public void setShooting() {
        setPosition(COLOR_GREEN);
    }

    public void setFull() {
        setPosition(COLOR_PURPLE);
    }

    /**
     * Non-blocking 0.5-second blink for the active alliance color while intaking.
     */
    public void blinkAlliance(Alliance alliance) {
        double color = (alliance == Alliance.BLUE) ? COLOR_BLUE : COLOR_RED;
        boolean lightOn = (blinkTimer.seconds() % (BLINK_INTERVAL_SEC * 2)) < BLINK_INTERVAL_SEC;
        setPosition(lightOn ? color : COLOR_OFF);
    }
}