package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

// REV Through-Bore absolute encoder in analog mode: outputs 0..maxVoltage over a full 0..2π revolution.
public class AnalogEncoder {
    private final AnalogInput encoder;
    private double offsetRad = 0.0;
    private boolean reversed = false;

    public AnalogEncoder(HardwareMap hwMap, String name) {
        encoder = hwMap.get(AnalogInput.class, name);
    }

    // Returns module heading wrapped to (-π, π].
    public double getAngleInRad() {
        double frac = encoder.getVoltage() / encoder.getMaxVoltage();
        double raw = frac * 2.0 * Math.PI;
        if (reversed) raw = (2.0 * Math.PI) - raw;
        double shifted = raw - offsetRad;
        return Math.atan2(Math.sin(shifted), Math.cos(shifted));
    }

    public void setOffset(double offsetRad) { this.offsetRad = offsetRad; }
    public void setReversed(boolean reversed) { this.reversed = reversed; }
}
