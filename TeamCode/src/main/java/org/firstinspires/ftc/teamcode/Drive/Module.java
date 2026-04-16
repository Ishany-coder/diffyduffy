package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

//TODO: figure out how to turn and spin the wheel at the same time and apply motor flipping
@Config
public class Module {

    public static double KP = 0.1;
    public static double KI = 0.1;
    public static double KD = 0.1;
    private double lastError = 0.0;
    private ElapsedTime timer = new ElapsedTime();

    //when moto spin same turny
    //when moto spin diffy movey
    private final double motorRPM = 1150;
    private final double wheelDiameterInches = 10; // fix later
    private AnalogEncoder encoder;
    List<DcMotorEx> motors = new ArrayList<>();
    private boolean hasInit = false;
    private double i = 0;
    private double lastTimeNs = 0;
    public Module(HardwareMap hwMap, String motor1Name, String motor2Name, String encoderMame){
        motors.add(hwMap.get(DcMotorEx.class, motor1Name));
        motors.add(hwMap.get(DcMotorEx.class, motor2Name));
        encoder = new AnalogEncoder(hwMap, encoderMame);

        for(DcMotorEx motor : motors){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public double inchesPerSecToPower(double targetInchesPerSec) {

        double circumference = Math.PI * wheelDiameterInches;
        double targetRPM = (targetInchesPerSec * 60.0) / circumference;
        double power = targetRPM / motorRPM;

        // Clamp to valid motor power range
        return Math.max(-1.0, Math.min(1.0, power));
    }
    // defualt to deg
    public double getHeading(){
        return Math.toDegrees(encoder.getAngleInRad());
    }
    public double getHeadingRad(){
        return encoder.getAngleInRad();
    }

    public void update(double inPerSec, double targetAng){
        if(!hasInit){timer.reset();lastTimeNs = System.nanoTime();hasInit = true;}

        double currentAng = encoder.getAngleInRad();
        double error = Math.atan2(Math.sin(targetAng - currentAng), Math.cos(targetAng - currentAng));

        double drivePower = inchesPerSecToPower(inPerSec);

        if(Math.abs(Math.toDegrees(error)) > 90){
            error -= Math.signum(error) * Math.PI; // when rotating also flip the direction by 180 deg
            drivePower *= -1;
        }

        //PID
        double dt = (System.nanoTime() - lastTimeNs) / 1e9;
        if(dt <= 0) {
            Log.i("DIFFY SWERVE: ", "DT LESS THAN 0 WOULD FAIL");
            return;
        }

        lastTimeNs = System.nanoTime();

        double p = KP * error;
        i += (error * dt);
        i = Math.max(-1, Math.min(1, i));
        i *= KI;
        double d = KD * ((error - lastError)/dt);

        double steeringPower = p + i + d;

        double pos = drivePower + steeringPower;
        double neg = drivePower - steeringPower;

        double max = Math.max(1.0, Math.max(Math.abs(pos), Math.abs(neg)));

        pos /= max;
        neg /= max;

        motors.get(0).setPower(pos);
        motors.get(1).setPower(neg);


        lastError = error;
    }
}
