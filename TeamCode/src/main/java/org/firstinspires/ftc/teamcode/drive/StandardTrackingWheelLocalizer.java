package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
//@Disabled
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 4000;
    public static double WHEEL_RADIUS = 0.984; // in  1.973" dia measured
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.04; // in; distance between the left and right wheels // 13.1944 2/3 bias // wheel to wheel:  // center to center: 13.0304
    public static double FORWARD_OFFSET = -4.17; // in; offset of the lateral wheel // 2/3 bias -4.2237 // unbiased -4.1443
    /* Lines 37-38 in StandardTrackingWheelLocalizer.java */
    public static double X_MULTIPLIER = 0.9965; // 0.9965 // Multiplier in the X direction
    public static double Y_MULTIPLIER = 0.9920; // 0.9920 // Multiplier in the Y direction

    public Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(-2.2391, LATERAL_DISTANCE / 2.0, 0), // left
                new Pose2d(-2.2391, -LATERAL_DISTANCE / 2.0, 0), // right
                new Pose2d(FORWARD_OFFSET, -2.3296, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "od_lf"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "od_rt"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "od_lat"));


        // TODO:  reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);

    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2.0 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                //   encoderTicksToInches(leftEncoder.getCurrentPosition()),
                //   encoderTicksToInches(rightEncoder.getCurrentPosition()),
                //   encoderTicksToInches(frontEncoder.getCurrentPosition())
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                //encoderTicksToInches(leftEncoder.getRawVelocity()),
                //encoderTicksToInches(rightEncoder.getRawVelocity()),
                //encoderTicksToInches(frontEncoder.getRawVelocity())
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
