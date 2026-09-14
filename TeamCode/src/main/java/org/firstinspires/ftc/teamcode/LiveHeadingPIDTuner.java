package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp
public class LiveHeadingPIDTuner extends LinearOpMode {
    private IMU imu;
    private double lastLoopTime;
    private double x;
    private double y;
    private double rx;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // PID Coefficients
    private double Kp = 0.0015;
    private double Ki = 0.0005;
    private double Kd = 0.001;
    private double Kf = 0.05;

    //tuning step size
    private final double delta = 0.0001;

    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        //init PID
        PIDController pidController = new PIDController(Kp, Ki, Kd);
        pidController.setAngleTarget(90);

        // Motor initialization
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime runtime = new ElapsedTime();
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double currentTime = runtime.seconds();
            double deltaTime = currentTime - lastLoopTime;
            lastLoopTime = currentTime;

            if(gamepad1.left_stick_button) {
                imu.resetYaw();
            }

            double pidOutput = pidController.calculateHeadingOutput(getHeadingDegrees(), deltaTime);
            // APPLY FEEDFORWARD (KF)
            double correction = pidOutput;
            // Only apply Feedforward if the PID is commanding movement above a small threshold (0.01).
            if (Math.abs(pidOutput) > 0.01) {
                // Add KF in the direction of the correction (using Math.copySign).
                correction += Math.copySign(Kf, pidOutput);
            }

            y = 0;
            x = 0;
            rx = correction;

            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad1.dpad_up)    { Kp += delta; sleep(150); }
            if (gamepad1.dpad_down)  { Kp = Math.max(0, Kp - delta); sleep(150); }
            if (gamepad1.dpad_right) { Ki += delta; sleep(150); }
            if (gamepad1.dpad_left)  { Ki = Math.max(0, Ki - delta); sleep(150); }
            if (gamepad1.y)          { Kd += delta; sleep(150); }
            if (gamepad1.a)          { Kd = Math.max(0, Kd - delta); sleep(150); }

            pidController.setPID(Kp, Ki, Kd);  // Update PID with new values
            turn(rx);

            // Telemetry
            telemetry.addData("Current Heading", getHeadingDegrees());
            telemetry.addData("Correction (PID + KF)", correction);
            telemetry.addData("Target Angle", pidController.getTarget());
            telemetry.addData("PID Output", pidOutput);
            telemetry.addData("Kp", Kp);
            telemetry.addData("Ki", Ki);
            telemetry.addData("Kd", Kd);
            telemetry.update();
        }
    }

    private void turn(double power) {
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        rx = -rx;
        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeft.setPower((rotY - rotX + rx) / denominator);
        backLeft.setPower((rotY + rotX + rx) / denominator);
        frontRight.setPower((rotY - rotX - rx) / denominator);
        backRight.setPower((rotY + rotX - rx) / denominator);
    }
    private double getHeadingDegrees() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
}

