package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="Robot: Primary Auto", group="Robot")
public class RobotPrimaryAuto extends LinearOpMode {

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    // Current Drive setting
    private double drivePwr = 0;
    private double strafePwr = 0;
    private double yawPwr = 0;

    // Data relevent to getting our current pos
    private PositionFinder positionFinder = new PositionFinder();

    // Hardware
    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel
    public IMU imu;

    // The threshold for
    private static final double POSITION_THRESHOLD = 0.1;
    private static final double YAW_THRESHOLD = 0.01;


    // Move to position

    private boolean isMoveToPositionActive = false;

    private double targetX, targetY, targetYaw;

    public synchronized void setTargetPosition(double x, double y, double yaw) {
        this.targetX = x;
        this.targetY = y;
        this.targetYaw = yaw;
    }

    public synchronized double[] getTargetPosition() {
        return new double[]{targetX, targetY, targetYaw};
    }

    // Team Color Guess

    public String allianceColor = "red";
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        initializeHardware();
        // Let's get our position finder ready
        positionFinder.InitializePositionFinder(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                hardwareMap.get(IMU.class, "imu"),
                hardwareMap.get(DcMotor.class, "rightFront"), // X axis encoder, hooked up to motor 1
                hardwareMap.get(DcMotor.class, "leftFront")   // Y axis encoder, hooked up to motor 0
        );
        // Multithreading at it's finest
        Thread positionFinderThread = new Thread(new Runnable() {
            @Override
            public void run() {
                positionFinder.FindBotPosition();
            }
        });
        positionFinderThread.start();

        Thread moveToPositionThread = new Thread(new Runnable() {
            @Override
            public void run() {
                moveToPosition();
            }
        });

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)",
                    positionFinder.x,
                    positionFinder.y,
                    positionFinder.yaw));
            telemetry.addLine("Yaw offset: " + positionFinder.imuPosOffset);
            telemetry.addLine("IMU Yaw: " + positionFinder.imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.update(); // Push telemetry to the Driver Station.

            setTargetPosition(0, 0, 0); // Set the target position to the center of the field

//            if (gamepad1.right_bumper) { // TEMPORARY
//                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                double drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//                double strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//                double turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//                moveRobot(drive,strafe,turn);
//            }
//            if (gamepad1.left_bumper) {
//                moveRobot(0,0,-positionFinder.yaw/30);
//            }

            moveRobotInternal();
//
//            // Save CPU resources; can resume streaming when needed.
//            if (gamepad1.dpad_down) {
//                visionPortal.stopStreaming();
//            } else if (gamepad1.dpad_up) {
//                visionPortal.resumeStreaming();
//            }

            // Share the CPU.
            sleep(20);
        }

        // Stop the opmode gracefully
        positionFinder.isOpmodeRunning = false;
        positionFinder.OnOpmodeStopped();
    }   // end method runOpMode()

    private void moveBotTowardsCenter() {
        moveRobot(-positionFinder.x/3, -positionFinder.y/3, 0);
    }

    private void moveToPosition() {
        while (opModeIsActive()) {
            if (isMoveToPositionActive) {
                double[] targetPosition = getTargetPosition();
                double targetX = targetPosition[0];
                double targetY = targetPosition[1];
                double targetYaw = targetPosition[2];

                double currentX = positionFinder.x;
                double currentY = positionFinder.y;
                double currentYaw = positionFinder.yaw;

                double distance = Math.hypot(targetX - currentX, targetY - currentY);
                double angleToTarget = calculateAngle(currentX, currentY, targetX, targetY);
                double angleError = normalizeAngle(angleToTarget - currentYaw);

                if (distance < POSITION_THRESHOLD && Math.abs(angleError) < YAW_THRESHOLD) {
                    break;
                }

                double drive = distance > POSITION_THRESHOLD ? distance / 3 : 0;
                double strafe = 0;
                double turn = angleError > YAW_THRESHOLD ? angleError / 3 : 0;

                moveRobot(drive, strafe, turn);
                moveRobotInternal();

                sleep(20);
            }
        }
    }

    private void initIMU() {

        // Retrieve and initialize the IMU.
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters;

        parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize((parameters));
    }

    private void initializeHardware() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    //    /**
//     * Add telemetry about AprilTag detections.
//     */
//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)",
//                        detection.robotPose.getPosition().x,
//                        detection.robotPose.getPosition().y,
//                        detection.robotPose.getPosition().z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//
//    }   // end method telemetryAprilTag()
    public void moveRobot(double x, double y, double yaw) {
        drivePwr = x;
        strafePwr = y;
        yawPwr = yaw;
    }
    public void moveRobotInternal() {
        // Calculate wheel powers.
        double leftFrontPower    =   drivePwr -strafePwr -yawPwr;
        double rightFrontPower   =   drivePwr +strafePwr +yawPwr;
        double leftBackPower     =  -drivePwr -strafePwr +yawPwr;
        double rightBackPower    =  -drivePwr +strafePwr -yawPwr;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
    // MATH!!!!!

    // Method to compute the angle between current and target positions
    private double calculateAngle(double currentX, double currentY, double targetX, double targetY) {
        return Math.atan2(targetY - currentY, targetX - currentX);
    }

    // Method to normalize angle between -PI and PI
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

}   // end class
