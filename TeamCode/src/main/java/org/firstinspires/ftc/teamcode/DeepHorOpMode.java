package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public abstract class DeepHorOpMode extends OpMode {
    //region Hardware
    //region Drive Motors
    DcMotor motorFL = null; // Front Left
    public DcMotor motorFR = null; // Front Right
    public DcMotor motorBL = null; // Back Left
    public DcMotor motorBR = null; // Back Right
    //endregion

    //region Slide
    public DcMotor motorSlide = null;
    public DcMotor motorSwing = null;
    public Servo clawServo = null;
    //endregion
    //region Arm
    /**
     * Motor attached to the bot to move the arm containing the main arm mechanism.
     */
    public DcMotor arm_BigHorizontal = null;
    /**
     * The first servo on the arm. Swivels the main scoop mechanism off the arm.
     */
    public Servo arm_SmallHorizontal = null;
    /**
     * The second servo on the arm. Moves the rest of the scoop up and down.
     */
    public Servo arm_VerticalServo = null;
    /**
     * The third servo on the arm. Controls the wheel to scoop the game pieces.
     */
    public CRServo arm_Scoop = null;

    public double armSmallHorizontal = 0.52;
    public double armVertical = 0.57;

    public int armTarget = 0;
    //endregion
    //endregion

    //region Drive

    // Current Drive setting
    private double drivePwr = 0;
    private double strafePwr = 0;
    private double yawPwr = 0;

    //endregion

    public void ConfigureHardware() {
        //region Drive Motors

        this.motorFL = this.hardwareMap.get(DcMotor.class, "leftFront");
        this.motorFR = this.hardwareMap.get(DcMotor.class, "rightFront");
        this.motorBL = this.hardwareMap.get(DcMotor.class, "leftBack");
        this.motorBR = this.hardwareMap.get(DcMotor.class, "rightBack");

        this.motorFL.setDirection(DcMotor.Direction.REVERSE);
        this.motorBL.setDirection(DcMotor.Direction.REVERSE);
        this.motorFR.setDirection(DcMotor.Direction.FORWARD);
        this.motorBR.setDirection(DcMotor.Direction.FORWARD);

        //endregion

        //region Slide
        this.motorSlide = this.hardwareMap.get(DcMotor.class, "slideMotor");

        this.motorSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.motorSwing = this.hardwareMap.get(DcMotor.class, "swing");
        this.motorSwing.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        this.clawServo = this.hardwareMap.get(Servo.class, "claw");


        //endregion

        //region Arm Hardware
        this.arm_BigHorizontal = this.hardwareMap.get(DcMotor.class, "bigHorizontal");
        this.arm_SmallHorizontal = this.hardwareMap.get(Servo.class, "RotHori");
        this.arm_VerticalServo = this.hardwareMap.get(Servo.class, "RotVert");
        this.arm_Scoop = this.hardwareMap.get(CRServo.class, "Spinny");


        //endregion

    }

    public void MoveRobot(double x, double y, double yaw) {
        drivePwr = x;
        strafePwr = y;
        yawPwr = yaw;
    }

    public void UpdateMoveRobot(double speedLimit) {
        // Calculate wheel powers.
        double leftFrontPower    =   drivePwr -strafePwr -yawPwr;
        double rightFrontPower   =   drivePwr +strafePwr +yawPwr;
        double leftBackPower     =  -drivePwr -strafePwr +yawPwr;
        double rightBackPower    =  -drivePwr +strafePwr -yawPwr;

        // Normalize wheel powers to be less than 1.0
        double max = (Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        motorFL.setPower(leftFrontPower*speedLimit);
        motorFR.setPower(rightFrontPower*speedLimit);
        motorBL.setPower(leftBackPower*speedLimit);
        motorBR.setPower(rightBackPower*speedLimit);
    }
}
