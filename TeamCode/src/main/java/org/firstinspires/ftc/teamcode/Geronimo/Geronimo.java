package org.firstinspires.ftc.teamcode.Geronimo;
import static org.firstinspires.ftc.teamcode.Geronimo.MecanumDrive_Geronimo.PARAMS;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


@Config
public class Geronimo {
    LinearOpMode opMode;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;

    DcMotor leftRotater;
    DcMotor rightRotater;
    int rotatorTargetPosition = 0;
    double rotatorArmPower = 0.0;
    boolean rotatorArmRunningToPosition = false;
    public static final int ROTATOR_ARMS_MIN_POS = 0;
    public static final int ROTATOR_ARMS_MAX_POS = 1000;
    public static final double ROTATOR_ARMS_POS_POWER = 0.75;
    int slidesTargetPosition = 0;
    boolean slidesRunningToPosition = false;
    public static final double SLIDES_POS_POWER = 0.75;


    DcMotor slideLeft;
    DcMotor slideRight;
    IMU imu;

    Servo clawServo;
    public static final double CLAW_MAX_POS = 0.45;
    public static final double CLAW_MIN_POS = 0.0;
    double claw_position = 0.5;

    Servo intakeRotater;
    public static final double INTAKE_ROTATOR_MAX_POS = 1.0;
    public static final double INTAKE_ROTATOR_MIN_POS = 0.0;
    public static final double INTAKE_ROTATOR_INCREMENT = 0.05;
    double intakeRotatorPosition = 0.5;

    Servo intakeHangerLeft;
    Servo intakeHangerRight;
    public static final double HANGER_MAX_POS = 0.95;
    public static final double HANGER_MIN_POS = 0.0;
    double intakeHangerLeftPosition = 0.5;
    double intakeHangerRightPosition = 0.5;
    static final double HANGER_INCREMENT   = 0.05;

    public CRServo intakeStar;
    double intakeStarPower = 0.0;

    TouchSensor touchSensorRight;
    TouchSensor touchSensorLeft;
    TouchSensor touchSensorRotator;

    public double imuOffsetInDegrees;

    RevBlinkinLedDriver.BlinkinPattern Blinken_left_pattern;
    RevBlinkinLedDriver.BlinkinPattern Blinken_right_pattern;
    RevBlinkinLedDriver blinkinLedDriverLeft;
    RevBlinkinLedDriver blinkinLedDriverRight;

    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;

    boolean IsDriverControl;
    boolean IsFieldCentric;

    //NAV TO TAG VARIABLES
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    public static int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    double rangeError = 0;
    double yawError = 0;
    private double slidePower = 0.0;
    int redLeft = 0;
    int greenLeft = 0;
    int blueLeft = 0;
    int redRight = 0;
    int greenRight = 0;
    int blueRight = 0;
    double LimelightMountingHeight = 6;  //Adjust when robot is built
    double LimelightMountingAngle = Math.toDegrees(90);
    double distance_to_object = 0;
    double objectHeight = 0;
    double XDistance_to_object = 0;
    double YDistance_to_object = 0;
    double angletoObject = Math.toRadians(60);
    public static boolean allianceColorIsBlue = false;
    public static double autoTimeLeft = 0.0;

    //LimeLight
    Limelight3A limelightbox;
    private static final double KpDistance = -0.1; // Proportional control constant for distance adjustment
    private static final double KpAim = 0.1; // Proportional control constant for aiming adjustment



    // REV v3 color sensor variables
    public enum colorEnum {
        noColor,
        red,
        yellow,
        blue;
    }
    colorEnum colorDetected = colorEnum.noColor;

    public Geronimo(boolean isDriverControl, boolean isFieldCentric, LinearOpMode opMode) {
        this.IsDriverControl = isDriverControl;
        this.IsFieldCentric = isFieldCentric;
        this.opMode = opMode;
    }
    public void Init (HardwareMap hardwareMap) {
        // ************* Drive MOTORS ****************
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront_leftOdometry");
        leftRear= hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear_rightOdometry");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront_centerOdometry");

        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // ************* Rotator ARM MOTORS ****************
        leftRotater = hardwareMap.get(DcMotorEx.class, "leftRotater");
        rightRotater = hardwareMap.get(DcMotorEx.class, "rightRotater");

        leftRotater.setDirection(DcMotor.Direction.FORWARD);
        rightRotater.setDirection(DcMotorSimple.Direction.REVERSE);

        leftRotater.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        rightRotater.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        leftRotater.setPower(0.0);
        rightRotater.setPower(0.0);
        leftRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotater.setTargetPosition(rotatorTargetPosition);
        leftRotater.setTargetPosition(rotatorTargetPosition);
        leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ************* Slide MOTORS ****************
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");

        slideLeft.setDirection(DcMotor.Direction.REVERSE);
        slideRight.setDirection(DcMotorSimple.Direction.FORWARD);

        slideLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setPower(0.0);
        slideRight.setPower(0.0);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // ********** Servos ********************
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        intakeRotater = hardwareMap.get(Servo.class, "intakeRotater");
        intakeHangerLeft = hardwareMap.get(Servo.class, "intakeHangerLeft");
        intakeHangerRight = hardwareMap.get(Servo.class, "intakeHangerRight");
        intakeStar = hardwareMap.get(CRServo.class, "intakeStar");

        // ********** Color Sensors ********************
        /*
        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");
        rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorRight");
        leftColorSensor.enableLed(false);
        rightColorSensor.enableLed(false);
        */
        // ********** Touch Sensors ********************

        touchSensorRight = hardwareMap.get(TouchSensor.class, "sensor_touchRight");
        touchSensorLeft = hardwareMap.get(TouchSensor.class, "sensor_touchLeft");
        touchSensorRotator = hardwareMap.get(TouchSensor.class, "sensor_touchRotate");

        // limelightbox = hardwareMap.get(Limelight3A.class, "limelight");
        InitBlinkin(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        imu.initialize(parameters);
        imu.resetYaw();

    }

    public void WebcamInit (HardwareMap hardwareMap){
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)
        /*aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();*/
        // Create the TensorFlow processor the easy way.
        // = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.

        //     visionPortal = VisionPortal.easyCreateWithDefaults(
        //             hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
    }
    public void InitLimelight(HardwareMap hardwareMap){
        limelightbox = hardwareMap.get(Limelight3A.class, "limelight");
        limelightbox.pipelineSwitch(0);
        limelightbox.start();
        //limelightbox.getLatestResult().getTx();
       // limelightbox.getLatestResult().getTy();
       // angletoObject = LimelightMountingAngle + (limelightbox.getLatestResult().getTy());
        //distance_to_object = (objectHeight-LimelightMountingHeight)/(Math.tan(angletoObject));
        // Equation above was pulled from the Limelight documentation online
       // XDistance_to_object = distance_to_object*Math.cos((limelightbox.getLatestResult().getTx()));
       // YDistance_to_object = distance_to_object*Math.sin((limelightbox.getLatestResult().getTx()));
}

    public void NavToTag(){
        desiredTag  = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == DESIRED_TAG_ID) ){
                desiredTag = detection;
                rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                yawError        = desiredTag.ftcPose.yaw;
                break;  // don't look any further.
            } else {
                opMode.telemetry.addData("Unknown Target - ", "No Tag Detected");
                // set some range and yaw error
            }
        }
    }


    public List<Double>  AlignToTargetImage(String targetImageName) {
        List<Double> offset = new ArrayList<>();

        //Aligns the Robot to the Target Image horiontally and return True if success else False
        LLResult result = limelightbox.getLatestResult();

        if (result.isValid()) {
            // Access detector results
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            for (LLResultTypes.DetectorResult dr : detectorResults) {
                String BoxType = dr.getClassName();

                // Confirm once that Target Image is Found, before attempting alignment
                if (BoxType.equals(targetImageName)) {
                    double currentOffsetX = result.getTx() ;
                    double currentOffsetY = result.getTy() ;

                    //Return both offsets as a list
                    offset.add(currentOffsetX);
                    offset.add(currentOffsetY);
                    return offset;
                }
            }
        }
        //If the target is not found, add -1.0 to both offsets
        offset.add(-1.0);
        offset.add(-1.0);
        return offset;
    }

    public double GetStrafeOffsetInInches(String targetImageName) {

        List<Double> scaledOffsets = AlignToTargetImage(targetImageName);

        // Check if target was found
        if (scaledOffsets.get(0) == -1.0 && scaledOffsets.get(1) == -1.0) {
            // Target not found; propagate the -1.0 flags
            return 0;
        }

        // Convert scaled offsets back to raw angles
        double rawTx = scaledOffsets.get(0);;
        double rawTy = scaledOffsets.get(1);;

        // Fixed distance from the target in inches guaranteed by roadrunner
        final double D = 6.0;

        // Convert angles from degrees to radians for trigonometric functions
        double txRadians = Math.toRadians(rawTx);
        double tyRadians = Math.toRadians(rawTy);

        // H1 -- height of camera, H2,.. height of object.
        // Calculate strafing distances
        double strafeX = D * Math.tan(txRadians); // Left/Right adjustment
        double strafeY = D * Math.tan(tyRadians); // Forward/Backward adjustment

        // Create a new list to hold the strafing distances
        List<Double> strafeOffsetsInInches = new ArrayList<>();
        strafeOffsetsInInches.add(strafeX);
        strafeOffsetsInInches.add(strafeY);

        return strafeOffsetsInInches.get(0);
    }

    public boolean limelightHasTarget() {
        LLResult result = limelightbox.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
            for (LLResultTypes.ColorResult colorResult : colorResults) {
                // Example: Check if Limelight sees a color target
                if (colorResult.getTargetXDegrees() != 0 && colorResult.getTargetYDegrees() != 0) {
                    opMode.telemetry.addData("Color Detected", "X: %.2f, Y: %.2f", colorResult.getTargetXDegrees(), colorResult.getTargetYDegrees());
                    return true;
                }
            }
        }
        return false;
    }

    public boolean limelightHasCustomTarget() {
        LLResult result = limelightbox.getLatestResult();
        if (result != null && result.isValid()) {
            List<LLResultTypes.DetectorResult> detectorResults = result.getDetectorResults();
            if (detectorResults != null && !detectorResults.isEmpty()) {
                for (LLResultTypes.DetectorResult detectorResult : detectorResults) {
                    // Optionally, check for a specific class ID if you have multiple objects
                    //int classID = detectorResult.getClassID();
                    double confidence = detectorResult.getConfidence();
                    //opMode.telemetry.addData("Object Detected", "Class ID: %d, Confidence: %.2f", classID, confidence);
                    // If you want to check for a specific object, replace 'YOUR_CUSTOM_OBJECT_CLASS_ID' with your object's class ID
                    // if (classID == YOUR_CUSTOM_OBJECT_CLASS_ID) {
                    return true;
                    // }
                }
            }
        }
        return false;
    }

    protected void InitColorRevV3Sensor() {
        float gain = 51;
        final float[] hsvValues = new float[3];
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;
        if (leftColorSensor instanceof SwitchableLight) {
            ((SwitchableLight) leftColorSensor).enableLight(true);
        }
    }

    // colorFound loop
    public boolean ColorRevV3SensorChecker(colorEnum targetColor) {
        boolean colorFound = false;
        double breakLoop = 10 + opMode.getRuntime();
        while (!colorFound && opMode.getRuntime() <= breakLoop) {
            ColorRevV3Sensor();
            if (colorDetected == targetColor) {
                colorFound = true;
            }
            opMode.sleep(100);
        }
        return colorFound;
    }

    // returns colorEnum color detected
    public colorEnum ColorRevV3Sensor() {
        NormalizedRGBA colors = leftColorSensor.getNormalizedColors();
        float[] hsvValues = {0,0,0};
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Red HSV Color ranges
        double hMinRed = 19.811;
        double hMaxRed = 128.000;
        double sMinRed = 0.397;
        double sMaxRed = 0.671;
        double vMinRed = 0.263;
        double vMaxRed = 0.722;

        // Yellow HSV Color Values
        double hMinYellow = 59.000;
        double hMaxYellow = 113.043;
        double sMinYellow = 0.501;
        double sMaxYellow = 0.702;
        double vMinYellow = 0.565;
        double vMaxYellow = 1.000;

        // Blue HSV Color Values
        double hMinBlue = 187.152;
        double hMaxBlue = 219.568;
        double sMinBlue = 0.741;
        double sMaxBlue = 0.832;
        double vMinBlue = 0.514;
        double vMaxBlue = 1.000;

        // determine if color is blue, red or yellow and show telemetry
        if (hsvValues[0] >= hMinBlue && hsvValues[1] >= sMinBlue)
            colorDetected = colorEnum.blue;

        else if (hsvValues[1] <= sMaxYellow && hsvValues[1] >= sMinYellow && hsvValues[0] <= hMaxYellow && hsvValues[0] >= hMinYellow)
            colorDetected = colorEnum.yellow;

        else if (hsvValues[1] <= sMaxRed && hsvValues[1] >= sMinRed && hsvValues[0] <= hMaxRed && hsvValues[0] >= hMinRed)
            colorDetected = colorEnum.red;

        else
            colorDetected = colorEnum.noColor;

        return colorDetected;
    }

    public void DriveToTag() {
        double drive = 0.0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0.0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0.0;        // Desired turning power/speed (-1 to +1)

        // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
        NavToTag();
        double headingError = desiredTag.ftcPose.bearing;

        double timeout = opMode.getRuntime() + 5;

        while ((rangeError > DESIRED_DISTANCE) &&
                ((headingError > 2.0) || (headingError < -2.0)) &&
                ((yawError > 2.0) || (yawError < -2.0)) && (opMode.getRuntime() < timeout)) {
            // Determine heading, range and Yaw (tag image rotation) errors
            NavToTag();
            headingError = desiredTag.ftcPose.bearing;

            // Use the speed and turn "gains" to calculate how we want the robot to move.
            // Speed has been reduced for the AirShow on June 9th
            drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
            strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

            opMode.telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            opMode.telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
            opMode.telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            opMode.telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            opMode.telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
            opMode.telemetry.update();

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn);
            opMode.sleep(10);
        }
    }
   public void setBlinken_to5Volt()
    {
        blinkinLedDriverLeft.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1625));
        blinkinLedDriverRight.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1625));
    }
    public void InitBlinkin(HardwareMap hardwareMap) {
        blinkinLedDriverLeft = hardwareMap.get(RevBlinkinLedDriver.class,"leftBlinkin");
        blinkinLedDriverRight = hardwareMap.get(RevBlinkinLedDriver.class,"rightBlinkin");

        /*
        Servo fakeBlinkinLeft = hardwareMap.get(Servo.class, "leftBlinkin");
        Servo fakeBlinkinRight = hardwareMap.get(Servo.class, "rightBlinkin");
        fakeBlinkinLeft.getController().setServoPosition(fakeBlinkinLeft.getPortNumber(), 2125);
        fakeBlinkinRight.getController().setServoPosition(fakeBlinkinRight.getPortNumber(), 2125);
        opMode.sleep(100);
        */


        //setBlinken_to5Volt();
        //leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");

        if(allianceColorIsBlue){
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }else {
            Blinken_left_pattern  = RevBlinkinLedDriver.BlinkinPattern.RED;
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
    }

    // Servo Methods
    public void SetClawPosition(double position)
    {
        if (position > CLAW_MAX_POS)
        {
            claw_position = CLAW_MAX_POS;
        }
        else if (position < CLAW_MIN_POS)
        {
            claw_position = CLAW_MIN_POS;
        }
        else
        {
            claw_position = position;
        }
        clawServo.setPosition(claw_position);
    }

    public void SetIntakeStarRotatorPosition(double position)
    {
        if (position > INTAKE_ROTATOR_MAX_POS)
        {
            intakeRotatorPosition = INTAKE_ROTATOR_MAX_POS;
        }
        else if (position < INTAKE_ROTATOR_MIN_POS)
        {
            intakeRotatorPosition = INTAKE_ROTATOR_MIN_POS;
        }
        else {
            intakeRotatorPosition = position;
        }
        intakeRotater.setPosition(intakeRotatorPosition);
    }

    public void IntakeStarRotatorIncrementUp()
    {
        intakeRotatorPosition += INTAKE_ROTATOR_INCREMENT;
        SetIntakeStarRotatorPosition(intakeRotatorPosition);
    }

    public void IntakeStarRotatorDecrementDown()
    {
        intakeRotatorPosition -= INTAKE_ROTATOR_INCREMENT;
        SetIntakeStarRotatorPosition(intakeRotatorPosition);
    }

    public void SetHangerMaxUp()
    {
        intakeHangerRightPosition = HANGER_MAX_POS;
        intakeHangerLeftPosition = HANGER_MIN_POS;

        intakeHangerLeft.setPosition(intakeHangerLeftPosition);
        intakeHangerRight.setPosition(intakeHangerRightPosition);
    }
    public void SetHangerMaxDown()
    {
        intakeHangerRightPosition = HANGER_MIN_POS;
        intakeHangerLeftPosition = HANGER_MAX_POS;

        intakeHangerLeft.setPosition(intakeHangerLeftPosition);
        intakeHangerRight.setPosition(intakeHangerRightPosition);
    }
    public void HangerIncrementUp()
    {
        intakeHangerRightPosition += HANGER_INCREMENT;
        SetHangerPosition(intakeHangerRightPosition);
    }
    public void HangerDecrementDown()
    {
        intakeHangerRightPosition -= HANGER_INCREMENT;
        SetHangerPosition(intakeHangerRightPosition);
    }
    public void SetHangerPosition(double position)
    {
        if (position > HANGER_MAX_POS)
        {
            intakeHangerRightPosition = HANGER_MAX_POS;
            intakeHangerLeftPosition = HANGER_MIN_POS;
        }
        else if (position < HANGER_MIN_POS) {
            intakeHangerRightPosition = HANGER_MIN_POS;
            intakeHangerLeftPosition = HANGER_MAX_POS;
        }
        else {
            intakeHangerRightPosition = position;
            intakeHangerLeftPosition = HANGER_MAX_POS - position;
        }

        intakeHangerLeft.setPosition(intakeHangerLeftPosition);
        intakeHangerRight.setPosition(intakeHangerRightPosition);
    }

    public void SetIntakeStarPower(double power)
    {
        intakeStarPower = power;
        intakeStar.setPower(intakeStarPower);
    }

    public void SetSlidesPower (double power)
    {
        slidePower = power;
        slideLeft.setPower(slidePower);
        slideRight.setPower(slidePower);
    }

    public void SetSlideToPosition (int position)
    {
        slidesTargetPosition = position;
        slidesRunningToPosition = true;
        slideLeft.setTargetPosition(slidesTargetPosition);
        slideRight.setTargetPosition(slidesTargetPosition);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidePower = SLIDES_POS_POWER;
        slideLeft.setPower(slidePower);
        slideRight.setPower(slidePower);
    }

    public void IntakeFromFloor()
    {
        SetSlideToPosition(-695);
        SpecialSleep(500); //reduce in future
        SetRotatorToPosition(0);
        SpecialSleep(250);
        SetHangerPosition(0.7);
        SetIntakeStarRotatorPosition(1);
    }

    public void AutoStartPosition()
    {
        SetIntakeStarRotatorPosition(0.9);
        SetHangerPosition(0);
        SetSlideToPosition(0);
        SetRotatorToPosition(0);
    }

    public void SpecimenDeliverLow(){
        SetIntakeStarRotatorPosition(0.75);
        SetHangerPosition(0.1);
        SetSlideToPosition(-695);
        SetRotatorToPosition(250); //subject to change
    }

    public void SpecimenDeliverHigh(){
        SetIntakeStarRotatorPosition(0.75);
        SetHangerPosition(0.1);
        SetSlideToPosition(-750);
        SetRotatorToPosition(450); //subject to change
    }

    public void SpecimenPickupFromWall(){
        SetIntakeStarRotatorPosition(0.75);
        SetHangerPosition(0.1);
        SetSlideToPosition(-695);
        SetRotatorToPosition(180); //subject to change
    }

    public void BasketHigh(){
        SetIntakeStarRotatorPosition(1);
        SetHangerPosition(0.6);
        SetSlideToPosition(-850);
        SetRotatorToPosition(450);
    }

    public void BasketLow(){
        SetIntakeStarRotatorPosition(1);
        SetHangerPosition(0.6);
        SetSlideToPosition(-700);
        SetRotatorToPosition(450);
    }




    public void SetRotatorArmPower (double power)
    {
        rotatorArmPower = power;
        rotatorArmRunningToPosition = false;
        leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void SetRotatorToPosition (int position)
    {
        rotatorTargetPosition = position;
        rotatorArmRunningToPosition = true;
        rightRotater.setTargetPosition(rotatorTargetPosition);
        leftRotater.setTargetPosition(rotatorTargetPosition);
        leftRotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRotater.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotatorArmPower = ROTATOR_ARMS_POS_POWER;
        leftRotater.setPower(rotatorArmPower);
        rightRotater.setPower(rotatorArmPower);
    }
    public void SetRotatorArmHoldPositon()
    {
        if (rotatorTargetPosition == 0) {
            rotatorArmPower = 0.0;
            leftRotater.setPower(rotatorArmPower);
            rightRotater.setPower(rotatorArmPower);
            leftRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRotater.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else {
            rotatorArmPower = 0.25;
            leftRotater.setPower(rotatorArmPower);
            rightRotater.setPower(rotatorArmPower);
        }



        //rotatorArmRunningToPosition = false;
    }
    public boolean GetRotatorArmRunningToPosition()
    {
        return rotatorArmRunningToPosition;
    }
    public int GetRotatorArmTargetPosition()
    {
        return rotatorTargetPosition;
    }
    public int GetRotatorLeftArmCurrentPosition()
    {
        return (leftRotater.getCurrentPosition());
    }
    public int GetRotatorRightArmCurrentPosition()
    {
        return (rightRotater.getCurrentPosition());
    }

    public void ShowTelemetry(){
        //opMode.telemetry.addData("Left Hopper: ", leftColorSensor.getDistance(DistanceUnit.MM));
        //opMode.telemetry.addData("Right Hopper: ", rightColorSensor.getDistance(DistanceUnit.MM));
        opMode.telemetry.addData("Auto Last Time Left: ", autoTimeLeft);
        opMode.telemetry.addData("imu Heading: ", GetIMU_HeadingInDegrees());
        opMode.telemetry.addData("Claw Position: ", claw_position);
        opMode.telemetry.addData(">", "Claw - use bumpers for control" );
        opMode.telemetry.addData("Hanger Right Position: ", intakeHangerRightPosition);
        opMode.telemetry.addData("Hanger Left Position: ", intakeHangerLeftPosition);
        opMode.telemetry.addData(">", "Hangers - use dpad for control" );
        opMode.telemetry.addData("Intake Star Power: ", intakeStarPower);
        opMode.telemetry.addData(">", "Intake Star - use triggers for control" );
        opMode.telemetry.addData("slideLeft Position: ", slideLeft.getCurrentPosition());
        opMode.telemetry.addData("slideRight Position: ", slideRight.getCurrentPosition());
        opMode.telemetry.addData("slide Power: ", slidePower);
        opMode.telemetry.addData(">", "slides - use leftStick Y for control" );
        opMode.telemetry.addData("leftRotater Position: ", leftRotater.getCurrentPosition());
        opMode.telemetry.addData("rightRotater Position: ", rightRotater.getCurrentPosition());
        opMode.telemetry.addData("Rotator Tgt Position: ", rotatorTargetPosition);
        opMode.telemetry.addData("Rotator Arm Power: ", rotatorArmPower);
        opMode.telemetry.addData("Rotator mode in RUN_TO_POSITION? ", rotatorArmRunningToPosition);
        opMode.telemetry.addData(">", "rotateArms - use rightStick Y, and buttons for control" );
        opMode.telemetry.addData("intake star ROTATOR Pos: ", intakeRotatorPosition);
        opMode.telemetry.addData(">", "intake star rotator - use G1 - dpad for control" );


        showColorSensorTelemetry();
        opMode.telemetry.update();
    }

    public void moveRobot(double x, double y, double yaw) {
        // opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
        //  opMode.telemetry.update();
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

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
        leftFront.setPower(leftFrontPower *.5);
        rightFront.setPower(rightFrontPower *.5);
        leftRear.setPower(leftBackPower *.5);
        rightRear.setPower(rightBackPower *.5);

        //  opMode.telemetry.addData("Claw Distance: ", clawDistanceSensor.getDistance(DistanceUnit.MM));
        //  opMode.telemetry.update();
    }
/*
    public void SlidesRotating(double slidePower){
        leftRotater.setPower(slidePower);
        rightRotater.setPower(slidePower);
    }
*/
    public void EndgameBuzzer(){
        if(opMode.getRuntime() < 109.5 && opMode.getRuntime() > 109.0){
            opMode.gamepad1.rumble(1000);
            opMode.gamepad2.rumble(1000);
        }
    }
    public void ColorDetect(){
        //double rightColor = rightColorSensor.getLightDetected();
    }
    public void showColorSensorTelemetry(){
        //int leftColor = leftColorSensor.getNormalizedColors().toColor();
        //opMode.telemetry.addData("leftColorNorm: ", leftColor);
        opMode.telemetry.addData("leftColor(red): ", redLeft);
        opMode.telemetry.addData("leftColor(green): ", greenLeft);
        opMode.telemetry.addData("leftColor(blue): ", blueLeft);
        opMode.telemetry.addData("rightColor(red): ", redRight);
        opMode.telemetry.addData("rightColor(green): ", greenRight);
        opMode.telemetry.addData("rightColor(blue): ", blueRight);
        //opMode.telemetry.addData("rightColor: ", rightColor);
        //opMode.telemetry.addData("leftColorNorm(red): ", leftColorSensor.getNormalizedColors().red);
        //opMode.telemetry.addData("leftColorNorm(green): ", leftColorSensor.getNormalizedColors().green);
        //opMode.telemetry.addData("leftColorNorm(blue): ", leftColorSensor.getNormalizedColors().blue);
        /*
        int red = leftColorSensor.red();
        int green = leftColorSensor.green();
        int blue = leftColorSensor.blue();
        // Check for White Pixel
        if(red < 4000 && red > 1000 && green < 6000 && green > 3000 && blue < 7000 && blue > 3000) {
            opMode.telemetry.addData("Left: ", "is white");
        }
        // Check for yellow pixel
        else if(red < 2500 && red > 1000 && green < 3500 && green > 1500 && blue < 1000 && blue >0 )
        {
            opMode.telemetry.addData("Left: ", "is yellow");
        }
        // Check for green pixel
        else if(red < 1000 && red > 0 && green < 6000 && green > 1500 && blue < 1000 && blue >0 )
        {
            opMode.telemetry.addData("Left: ", "is green");
        }
        // Check for purple pixel
        else if(red < 3500 && red > 1000 && green < 4000 && green > 2000 && blue < 7000 && blue > 3500 )
        {
            opMode.telemetry.addData("Left: ", "is purple");
        }
        else {
            opMode.telemetry.addData("Left: ", "unknown");
        }

         */
    }

    public void SetBlinkinToPixelColor() {
        redLeft = leftColorSensor.red();
        greenLeft = leftColorSensor.green();
        blueLeft = leftColorSensor.blue();
        redRight = rightColorSensor.red();
        greenRight = rightColorSensor.green();
        blueRight = rightColorSensor.blue();

        // Left sensor left blinkin
        if(redLeft > (blueLeft / 2) && greenLeft > redLeft && blueLeft > redLeft) {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
        else if(greenLeft > redLeft && redLeft > blueLeft) {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
        else if(greenLeft > (redLeft * 2) && greenLeft > (blueLeft * 2)) {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
        else if(blueLeft > greenLeft && greenLeft > redLeft) {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }
        else if(allianceColorIsBlue){
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }else {
            Blinken_left_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverLeft.setPattern(Blinken_left_pattern);
        }

        //Right sensor right blinkin
        if(redRight > (blueRight / 2) && greenRight > redRight && blueRight > redRight) {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        else if(greenRight > redRight && redRight > blueRight) {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        else if(greenRight > (redRight * 2) && greenRight > (blueRight * 2)) {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        else if(blueRight > greenRight && greenRight > redRight) {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.HOT_PINK;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
        else if(allianceColorIsBlue){
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }else {
            Blinken_right_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriverRight.setPattern(Blinken_right_pattern);
        }
    }
    public void ShowBlinkinTelemetry() {
        opMode.telemetry.addData("Blinkin Left: ", Blinken_left_pattern.toString());
        opMode.telemetry.addData("Blinkin Right: ", Blinken_right_pattern.toString());
    }

    public void driveControlsRobotCentric() {
        double y = -opMode.gamepad1.left_stick_y;
        double x = opMode.gamepad1.left_stick_x * 1.1;
        double rx = opMode.gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
    public void driveControlsRobotCentricKID() {
        double y = -opMode.gamepad2.left_stick_y;
        double x = opMode.gamepad2.left_stick_x * 1.1;
        double rx = opMode.gamepad2.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower*.25);
        leftRear.setPower(backLeftPower*.25);
        rightFront.setPower(frontRightPower*.25);
        rightRear.setPower(backRightPower*.25);
    }

    public double GetIMU_HeadingInDegrees()
    {
        double botHeading = AngleUnit.normalizeDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + imuOffsetInDegrees);

        return botHeading;

    }
    public void driveControlsFieldCentric() {
        double y = -opMode.gamepad1.left_stick_y;
        double x = opMode.gamepad1.left_stick_x;
        double rx = opMode.gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.setPower(frontLeftPower*1.0);
        leftRear.setPower(backLeftPower*1.0);
        rightFront.setPower(frontRightPower*1.0);
        rightRear.setPower(backRightPower*1.0);
    }
    public void RunDriveControls() {
        if (IsFieldCentric) {
            driveControlsFieldCentric();
        }
        else {
            driveControlsRobotCentric();
        }
    }
    public void SetFieldCentricMode(boolean fieldCentricEnabled) {
        IsFieldCentric = fieldCentricEnabled;
    }
    public void SpecialSleep(long milliseconds) {
        for (long stop = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(milliseconds); stop > System.nanoTime(); ) {
            if (!opMode.opModeIsActive() || opMode.isStopRequested()) return;
            if (IsDriverControl) {
                if (IsFieldCentric) driveControlsFieldCentric();
                if (!IsFieldCentric) driveControlsRobotCentric();
            }
        }
    }

}