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
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;


@Config
public class Geronimo {
    IMU imu;
    public double imuOffsetInDegrees = 0.0;
    LinearOpMode opMode;
    public static boolean allianceColorIsBlue = false;
    public static double autoTimeLeft = 0.0;
    boolean IsDriverControl = true;
    boolean IsFieldCentric = true;

    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightFront;
    DcMotor rightRear;

    DcMotor leftSlideArmRotatorMotor;
    DcMotor rightSlideArmRotatorMotor;
    TouchSensor touchSensorSlideArmRotatorRight;
    TouchSensor touchSensorSlideArmRotatorLeft;
    int slideArmRotatorTargetPosition = 0;
    double slideArmRotatorPower = 0.0;
    boolean slideArmRotatorRunningToPosition = false;
    public static final int SLIDE_ARM_ROTATOR_MIN_POS = 0;
    public static final int SLIDE_ARM_ROTATOR_MAX_POS = 870;  //920
    public static final int SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES = 300; // TODO need to find the lowest rotator position we can allow the slides to go out
    public static final double SLIDE_ARM_ROTATOR_POWER = 0.75;

    //slides
    DcMotor slideLeft;
    DcMotor slideRight;
    int slidesTargetPosition = 0;
    boolean slidesRunningToPosition = false;
    public static final double SLIDES_POS_POWER = 1.0;
    public static final int SLIDE_ARM_MIN_POS = 0;
    public static final int SLIDE_ARM_MAX_VERTICAL_POS = 5618;
    public static final int SLIDE_ARM_MAX_HORIZONTAL_POS = 2900; //1550 //1400  //3690 //3310
    private double slidePower = 0.0;
    TouchSensor touchSensorSlideLeft;
    TouchSensor touchSensorSlideRight;

    Servo clawServo;
    public static final double CLAW_MAX_POS = 0.45;
    public static final double CLAW_MIN_POS = 0.0;
    double claw_position = 0.5;

    Servo swiperServo;
    public static final double SWIPER_MAX_POS = 0.8;
    public static final double SWIPER_MIN_POS = 0.25;
    private double swiper_position = 0.5;

    Servo intakeBoxRotaterServo;
    public static final double INTAKE_STAR_BOX_ROTATOR_MAX_POS = 1.0;
    public static final double INTAKE_STAR_BOX_ROTATOR_MIN_POS = 0.0;
    public static final double INTAKE_STAR_BOX_ROTATOR_INCREMENT = 0.005;
    double intakeBoxRotatorPosition = 0.5;

    Servo smallArmHangerLeftServo;
    Servo smallArmHangerRightServo;
    public static final double SMALL_ARM_HANGER_MAX_POS = 0.95;
    public static final double SMALL_ARM_HANGER_MIN_POS = 0.0;
    double smallArmHangerLeftPosition = 0.5;
    double smallArmHangerRightPosition = 0.5;
    static final double SMALL_ARM_HANGER_INCREMENT = 0.05;

  /*  public CRServo intakeStarServo;
    double intakeStarPower = 0.0;
    boolean intakeStarLastForward = false;

   */

    public Servo urchinServo;
    double urchinServo_position = 0.5;
    public static final double URCHIN_SERVO_MAX_POS = 1.0;
    public static final double URCHIN_SERVO_MIN_POS = 0.0;


    RevBlinkinLedDriver.BlinkinPattern Blinken_pattern;
    RevBlinkinLedDriver blinkinLedDriver;

    RevColorSensorV3 leftColorSensor;
    RevColorSensorV3 rightColorSensor;
    int redLeft = 0;
    int greenLeft = 0;
    int blueLeft = 0;
    int redRight = 0;
    int greenRight = 0;
    int blueRight = 0;


    // REV v3 color sensor variables
    public enum colorEnum {
        noColor,
        red,
        yellow,
        blue;
    }
    colorEnum colorDetected = colorEnum.noColor;

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
    double LimelightMountingHeight = 6;  //Adjust when robot is built
    double LimelightMountingAngle = Math.toDegrees(90);
    double distance_to_object = 0;
    double objectHeight = 0;
    double XDistance_to_object = 0;
    double YDistance_to_object = 0;
    double angletoObject = Math.toRadians(60);

    //LimeLight
    Limelight3A limelightbox;
    private static final double KpDistance = -0.1; // Proportional control constant for distance adjustment
    private static final double KpAim = 0.1; // Proportional control constant for aiming adjustment

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

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        // ************* Slide Arm Rotator MOTORS ****************
        leftSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "leftRotater");
        rightSlideArmRotatorMotor = hardwareMap.get(DcMotorEx.class, "rightRotater");

        leftSlideArmRotatorMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSlideArmRotatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlideArmRotatorMotor.setPower(0.0);
        rightSlideArmRotatorMotor.setPower(0.0);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideArmRotatorMotor.setTargetPosition(slideArmRotatorTargetPosition);
        leftSlideArmRotatorMotor.setTargetPosition(slideArmRotatorTargetPosition);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        // ********** Servos   ********************
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        intakeBoxRotaterServo = hardwareMap.get(Servo.class, "intakeRotater");
        smallArmHangerLeftServo = hardwareMap.get(Servo.class, "intakeHangerLeft");
        smallArmHangerRightServo = hardwareMap.get(Servo.class, "intakeHangerRight");
      //  intakeStarServo = hardwareMap.get(CRServo.class, "intakeStar");
        urchinServo = hardwareMap.get(Servo.class, "urchinServo");
        swiperServo = hardwareMap.get(Servo.class, "swiper");

        // ********** Color Sensors ********************

        leftColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorLeft");
        //rightColorSensor = hardwareMap.get(RevColorSensorV3.class, "ColorSensorRight");
        leftColorSensor.enableLed(false);
        //rightColorSensor.enableLed(false);


        // ********** Touch Sensors ********************
        touchSensorSlideArmRotatorRight = hardwareMap.get(TouchSensor.class, "sensor_touchRightRotator");
        touchSensorSlideArmRotatorLeft = hardwareMap.get(TouchSensor.class, "sensor_touchLeftRotator");
        touchSensorSlideLeft = hardwareMap.get(TouchSensor.class, "sensor_touchLeft");
        touchSensorSlideRight = hardwareMap.get(TouchSensor.class, "sensor_touchRight");

        // limelightbox = hardwareMap.get(Limelight3A.class, "limelight");
        InitBlinkin(hardwareMap);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                PARAMS.logoFacingDirection, PARAMS.usbFacingDirection));
        imu.initialize(parameters);
        imu.resetYaw();

    }

    // *********************************************************
    // ****      LIME LIGHT Methods                         ****
    // *********************************************************
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

    // *********************************************************
    // ****      Color Sensor Methods                       ****
    // *********************************************************

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
    /*
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
    */

    // returns colorEnum color detected
    float gain = 51;
    float[] hsvValues = {0,0,0};
    public colorEnum ColorRevV3Sensor() {
        leftColorSensor.setGain(gain);
        NormalizedRGBA colors = leftColorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Red HSV Color ranges
        // changed
        double hMinRed = 18.800;
        double hMaxRed = 128.000;
        double sMinRed = 0.3;
        double sMaxRed = 0.7235;
        double vMinRed = 0.263;
        double vMaxRed = 0.722;

        // Yellow HSV Color Values
        double hMinYellow = 59.000;
        double hMaxYellow = 113.043;
        double sMinYellow = 0.501;
        double sMaxYellow = 0.772;
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
        {
            colorDetected = colorEnum.blue;
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }


        else if (hsvValues[1] <= sMaxYellow && hsvValues[1] >= sMinYellow && hsvValues[0] <= hMaxYellow && hsvValues[0] >= hMinYellow)
        {
            colorDetected = colorEnum.yellow;
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }

        else if (hsvValues[1] <= sMaxRed && hsvValues[1] >= sMinRed && hsvValues[0] <= hMaxRed && hsvValues[0] >= hMinRed)
        {
            colorDetected = colorEnum.red;
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }

        else
        {
            colorDetected = colorEnum.noColor;
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }

        return colorDetected;

    }
    public void showColorSensorTelemetry(){
        //int leftColor = leftColorSensor.getNormalizedColors().toColor();
        //opMode.telemetry.addData("leftColorNorm: ", leftColor);
        //opMode.telemetry.addData("leftColor: ", "red: %d, green: %d, blue: %d", redLeft, greenLeft, blueLeft);
        //opMode.telemetry.addData("rightColor: ", "red: %d, green: %d, blue: %d", redRight, greenRight, blueRight);
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

    // *********************************************************
    // ****      BLINKIN LED Lights Controls                ****
    // *********************************************************

    public void InitBlinkin(HardwareMap hardwareMap) {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class,"RevBLinkinLedDriver");
        /*
        if(allianceColorIsBlue)
        {
            Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }
        else
        {
            Blinken_pattern  = RevBlinkinLedDriver.BlinkinPattern.RED;
            blinkinLedDriver.setPattern(Blinken_pattern);
        }
        */

        Blinken_pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        blinkinLedDriver.setPattern(Blinken_pattern);
    }

    public void setBlinken_to5Volt()
    {
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.fromNumber(1625));
    }

    // *********************************************************
    // ****      COMBO MOVES                                ****
    // *********************************************************

    public void AutoStartPosition()
    {
        SetIntakeBoxRotatorPosition(0.875); //0.875
        SetSmallArmHangerPosition(0.1); //0 //.25
        SetSlideToPosition(0);
        SetSlideRotatorArmToPosition(0);
        SetClawPosition(Geronimo.CLAW_MIN_POS);
        SetSwiperPosition(Geronimo.SWIPER_MAX_POS);
    }

    public void IntakeFromFloor()
    {
        /*
        SetSlideToPosition(1946); //695
        SpecialSleep(500); //reduce in future
        SetSlideRotatorArmToPosition(0);
        SpecialSleep(250);
        SetSmallArmHangerPosition(0.95); //0.7
        SetIntakeBoxRotatorPosition(1.085); //1

         */
    }

    public void SpecimenDeliverLow(){
        SetIntakeBoxRotatorPosition(0.525); //0.44
        SetSmallArmHangerPosition(0.6); //0.4 //0.65
    }

    public void RemoveFromWall(){
        RemoveFromWallServoPosition();

        double timeout = opMode.getRuntime() + 0.5;
        SetSlideToPosition(0);
        while (!GetSlidesLimitSwitchPressed() && opMode.getRuntime() < timeout) {
            SpecialSleep(50);
        }
        SetSlideRotatorArmToPosition(0);
    }

    public void RemoveFromWallServoPosition() {
        SetIntakeBoxRotatorPosition(0.96); //0.875
        SetSmallArmHangerPosition(.2); //0 //0.25
    }

    public void SpecimenDeliverHigh(){
        SetIntakeBoxRotatorPosition(0.58); //0.495)
        SetSmallArmHangerPosition(0.6); //.4 //0.6
        SetSlideToPosition(1680); //600
        SetSlideRotatorArmToPosition(323); //subject to change
    }

    public void SpecimenDeliverHighChamberAlternate(){
        SetIntakeBoxRotatorPosition(0.945); //0.82  //0.905
        SetSmallArmHangerPosition(.20); //0 //0.25
        SetSlideToPosition(1240);
        SetSlideRotatorArmToPosition(710);
    }

    public void SpecimenDeliverHighChamberFinishingMove(){
        SetIntakeBoxRotatorPosition(0.82); //0.945
        SetSmallArmHangerPosition(.20); //0 //0.25
        SetSlideToPosition(2350);///00
        SetSlideRotatorArmToPosition(710); //642
    }

    public void SpecimenPickupFromWall() {
        SpecimenPickupFromWallServoPosition();

        double timeout = opMode.getRuntime() + 0.5;
        SetSlideToPosition(0);
        while (!GetSlidesLimitSwitchPressed() && opMode.getRuntime() < timeout) {
            SpecialSleep(50);
        }
        SetSlideRotatorArmToPosition(0);
    }

    public void SpecimenPickupFromWallServoPosition(){
        SetIntakeBoxRotatorPosition(0.96); //0.875
        SetSmallArmHangerPosition(0.35); //.15 //0.4
    }

    public void SampleUrchinFloorPickup(){
        SetSlideToPosition(1945);
        SpecialSleep(300);
        SetIntakeBoxRotatorPosition(0.485);
        SetSmallArmHangerPosition(0.75); //.15 //0.80
        SetSlideRotatorArmToPosition(0);
    }

    public void SampleUrchinFloorPickupFinishingMove(){
        SetSlideToPosition(1945);
        SpecialSleep(300);
        SetIntakeBoxRotatorPosition(0.525);   //0.485
        SetSmallArmHangerPosition(0.8); //.15 //0.80
        SetSlideRotatorArmToPosition(0);
    }

    public void BasketHigh(){
        SetIntakeBoxRotatorPosition(0.935); //0.85
        SetSmallArmHangerPosition(1.); //.8 //1.05
        SetSlideRotatorArmToPosition(800); //8008, 450
        // wait for the rotators to move to vertical before raising slides
        SpecialSleep(2000);
        SetSlideToPosition(6496); //2320
    }
    // hanger position 0.8
    // .15 right
    //slides - -2320
    //rotator 8008
    //0.85 IntakeStarRotator, hanger position (0.15/0.8)

    public void BasketLow(){
        SetIntakeBoxRotatorPosition(1.085); //1
        SetSmallArmHangerPosition(0.8); //0.6 //0.85
        SetSlideToPosition(1300);
        SetSlideRotatorArmToPosition(450);
    }

    // *********************************************************
    // ****       Green Intake Box Controls                 ****
    // *********************************************************

    public void SetIntakeBoxRotatorPosition(double position)
    {
        if (position > INTAKE_STAR_BOX_ROTATOR_MAX_POS)
        {
            intakeBoxRotatorPosition = INTAKE_STAR_BOX_ROTATOR_MAX_POS;
        }
        else if (position < INTAKE_STAR_BOX_ROTATOR_MIN_POS)
        {
            intakeBoxRotatorPosition = INTAKE_STAR_BOX_ROTATOR_MIN_POS;
        }
        else {
            intakeBoxRotatorPosition = position;
        }
        intakeBoxRotaterServo.setPosition(intakeBoxRotatorPosition);
    }

    public void SetIntakeBoxRotatorIncrementUp()
    {
        intakeBoxRotatorPosition += INTAKE_STAR_BOX_ROTATOR_INCREMENT;
        SetIntakeBoxRotatorPosition(intakeBoxRotatorPosition);
    }

    public void SetIntakeBoxRotatorDecrementDown()
    {
        intakeBoxRotatorPosition -= INTAKE_STAR_BOX_ROTATOR_INCREMENT;
        SetIntakeBoxRotatorPosition(intakeBoxRotatorPosition);
    }
// *********************************************************
    // ****       Urchin Controls                     ****
    // *********************************************************

    public void SetUrchinServoPosition(double position)
    {
        if (position > URCHIN_SERVO_MAX_POS)
        {
            urchinServo_position = URCHIN_SERVO_MAX_POS;
        }
        else if (position < URCHIN_SERVO_MIN_POS)
        {
            urchinServo_position = URCHIN_SERVO_MIN_POS;
        }
        else {
            urchinServo_position = position;
        }
        urchinServo.setPosition(urchinServo_position);
    }


    // *********************************************************
    // ****       Intake Stars Controls                     ****
    // *********************************************************

    public void SetIntakeStarPower(double power)
    {/*
        intakeStarPower = power;
        intakeStarServo.setPower(intakeStarPower);
        if (power > 0.0)
        {
            intakeStarLastForward = true;
        }
        else if (power < 0.0)
        {
            intakeStarLastForward = false;
        }
        */
    }

    public void CycleIntakeStarMode() {
        /*
        if (intakeStarPower != 0.0) {
            SetIntakeStarPower(0);
        } else if(intakeStarLastForward) {
            SetIntakeStarPower(-1.0);
        }
        else {
            SetIntakeStarPower(1.0);

        }

         */
    }

    // *********************************************************
    // ****       CLAW Controls                             ****
    // *********************************************************

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

    public void SetSwiperPosition(double position)
    {
        if (position > SWIPER_MAX_POS)
        {
            swiper_position = SWIPER_MAX_POS;
        }
        else if (position < SWIPER_MIN_POS)
        {
           swiper_position = SWIPER_MIN_POS;
        }
        else
        {
            swiper_position = position;
        }
        swiperServo.setPosition(swiper_position);
    }

    // *********************************************************
    // ****       Small Arm (Hangers) Controls              ****
    // *********************************************************

    public void SetSmallArmHangerIncrementUp()
    {
        smallArmHangerRightPosition += SMALL_ARM_HANGER_INCREMENT;
        SetSmallArmHangerPosition(smallArmHangerRightPosition);
    }

    public void SetSmallArmHangerDecrementDown()
    {
        smallArmHangerRightPosition -= SMALL_ARM_HANGER_INCREMENT;
        SetSmallArmHangerPosition(smallArmHangerRightPosition);
    }

    public void SetSmallArmHangerPosition(double position)
    {
        if (position > SMALL_ARM_HANGER_MAX_POS)
        {
            smallArmHangerRightPosition = SMALL_ARM_HANGER_MAX_POS;
            smallArmHangerLeftPosition = SMALL_ARM_HANGER_MIN_POS;
        }
        else if (position < SMALL_ARM_HANGER_MIN_POS) {
            smallArmHangerRightPosition = SMALL_ARM_HANGER_MIN_POS;
            smallArmHangerLeftPosition = SMALL_ARM_HANGER_MAX_POS;
        }
        else {
            smallArmHangerRightPosition = position;
            smallArmHangerLeftPosition = SMALL_ARM_HANGER_MAX_POS - position;
        }

        smallArmHangerLeftServo.setPosition(smallArmHangerLeftPosition);
        smallArmHangerRightServo.setPosition(smallArmHangerRightPosition);
    }

    // *********************************************************
    // ****       Slide Arm Controls                ****
    // *********************************************************

    public boolean GetSlidesLimitSwitchPressed(){
        boolean returnValue = false;

        if ((!touchSensorSlideLeft.isPressed()) || (!touchSensorSlideRight.isPressed())){
            returnValue = true;
        }

        return returnValue;
    }

    // TODO -- need to determine other rotator arm positions to limit than just zero
    public void Slides_Horizontal_MAX_Limit(){
        // if the rotator arms are in a horizontal orientation
        // AND the slides are starting to go past the max horizontal position
        if ((GetRotatorRightArmCurrentPosition() <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES || GetRotatorLeftArmCurrentPosition() <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES) &&
                (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS || slideRight.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS)){
            // if slides are in power mode
            if (!slidesRunningToPosition) {
                // if user is not commanding slides to go in
                if (slidePower >= 0) {
                    // override the command to limit the slides to a max horizontal position
                    SetSlideToPosition(SLIDE_ARM_MAX_HORIZONTAL_POS);
                }
            }
            // else slides are being commanded to a position, but if slides have been commanded past the limit, bring the slides back in
            else if (slidesTargetPosition > SLIDE_ARM_MAX_HORIZONTAL_POS)
            {
                SetSlideToPosition(SLIDE_ARM_MAX_HORIZONTAL_POS);
            }
        }
    }

    public void SetSlidesToPowerMode(double power)
    {
        // if rotator arms are in horizontal position, AND slides are at the limit already, AND commanding to get longer
        if ((GetRotatorRightArmCurrentPosition() == 0 || GetRotatorLeftArmCurrentPosition() == 0) &&
                (power > 0) &&
                (slideLeft.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS || slideRight.getCurrentPosition() >= SLIDE_ARM_MAX_HORIZONTAL_POS)){
            // override and ignore the bad command by killing power to the slides
            slidePower = 0;
        }else {

            slidePower = power;
        }
        slidesRunningToPosition = false;
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        slideLeft.setPower(slidePower);
        slideRight.setPower(slidePower);
    }

    public void ResetSlidesToZero (){
        slidesRunningToPosition = false;
        slidePower = 0;
        slideLeft.setPower(slidePower);
        slideRight.setPower(slidePower);
        this.SpecialSleep(50);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void SetSlideToPosition (int position)
    {
        // verify not needing to limit because of horizontal limits
        if (((slideArmRotatorTargetPosition <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES) ||
                (!slideArmRotatorRunningToPosition && leftSlideArmRotatorMotor.getCurrentPosition() <= SLIDE_ARM_ROTATOR_POS_TO_LIMIT_SLIDES) ) &&
                (position >= SLIDE_ARM_MAX_HORIZONTAL_POS)) {
            slidesTargetPosition = SLIDE_ARM_MAX_HORIZONTAL_POS;
        }
        else if (position > SLIDE_ARM_MAX_VERTICAL_POS)
        {
            slidesTargetPosition = SLIDE_ARM_MAX_VERTICAL_POS;
        }
        else if (position < SLIDE_ARM_MIN_POS)
        {
            slidesTargetPosition = SLIDE_ARM_MIN_POS;
        }
        else {
            slidesTargetPosition = position;
        }
        slidesRunningToPosition = true;
        slideLeft.setTargetPosition(slidesTargetPosition);
        slideRight.setTargetPosition(slidesTargetPosition);
        slideLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        slidePower = SLIDES_POS_POWER;
        slideLeft.setPower(slidePower);
        slideRight.setPower(slidePower);
    }

    public void SetSlidesToHoldCurrentPosition()
    {
        if (slidesTargetPosition == 0)
        {
            ResetSlidesToZero();
        }
        else {
            slidePower = 0;
            slideLeft.setPower(slidePower);
            slideRight.setPower(slidePower);
            slidesTargetPosition = slideLeft.getCurrentPosition();
            slideLeft.setTargetPosition(slidesTargetPosition);
            slideRight.setTargetPosition(slideRight.getCurrentPosition());
            slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slideLeft.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            slideRight.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        }
        slidesRunningToPosition = false;
    }

    public boolean GetSlidesRunningToPosition() { return slidesRunningToPosition; }
    public int GetSlidesTargetPosition() { return slidesTargetPosition; }
    public int GetSlideLeftCurrentPosition() { return slideLeft.getCurrentPosition(); }
    public int GetSlideRightCurrentPosition() { return slideRight.getCurrentPosition(); }

    // *********************************************************
    // ****       Slide Rotator Arm Controls                ****
    // *********************************************************
    public void SetSlideRotatorToPowerMode(double power)
    {
        slideArmRotatorPower = power;
        //820 to 920 for limit -- SLIDE_ARM_ROTATOR_MAX_POS
        if (slideArmRotatorPower > 0 && leftSlideArmRotatorMotor.getCurrentPosition() >= SLIDE_ARM_ROTATOR_MAX_POS) {
            SetSlideRotatorArmToPosition(SLIDE_ARM_ROTATOR_MAX_POS);
        }
        else {
            slideArmRotatorRunningToPosition = false;
            leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
            rightSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);

            rightSlideArmRotatorMotor.setPower(slideArmRotatorPower);
            leftSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        }

    }

    public void SetSlideRotatorArmToPosition(int position)
    {
        // if slide arm rotators are going down then reduce the max power
        if (leftSlideArmRotatorMotor.getCurrentPosition() > position) {
            slideArmRotatorPower = SLIDE_ARM_ROTATOR_POWER / 3.0;
        }
        else{
            slideArmRotatorPower = SLIDE_ARM_ROTATOR_POWER;
        }

        if (position > SLIDE_ARM_ROTATOR_MAX_POS)
        {
            slideArmRotatorTargetPosition = SLIDE_ARM_ROTATOR_MAX_POS;
        }
        else if (position < SLIDE_ARM_ROTATOR_MIN_POS)
        {
            slideArmRotatorTargetPosition = SLIDE_ARM_ROTATOR_MIN_POS;
        }
        else {
            slideArmRotatorTargetPosition = position;
        }
        slideArmRotatorRunningToPosition = true;
        rightSlideArmRotatorMotor.setTargetPosition(slideArmRotatorTargetPosition);
        leftSlideArmRotatorMotor.setTargetPosition(slideArmRotatorTargetPosition);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        rightSlideArmRotatorMotor.setPower(slideArmRotatorPower);
    }
    public void ResetSlideRotatorArmToZero(){
        slideArmRotatorPower = 0.0;
        leftSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        rightSlideArmRotatorMotor.setPower(slideArmRotatorPower);
        this.SpecialSleep(50);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlideArmRotatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideArmRotatorMotor.setZeroPowerBehavior (DcMotor.ZeroPowerBehavior.BRAKE);

    }
    public boolean GetSlideRotatorArmLimitSwitchPressed(){
        boolean returnValue = false;

        if ((!touchSensorSlideArmRotatorRight.isPressed()) || (!touchSensorSlideArmRotatorLeft.isPressed())){
            returnValue = true;
        }

        return returnValue;
    }
    public void SetSlideRotatorArmToHoldCurrentPosition()
    {
        if (slideArmRotatorTargetPosition == 0) {
            ResetSlideRotatorArmToZero();
        }
        else {
            slideArmRotatorPower = SLIDE_ARM_ROTATOR_POWER;
            leftSlideArmRotatorMotor.setPower(slideArmRotatorPower);
            rightSlideArmRotatorMotor.setPower(slideArmRotatorPower);
            leftSlideArmRotatorMotor.setTargetPosition(leftSlideArmRotatorMotor.getCurrentPosition());
            rightSlideArmRotatorMotor.setTargetPosition(leftSlideArmRotatorMotor.getCurrentPosition());
        }
        slideArmRotatorRunningToPosition = false;
    }
    public boolean GetRotatorArmRunningToPosition()
    {
        return slideArmRotatorRunningToPosition;
    }
    public int GetRotatorArmTargetPosition()
    {
        return slideArmRotatorTargetPosition;
    }
    public int GetRotatorLeftArmCurrentPosition()
    {
        return (leftSlideArmRotatorMotor.getCurrentPosition());
    }
    public int GetRotatorRightArmCurrentPosition()
    {
        return (rightSlideArmRotatorMotor.getCurrentPosition());
    }

    public void ShowTelemetry(){
        opMode.telemetry.addData("Auto Last Time Left: ", autoTimeLeft);
        opMode.telemetry.addData("imu Heading: ", GetIMU_HeadingInDegrees());

        opMode.telemetry.addData("Claw Position: ", claw_position);
        //opMode.telemetry.addData(">", "Claw - use bumpers for control" );

        opMode.telemetry.addData("Arm Hanger Positions ", "R: %.2f, L: %.2f", smallArmHangerRightPosition, smallArmHangerLeftPosition);
        //opMode.telemetry.addData(">", "Arm Hangers - rightStick X-Axis for control" );

       // opMode.telemetry.addData("Intake Star Power: ", "%.2f, %.2f", intakeStarPower, intakeStarServo.getPower());
        opMode.telemetry.addData("Urchin Position: ", urchinServo_position);
        //opMode.telemetry.addData(">", "Intake Star - use dpad down for control" );

        opMode.telemetry.addData("slides Position ", "L: %d, R: %d", slideLeft.getCurrentPosition(), slideRight.getCurrentPosition());
        opMode.telemetry.addData("slides ", "Target: %d, Power: %.2f", slidesTargetPosition, slidePower);
        opMode.telemetry.addData("Slides Left Touched: ", !touchSensorSlideLeft.isPressed());
        opMode.telemetry.addData("Slides Rght Touched: ", !touchSensorSlideRight.isPressed());
        opMode.telemetry.addData("Slides in RUN_TO_POSITION? ", slidesRunningToPosition);
        //opMode.telemetry.addData(">", "slides - use leftStick Y for control" );

        opMode.telemetry.addData("Slide Arm Rotator Positions: ", "L: %d, R: %d", leftSlideArmRotatorMotor.getCurrentPosition(), rightSlideArmRotatorMotor.getCurrentPosition());
        opMode.telemetry.addData("Slide Arm Rotator ", "Target: %d, Power: %.2f", slideArmRotatorTargetPosition, slideArmRotatorPower);
        opMode.telemetry.addData("Slide Arm Rotator Left Touched: ", !touchSensorSlideArmRotatorLeft.isPressed());
        opMode.telemetry.addData("Slide Arm Rotator Rght Touched: ", !touchSensorSlideArmRotatorRight.isPressed());
        opMode.telemetry.addData("Slide Arm Rotator in RUN_TO_POSITION? ", slideArmRotatorRunningToPosition);
        //opMode.telemetry.addData(">", "rotateArms - use triggers for control" );

        opMode.telemetry.addData("Green Intake BOX ROTATOR Pos: ", intakeBoxRotatorPosition);
        //opMode.telemetry.addData(">", "green intake box rotator - use dpad L/R for control" );
        // color sensor data PLEASE do not delete!
        opMode.telemetry.addData("colorDetected: " , ColorRevV3Sensor().toString());
        opMode.telemetry.addData("Blinkin Left: ", Blinken_pattern.toString());
        opMode.telemetry.addData("Hue: " , hsvValues[0]);
        opMode.telemetry.addData("Sat: " , hsvValues[1]);
        opMode.telemetry.addData("Val: " , hsvValues[2]);
        opMode.telemetry.addData("Swiper Position:", swiper_position);
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

    public void EndgameBuzzer(){
        if(opMode.getRuntime() < 109.5 && opMode.getRuntime() > 109.0){
            opMode.gamepad1.rumble(1000);
            opMode.gamepad2.rumble(1000);
        }
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