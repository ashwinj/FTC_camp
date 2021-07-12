//Double driver field centric for the blue alliance

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedHashMap;

@TeleOp(name = "Driving", group = "Taus")

@Config
public class Drivers extends LinearOpMode {

    public AutonomousMethods method = new AutonomousMethods();
    boolean isAPressed = false;
    boolean shooterOn = true;
    boolean isBPressed = false;
    boolean clawClosed = true;
    boolean isRunning = false;
    boolean isXPressed = false;
    boolean dpadPressed = false;
    boolean leftStick = false;
    boolean accelerating = true;
    boolean isYPressed = false;
    boolean isBlockerDown = true;
    boolean RingIn = false;
    boolean intakingRing = true;
    double rings = 0;
    boolean shooting = false;
    boolean middleGoal = false;
    boolean powerShotsIs = false;
    double powerX = 80.5;

    boolean recording = false;
    boolean isLeftStick = false;
    int numRings = 0;
    int numRingsEnd = 0;
    int wobbleGoals = 0;
    int powerShots = 0;
    int numRingsMid = 0;
    int numRingsMidEnd = 0;
    int addOns = 90;
    double timeIntaking = 0;
    double timeShooting = 0;

    double multiplier = 1;
    double speedFactor = 1;
    double previousY = 0;
    double previousX = 0;
    double prevMagnitude = 0;
    boolean firstShot = true;
    boolean intakingInitially;
    double scale = 1;
    boolean intaking = true;
    boolean back = false;
    Thread pos = new Position();
    LinkedHashMap<String, Integer> points= new LinkedHashMap<String, Integer>();
    double leftWheelInches;
    double rightWheelInches;
    double middleWheelInches;
    double anglingPow = 0;
    boolean clicked = false;
    boolean shootingTime = false;

    FtcDashboard dash = FtcDashboard.getInstance();

    TelemetryPacket packet = new TelemetryPacket();



    @Override
    //run file
    public void runOpMode() {
        method.robot.initializeHardware(hardwareMap);
        telemetry.addLine(method.magic8());
        telemetry.addData("interval", dash.getTelemetryTransmissionInterval());
        telemetry.update();
        method.currentYPosition = 72;
        method.currentXPosition = 108;

        method.robot.shooter.setVelocityPIDFCoefficients(method.p,method.i,method.d,method.f);
        method.resetAngle = method.getHeading();
        leftWheelInches = (method.robot.intake2.getCurrentPosition()/method.encoderCountsEnc)*Math.PI*method.wheelDiameterEnc;
        rightWheelInches = (method.robot.encoders.getCurrentPosition()/method.encoderCountsEnc)*Math.PI*method.wheelDiameterEnc;
        middleWheelInches = (method.robot.intake.getCurrentPosition()/method.encoderCountsEnc)*Math.PI*method.wheelDiameterEnc;
        while(!isStarted()) {
            telemetry.addData("system", method.robot.imu.getSystemStatus());
            telemetry.addData("status", method.robot.imu.getCalibrationStatus());
            telemetry.addLine(method.magic8());
            telemetry.update();
        }

        waitForStart();
        method.setShooterPower(method.shooterPower);
        method.controlIndexServo(1);
        method.controlBlocker(0);
        method.controlArmServo(0);//move arm up
        method.controlClawServo(.25);//open

        method.runtime2.reset();
        method.runtime3.reset();
        method.shooting.reset();
        method.intaking.reset();
        pos.start();

        packet.fieldOverlay().clear();
        while (opModeIsActive()) {

            packet.put("x", method.currentXPosition);
            packet.put("y", method.currentYPosition);
            packet.put("angle imu", method.getHeading());
            packet.put("angle enc", method.getHeadingComp());
            packet.put("new raw", method.getHeadingRawComp());
            packet.fieldOverlay().clear();
            packet.fieldOverlay().setFill("blue").fillRect(method.currentYPosition-9-72, -(method.currentXPosition+9-72), 18, 18);
            dash.sendTelemetryPacket(packet);
            packet.clearLines();

            drive();
            claw();
            startstop();

            shoot();
            powerShot();
            resetAngle();
            ringIn();

            telemetry.addData("angle", (int)method.getHeading());
            telemetry.addData("midgoal", middleGoal);
            telemetry.addData("target", (int)method.shooterRpm);
            telemetry.addData("rpm", (int)(method.robot.shooter.getVelocity()/28.0)*60);
            telemetry.addData("numRings", rings);
            telemetry.addData("left", leftWheelInches);
            telemetry.addData("right", rightWheelInches);
            telemetry.addData("position", "[" +method.currentXPosition + ", " + method.currentYPosition + "]");

            telemetry.update();
            telemetry.clear();




        }

        pos.interrupt();
        sleep(500);
        method.setAllMotorsTo(0);
        method.setShooterPower(0);
        method.setIntakePower(0);
    }

    //drive base movement
    public void drive(){
        method.runWithEncoders();
        if (gamepad1.right_stick_button&&!leftStick){
            if (speedFactor==1){
                speedFactor = .25;
            }
            else {
                speedFactor = 1;
            }
            leftStick = true;
        }
        if(!gamepad1.right_stick_button){
            leftStick = false;
        }
        double scaleFactor = 1;
        double rotationValue = 0;
        double stickX = 0;
        double stickY = 0;

        if(Math.abs(gamepad1.right_stick_x)>.05) {
            rotationValue = gamepad1.right_stick_x;
            anglingPow = 0;
            shooting = false;
        }
//        else if(gamepad1.right_trigger>.1){
//            rotationValue = .2;
//            anglingPow = 0;
//            angling.interrupt();
//            shooting = false;
//        }
//        else if(gamepad1.left_trigger>.1){
//            rotationValue = -.2;
//            anglingPow = 0;
//            angling.interrupt();
//            shooting = false;
//        }
        else if (shooting&& Math.abs(method.getHeading()-method.shootingAngle)>1){
            rotationValue=anglingPow;
        }
        else{
            rotationValue=0;
        }
        if(Math.abs(gamepad1.left_stick_x)>.05) {
            stickX = -gamepad1.left_stick_x;
        }
        else {
            stickX=0;
        }
        if(Math.abs(gamepad1.left_stick_y)>.05) {
            stickY = gamepad1.left_stick_y;
        }
        else {
            stickY=0;
        }
        double gyroAngle = method.getHeading() * Math.PI / 180; //Converts gyroAngle into radians

        //Robot Centric
        //gyroAngle = Math.PI / 2;

        //inverse tangent of game-pad stick y/ game-pad stick x = angle of joystick
        double joystickAngle = Math.atan2(stickY, stickX);
        double theta =  joystickAngle+gyroAngle;

        //changing from a [+] with -- being y and | being x to an [X] with \ being y and / being x (left is forward)
        double calculationAngle = theta - ((3*Math.PI) / 4);

        //magnitude of movement using pythagorean theorem
        double magnitude = Math.sqrt(Math.pow(stickX, 2) + Math.pow(stickY, 2));
        double xComponent = magnitude * (Math.cos(calculationAngle));
        double yComponent = magnitude * (Math.sin(calculationAngle));

        //creates scaleFactor to make sure movement+turning doesn't exceed power 1
        if (yComponent - rotationValue > 1) {
            scaleFactor = Math.abs(yComponent - rotationValue);
        }
        if (yComponent + rotationValue > 1 && yComponent + rotationValue > scaleFactor) {
            scaleFactor = Math.abs(yComponent + rotationValue);
        }

//        if(method.runtime4.seconds()>.25) {
//            if (Math.abs(magnitude - prevMagnitude) / method.runtime4.seconds() > 1) {
//                scale = magnitude - prevMagnitude;
//                method.runtime2.reset();
//            }
//            method.runtime4.reset();
//            prevMagnitude = magnitude*multiplier;
//        }

//        multiplier = method.errorToPower(method.runtime2.seconds(), scale, 0, 1, 0);

        method.robot.frontLeftMotor.setPower((((xComponent + rotationValue) / scaleFactor)*multiplier)*speedFactor);//x
        method.robot.backRightMotor.setPower((((xComponent - rotationValue) / scaleFactor)*multiplier)*speedFactor);//x
        method.robot.backLeftMotor.setPower((((yComponent + rotationValue) / scaleFactor)*multiplier)*speedFactor);//y
        method.robot.frontRightMotor.setPower((((yComponent - rotationValue) / scaleFactor)*multiplier)*speedFactor);//y

    }

    public void toAngle(){
        if(gamepad1.dpad_left){
            if (powerShotsIs){
                powerX = 66;
                updateShootingParameters();
                shooting = true;
            }
            else if(!middleGoal) {
                method.currentXPosition = 84;
                method.currentYPosition = 72;
                updateShootingParameters();
                shooting = true;
            }
            else{
                method.currentXPosition = 12;
                method.currentYPosition = 72;
                updateShootingParameters();
                shooting = true;
            }
        }
        if(gamepad2.dpad_up){
            if (powerShotsIs){
                powerX = 73;
                updateShootingParameters();
                shooting = true;
            }
            else if(!middleGoal) {
                method.currentXPosition = 108;
                method.currentYPosition = 72;
                updateShootingParameters();
                shooting = true;
            }
            else {
                method.currentXPosition = 36;
                method.currentYPosition = 72;
                method.updateShootingParameters4();
                shooting = true;
            }
        }
        if(gamepad1.dpad_right){
            if (powerShotsIs){
                powerX = 80.5;
                updateShootingParameters();
                shooting = true;
            }
            else if(!middleGoal) {
                method.currentXPosition = 132;
                method.currentYPosition = 72;
                updateShootingParameters();
                shooting = true;
            }
            else{
                method.currentXPosition = 60;
                method.currentYPosition = 72;
                method.updateShootingParameters4();
                shooting = true;
            }
        }
        if(gamepad1.left_bumper){
            updateShootingParameters();
            shooting=true;
        }
        if(gamepad1.a){
            middleGoal = false;
            powerShotsIs = false;
        }
        if(gamepad1.left_trigger>.1){
            powerShotsIs = !powerShotsIs;
            middleGoal = false;
        }
        if(gamepad1.right_trigger>.1){
            middleGoal = !middleGoal;
            powerShotsIs = false;
        }

        if (!shooting||(Math.abs(gamepad1.right_stick_x)>.1)){
            shooting=false;
        }

    }
    public void powerShot(){
        if(gamepad2.left_trigger>.1) {
            telemetry.addLine(method.magic8());
            telemetry.update();
            method.powerShot(-21, -17, -13, method.powerShotPower, method.shooterPower);//-19 -15 -11
            rings=0;
            powerShots=3;
        }
        if(gamepad2.dpad_left){
            method.setShooterPower(method.powerShotPower);
            method.toAngle(-21, .5);
            method.shootRings(1);
            method.setShooterPower(method.shooterPower);
            if (rings>0) {
                rings -= 1;
            }
        }
        if(gamepad2.dpad_up){
            method.setShooterPower(method.powerShotPower);
            method.toAngle(-17, .5);
            method.shootRings(1);
            method.setShooterPower(method.shooterPower);
            if (rings>0) {
                rings -= 1;
            }
        }
        if(gamepad2.dpad_right){
            method.setShooterPower(method.powerShotPower);
            method.toAngle(-13, .5);
            method.shootRings(1);
            method.setShooterPower(method.shooterPower);
            if (rings>0) {
                rings -= 1;
            }
        }
    }
    public void shoot(){
        if(gamepad1.x) {
            method.setAllMotorsTo(0);
            method.shootRings(3);
            if (recording) {
                if(method.runtime3.seconds()<90) {
                    if(method.shooterRpm==1850){
                        numRingsMid+=rings;
                    }
                    else {
                        numRings += rings;
                    }
                }
                else{
                    if(method.shooterRpm==1850){
                        numRingsMidEnd+=rings;
                    }
                    else {
                        numRingsEnd += rings;
                    }
                }
            }
            rings=0;
        }
    }
    public void updateShootingParameters(){
        if (middleGoal){
            method.updateShootingParameters4();
        }
        else {
            method.updateShootingParameters2();
        }

    }

    //subsystems and utilities
    public void claw(){
        if((gamepad1.b && !isBPressed)){//||(method.runtime3.seconds()>87&&method.runtime3.seconds()<90)
            isBPressed = true;
            if (!clawClosed) {
                method.controlClawServo(.25);//closing claw
                isRunning = true;
                method.runtime.reset();

            }
            else{
                method.controlArmServo(1);//moving arm down
                isRunning = true;
                method.runtime.reset();
                if (method.runtime3.seconds()>95&wobbleGoals<2){
                    wobbleGoals++;
                }
            }
        }
        if(!gamepad1.b){
            isBPressed = false;
        }
        if (isRunning){
            if (!clawClosed){
                if (method.runtime.seconds() > .5) {
                    method.controlArmServo(.25);//move arm up
                    clawClosed = true;
                    isRunning = false;
                }

            }
            else{
                if (method.runtime.seconds() > .5) {
                    method.controlClawServo(.7);//opening claw
                    clawClosed = false;
                    isRunning = false;
                }
            }
        }
    }
    public void ringIn(){
        if(!RingIn&&method.robot.distance.getDistance(DistanceUnit.CM)<5){
            RingIn = true;
            intakingInitially = intakingRing;
        }
        if(RingIn&&method.robot.distance.getDistance(DistanceUnit.CM)>5){
            RingIn= false;
            if(intakingRing&&intakingInitially) {
                rings++;
            }
            else if(!intakingRing&&!intakingInitially){
                rings--;
            }
        }
        if(rings<3&&method.runtime3.seconds()<90 && recording){
            if(shootingTime){
                shootingTime = false;
                method.intaking.reset();
            }
            timeIntaking+=method.intaking.seconds();
            method.intaking.reset();
            if(!intaking){
                intaking=true;
            }

        }
        if(rings>2&&method.runtime3.seconds()<90 && recording){
            if(!shootingTime){
                shootingTime = true;
                method.shooting.reset();
            }
            timeShooting+=method.shooting.seconds();
            method.shooting.reset();
            if(intaking){
                intaking=false;
            }
        }
        if(gamepad1.y){
            method.setIntakePower(-1);
        }
        if(rings<3&!shooting){
            method.setIntakePower(1);
        }
        else {
            method.setIntakePower(0);
        }
    }
    public void resetAngle() {
        if (gamepad1.right_bumper) {
            method.resetAngle = method.getHeading() + method.resetAngle;
            method.resetAngle2 = method.getHeading2() +method.resetAngle2;
            method.robot.encoders.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            method.robot.intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            method.robot.intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void startstop(){
        if (gamepad1.back && !back){
            recording = !recording;
            if(recording){
                method.runtime3.reset();
                method.intaking.reset();
                method.shooting.reset();
                numRings = 0;
                numRingsEnd = 0;
                wobbleGoals = 0;
                powerShots = 0;
                numRingsMid = 0;
                numRingsMidEnd = 0;
                addOns=90;
                timeIntaking = 0;
                timeShooting = 0;
                intaking = true;
            }
            else{
                points.put("rings high", numRings);
                points.put("rings mid", numRingsMid);
                points.put("endgame rings high", numRingsEnd);
                points.put("endgame rings mid", numRingsMidEnd);
                points.put("power shots", powerShots);
                points.put("wobble goals", wobbleGoals);
                points.put("addOns", addOns);
                int index = 0;
                String current= "";
                while(!(gamepad1.right_trigger>.1)){
                    int finder = 0;
                    for (String i: points.keySet()){
                        if (index == finder){
                            current = i;
                        }
                        finder++;
                    }
                    points=numPicker(current, points, index);
                    if (method.down) {
                        index++;
                        if(index>points.size()-1){
                            index = 0;
                        }
                    }
                    else{
                        index--;
                        if (index<0) {
                            index = points.size()-1;
                        }
                    }
                }
                String summary="Game Summary:\n";
                numRings = points.get("rings high");
                numRingsMid = points.get("rings mid");
                numRingsEnd = points.get("endgame rings high");
                numRingsMidEnd = points.get("endgame rings mid");
                powerShots = points.get("power shots");
                wobbleGoals = points.get("wobble goals");
                addOns = points.get("addOns");
                int pointsRings=numRings*6;
                int pointsRingsMid=numRingsMid*4;
                int cycles = (numRings+numRingsMid)/3;
                int pointsWobble=wobbleGoals*20;
                int pointsPower=powerShots*15;
                int pointsRingsEnd=numRingsEnd*6;
                int pointsRingsEndMid=numRingsMidEnd*4;
                int cycleTime = (90/cycles);
                if(method.runtime3.seconds()<90){
                    cycleTime = (int)(method.runtime3.seconds()/cycles);
                }
                int total = pointsRings+pointsRingsEnd+pointsRingsMid+pointsRingsEndMid+pointsWobble+pointsPower;
                double intakingTime = (int)method.intaking.seconds();
                double shootingTime = (int)method.shooting.seconds();
                double intakingTimePerCycle = (int)(intakingTime/cycles);
                double shootingTimePerCycle = (int)(shootingTime/cycles);
                summary+="Teleop:\n";
                summary+=numRings + " rings in high (" + pointsRings + " pts)\n";
                summary+=numRingsMid + " rings in mid (" + pointsRingsMid + " pts)\n";
                summary+=cycles + " cycles\n";
                summary+="cycle time of " + cycleTime + " s/cycle\n";
                summary+=shootingTime + " shooting (" + shootingTimePerCycle + "s/cycle)\n";
                summary+=intakingTime + " intaking (" + intakingTimePerCycle + "s/cycle)\n";
                summary+="Endgame:\n";
                summary+=numRingsEnd + " endgame rings in high(" + pointsRingsEnd + " pts)\n";
                summary+=numRingsMidEnd + " endgame rings in mid(" + pointsRingsEndMid + " pts)\n";
                summary+= wobbleGoals + "/2 wobble goals (" + pointsWobble + " pts)\n";
                summary+= powerShots + "/3 powershots (" + pointsPower + " pts)\n\n";
                summary+= "total: " + total + "\n";
                summary+= "total+: " + (total+addOns) + "\n";
                telemetry.addLine(summary);
                telemetry.update();
                sleep(1000);
                while (!(gamepad1.right_trigger>.1)){
                    idle();
                }

            }
            back = true;
        }
        else if (!gamepad1.back){
            back = false;
        }
    }
    public LinkedHashMap numPicker(String key, LinkedHashMap points, int index){
        LinkedHashMap<String, Integer> newPoints = points;
        while(!(gamepad1.right_trigger>.1)) {
            if (gamepad1.dpad_right&!clicked) {
                newPoints.put(key, newPoints.get(key)+1);
                clicked = true;
            }
            else if (gamepad1.dpad_left&!clicked) {
                newPoints.put(key, newPoints.get(key)-1);
                clicked = true;
            }
            else if (gamepad1.dpad_up&!clicked) {
                method.down=false;
                clicked = true;
                return newPoints;
            }
            else if (gamepad1.dpad_down&!clicked) {
                method.down=true;
                clicked=true;
                return newPoints;
            }
            else if (!gamepad1.dpad_right&&!gamepad1.dpad_left&&!gamepad1.dpad_down&&!gamepad1.dpad_up){
                clicked=false;
            }
            String line = method.hashToString(points, index);
            telemetry.addLine(line);
            telemetry.update();
        }
        return newPoints;
    }
    public void adjust (){
        if(powerShotsIs){
            method.updateShootingParameters6(powerX);
        }
        else if (middleGoal) {
            method.updateShootingParameters4();
        }
        else{
            method.updateShootingParameters2();
        }

        anglingPow = method.toAngle2(method.shootingAngle, 1);
    }


    private class Position extends Thread {
        public Position() {
            this.setName("Position");
        }

        @Override
        public void run() {

            while (!isInterrupted()&opModeIsActive()) {
                if (gamepad1.dpad_left) {
                    method.currentXPosition=108;
                    method.currentYPosition=72;
                }
                leftWheelInches = (method.robot.intake2.getCurrentPosition() / method.encoderCountsEnc) * Math.PI * method.wheelDiameterEnc * method.yMult;
                rightWheelInches = (method.robot.encoders.getCurrentPosition() / method.encoderCountsEnc) * Math.PI * method.wheelDiameterEnc * method.yMult;
                middleWheelInches = (method.robot.intake.getCurrentPosition() / method.encoderCountsEnc) * Math.PI * method.wheelDiameterEnc * method.xMult;
                double rotation = (method.robot.backLeftMotor.getCurrentPosition() - method.robot.frontRightMotor.getCurrentPosition()) / 2.0;

                double currentY = (leftWheelInches+rightWheelInches)/2;
                double deltaY1 = currentY-previousY;
                //telemetry.addData("Delta y1", deltaY1);

                double currentX = middleWheelInches-((method.getHeadingRawComp()/360)*method.encCircX);
                double deltaX1 = currentX-previousX;
                //telemetry.addData("Delta x1", deltaX1);

                double thetaX = 0;
                double thetaY = Math.PI/2;

                //changing from a [+] with | being y and -- being x to an [X] with \ being y and / being x (forward is forward)
                //double rotatedTheta = theta + (Math.PI / 4);
                double gyroAngle = method.getHeadingComp() * (Math.PI / 180); //Converts gyroAngle into radians

                double calculationAngleX =  thetaX-gyroAngle;
                double calculationAngleY =  thetaY-gyroAngle;

                double deltaY2 = Math.sin(calculationAngleY) * deltaY1;
                if(Math.abs(Math.sin(calculationAngleX) * deltaX1)>Math.abs(Math.sin(calculationAngleY) * deltaY1)){
                    deltaY2 = Math.sin(calculationAngleX) * deltaX1;
                }
                if(((Math.sin(calculationAngleY) * deltaY1)>0&&(Math.sin(calculationAngleX) * deltaX1)<0)||((Math.sin(calculationAngleY) * deltaY1)<0&&(Math.sin(calculationAngleX) * deltaX1)>0)) {
                    deltaY2 = (Math.sin(calculationAngleY) * deltaY1) + (Math.sin(calculationAngleX) * deltaX1);
                }
                double deltaX2 = Math.cos(calculationAngleX) * deltaX1;
                if(Math.abs(Math.cos(calculationAngleY) * deltaY1)>Math.abs(Math.cos(calculationAngleX) * deltaX1)){
                    deltaX2 = Math.cos(calculationAngleY) * deltaY1;
                }
                if((((Math.cos(calculationAngleY) * deltaY1)>0)&&(Math.cos(calculationAngleX) * deltaX1)<0)||(((Math.cos(calculationAngleY) * deltaY1)<0)&&((Math.cos(calculationAngleX) * deltaX1)>0))) {
                    deltaX2 = Math.cos(calculationAngleY) * deltaY1 + Math.cos(calculationAngleX) * deltaX1;
                }
                method.currentYPosition += deltaY2;
                if(method.currentYPosition<9){
                        method.currentYPosition = 9;
                    }
                else if (method.currentYPosition>132.5){
                        method.currentYPosition = 132.5;
                    }
                method.currentXPosition += deltaX2;
                if(method.currentXPosition<9){
                        method.currentXPosition = 9;
                    }
                else if (method.currentXPosition>132.5){
                        method.currentXPosition = 132.5;
                    }
                previousX = currentX;
                previousY = currentY;
                toAngle();
                adjust();


            }
        }
    }

}