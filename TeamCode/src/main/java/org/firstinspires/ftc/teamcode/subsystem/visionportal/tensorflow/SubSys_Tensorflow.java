package org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow;

import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.CAMERA_FOV;
import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.CAMERA_PITCH;
import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.OrderMode.CENTERMOST;
import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.OrderMode.HIGHEST;
import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.OrderMode.LARGEST;
import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.OrderMode.LEFTMOST;
import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.OrderMode.LOWEST;
import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.OrderMode.RIGHTMOST;
import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.OrderMode.SMALLEST;
import static org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow.SubSys_Tensorflow_Constants.ROBOT_TO_CAM_Z;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

public class SubSys_Tensorflow extends SubsystemBase
{
    private TfodProcessor processor;

    public SubSys_Tensorflow() {

        /*
        // Create new Tfod processor (ADVANCED)
        processor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(SubSys_Tensorflow_Constants.ProcessorOptions.MAX_RECOGNITIONS)
                .setUseObjectTracker(SubSys_Tensorflow_Constants.ProcessorOptions.USE_OBJECT_TRACKER)
                .setTrackerMaxOverlap(SubSys_Tensorflow_Constants.ProcessorOptions.MAX_BOX_OVERLAP)
                .setTrackerMinSize(SubSys_Tensorflow_Constants.ProcessorOptions.MIN_BOX_SIZE)
                .setIsModelTensorFlow2(SubSys_Tensorflow_Constants.ProcessorOptions.Advanced.TENSORFLOW_2)
                .setIsModelQuantized(SubSys_Tensorflow_Constants.ProcessorOptions.Advanced.IS_QUANTIZED)
                .setModelAssetName(SubSys_Tensorflow_Constants.ASSET_NAME)
                .setModelLabels(SubSys_Tensorflow_Constants.MODEL_LABELS)
                .build();


         */

        // Create new Tfod processor (TESTING ONLY, TRAIN CUSTOM ASAP)

        processor = new TfodProcessor.Builder()
                .setTrackerMinCorrelation(0.8f)
                .setIsModelTensorFlow2(false)
                .build();

    }

    @Override
    public void periodic() {
        // Runs once every scheduler run
    }

    public TfodProcessor getProcessor() {
        return processor;
    }

    /** Get a recognition by the number in the list*/
    public Recognition getRecognition(int index) {
        return processor.getRecognitions().get(index);
    }

    /** A neat wrapper for all get recognition functions*/
    public Recognition getBestRecognition(int orderMode) {
        Recognition bestRecognition = null;
        if (processor.getRecognitions() != null) {
            switch (orderMode) {
                case LARGEST:
                    bestRecognition = getLargestRecognition();
                    break;
                case SMALLEST:
                    bestRecognition = getSmallestRecognition();
                    break;
                case HIGHEST:
                    bestRecognition = getHighestRecognition();
                    break;
                case LOWEST:
                    bestRecognition = getLowestRecognition();
                    break;
                case RIGHTMOST:
                    bestRecognition = getRightmostRecognition();
                    break;
                case LEFTMOST:
                    bestRecognition = getLeftmostRecognition();
                    break;
                case CENTERMOST:
                    System.out.println("WARNING: Centermost mode. NOT TESTED");
                    bestRecognition = getCentermostRecognition();
                    break;

            }
        }
        return bestRecognition;
    }

    /** Gets the LARGEST recognition from the list*/
    public Recognition getLargestRecognition() {
        double largestSize = 0;
        Recognition bestRecognition = null;
       for (Recognition recognition : processor.getRecognitions()) {
           double curSize = (recognition.getWidth() * recognition.getHeight()); // Get the area of current object
           // if the area is bigger than the var, replace it, replace the best recognition with that
           if (largestSize < curSize) {
               largestSize = curSize;
               bestRecognition = recognition;
           }
       }
       return bestRecognition;
    }

    /** Gets the SMALLEST recognition from the list*/
    public Recognition getSmallestRecognition() {
        double smallestSize = Double.MAX_VALUE;
        Recognition bestRecognition = null;
        for (Recognition recognition : processor.getRecognitions()) {
            double curSize = (recognition.getWidth() * recognition.getHeight()); // Get the area of current object
            // if the area is smaller than the var, replace it, replace the best recognition with that
            if (curSize < smallestSize) {
                smallestSize = curSize;
                bestRecognition = recognition;
            }
        }
        return bestRecognition;
    }

    /** Gets the HIGHEST recognition from the list*/
    public Recognition getHighestRecognition() {
        double highestHeight = Double.MAX_VALUE;
        Recognition bestRecognition = null;
        for (Recognition recognition : processor.getRecognitions()) {
            double curHeight = recognition.getTop(); // Get the dist from top of current object
            // if the height is larger than the var, replace it, replace the best recognition with that
            if (curHeight < highestHeight) {
                highestHeight = curHeight;
                bestRecognition = recognition;
            }
        }
        return bestRecognition;
    }

    /** Gets the LOWEST recognition from the list*/
    public Recognition getLowestRecognition() {
        double lowestHeight = Double.MAX_VALUE; // Initialize to a high value
        Recognition bestRecognition = null;
        for (Recognition recognition : processor.getRecognitions()) {
            double curHeight = recognition.getBottom(); // Get the dist from bottom of current object
            // if the height is smaller than the var, replace it, replace the best recognition with that
            if (curHeight < lowestHeight) {
                lowestHeight = curHeight;
                bestRecognition = recognition;
            }
        }
        return bestRecognition;
    }

    /** Gets the RIGHTMOST recognition from the list*/
    public Recognition getRightmostRecognition() {
        double mostRight = Double.MAX_VALUE;
        Recognition bestRecognition = null;
        for (Recognition recognition : processor.getRecognitions()) {
            double curRight = (recognition.getRight()); // Get the dist from right of current object
            // if the dist is less than the var, replace it, replace the best recognition with that
            if (curRight < mostRight) {
                mostRight = curRight;
                bestRecognition = recognition;
            }
        }
        return bestRecognition;
    }

    /** Gets the LEFTMOST recognition from the list*/
    public Recognition getLeftmostRecognition() {
        double mostLeft = Double.MAX_VALUE;
        Recognition bestRecognition = null;
        for (Recognition recognition : processor.getRecognitions()) {
            double curLeft = (recognition.getLeft()); // Get the dist from left of current object
            // if the dist is less than the var, replace it, replace the best recognition with that
            if (curLeft < mostLeft) {
                mostLeft = curLeft;
                bestRecognition = recognition;
            }
        }
        return bestRecognition;
    }

    /** Gets the CENTERMOST recognition from the list ---
     * I probably would not use this as it was written by ChatGPT. (Yet to be tested)
     **/
    public Recognition getCentermostRecognition() {
        double closestDistance = Double.MAX_VALUE; // Initialize to a high value
        Recognition centermostRecognition = null;
        for (Recognition recognition : processor.getRecognitions()) {
            double imageCenterX = recognition.getImageWidth() * 0.5; // Calculate the horizontal center of the image
            double imageCenterY = recognition.getImageHeight() * 0.5; // Calculate the vertical center of the image

            double curCenterX = (recognition.getLeft() + recognition.getRight()) * 0.5; // Calculate the horizontal center of the current object
            double curCenterY = (recognition.getTop() + recognition.getBottom()) * 0.5; // Calculate the vertical center of the current object
            double dx = curCenterX - imageCenterX;
            double dy = curCenterY - imageCenterY;
            double distanceToCenter = Math.sqrt(dx * dx + dy * dy); // Calculate the Euclidean distance to the image center

            // if the distance to center is smaller than the closest distance, replace it, replace the centermost recognition with that
            if (distanceToCenter < closestDistance) {
                closestDistance = distanceToCenter;
                centermostRecognition = recognition;
            }
        }

        return centermostRecognition;
    }

    /** Gets the coordinates of the center of the detection
     * @param recognition The recognition to get the center point
     * @return distanceToCenterX: The distance to the center from the LEFT side of the screen<br>
     * distanceToCenterZ: The distance to the center from the BOTTOM of the screen
     * */
    public double[] getCenter(Recognition recognition) {
        double distanceToCenterX = 0;
        double distanceToCenterZ = 0;
        if (recognition != null ) {
            double imageWidth = recognition.getImageWidth();
            double imageHeight = recognition.getImageHeight();
            double detectionCenterX = recognition.getLeft() + (recognition.getWidth() * 0.5); // Calculate the horizontal center of the detection
            double detectionCenterZ = recognition.getBottom() + (recognition.getHeight() * 0.5); // Calculate the vertical center of the direction

            distanceToCenterX = imageWidth - detectionCenterX; // Calculate the distance from the LEFT side of the screen to the center of the detection
            distanceToCenterZ = imageHeight - detectionCenterZ;
            distanceToCenterZ = imageHeight - detectionCenterZ;
        }

        return new double[]{distanceToCenterX, distanceToCenterZ};
    }

    /** Gets the distance to the object
     * @param recognition The recognition given by TensorFlow
     * @param objectHeight The height of the object off the ground
     * @return The distance to the object
     * */
    public double getDistance(Recognition recognition, double objectHeight, double objectWidth) {
        double yaw = recognition.estimateAngleToObject(AngleUnit.DEGREES);

        // Calculate the angular size in radians
        double angularSizeRadians = Math.toRadians((recognition.getWidth() / recognition.getImageWidth()) * CAMERA_FOV);

        // Calculate the horizontal distance to the object in meters
        double horizontalDistanceToObject = (objectWidth / 2) / Math.tan(angularSizeRadians / 2);

        // Calculate the vertical distance to the object based on the pitch angle
        double verticalDistanceToObject = Math.tan(Math.toRadians(CAMERA_PITCH)) * horizontalDistanceToObject;

        // Calculate the total distance to the object, accounting for height difference
        double distanceToObject = Math.sqrt(Math.pow(horizontalDistanceToObject, 2) + Math.pow(objectHeight - ROBOT_TO_CAM_Z + verticalDistanceToObject, 2));

        return distanceToObject;
    }

    /** Gets the yaw to the object
     * @param recognition The recognition given by TensorFlow
     * @return The yaw to the object (DEGREES)
     * */
    public double getYaw(Recognition recognition) {
        if (recognition == null) return 0;
        return recognition.estimateAngleToObject(AngleUnit.DEGREES);
    }

    
}


