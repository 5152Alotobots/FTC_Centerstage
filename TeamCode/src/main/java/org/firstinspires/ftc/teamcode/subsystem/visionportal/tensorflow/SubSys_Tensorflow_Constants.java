package org.firstinspires.ftc.teamcode.subsystem.visionportal.tensorflow;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class SubSys_Tensorflow_Constants
{
    public class ProcessorOptions
    {
        public static final int MAX_RECOGNITIONS = 16; // Max number of recognised objects on screen
        public static final boolean USE_OBJECT_TRACKER = true; // Use TFlite object tracker
        public static final float MAX_BOX_OVERLAP = 0.2f; // How much the boxes can overlap
        public static final float MIN_BOX_SIZE = 16; // Minimum size of detection recognized by the system

        public class Advanced
        {
            public static final boolean TENSORFLOW_2 = false; // Is the detector tf2
            public static final boolean IS_QUANTIZED = false; // Is the model quantized
        }
    }
    // Put these in main class BECAUSE JAVA 8 IS STUPID WTF DYM I CAN'T PUT A STATIC STRING LIST IN A SUBCLASS
    public static final String ASSET_NAME = "Test"; // Name of the asset bundled into the build
    public static final String[] MODEL_LABELS = new String[]{"A", "B", "C"}; // Labels in a String list

    public static final Transform2d ROBOT_TO_CAM = new Transform2d(
            new Translation2d(0,0), // 0M FORWARD of center, 0M RIGHT of center
            new Rotation2d(Math.toRadians(0)) // DEGREES from 0
    );
    public static final double ROBOT_TO_CAM_Z = 0.0; // 0M UP of center

    public static final double CAMERA_FOV = 60.0;
    public static final double CAMERA_PITCH = 10.0; // Positive is pitched DOWN

    public class OrderMode {
        public static final int LARGEST = 0;
        public static final int SMALLEST = 1;
        public static final int HIGHEST = 2;
        public static final int LOWEST = 3;
        public static final int RIGHTMOST = 4;
        public static final int LEFTMOST = 5;
        public static final int CENTERMOST =6;

    }

    public class ObjectSpecifications
    {
        public class Pixel
        {
            public static final double WIDTH = 0.2; // Meters
            public static final double HEIGHT = 0.2; //Meters off the ground
        }
    }
}

