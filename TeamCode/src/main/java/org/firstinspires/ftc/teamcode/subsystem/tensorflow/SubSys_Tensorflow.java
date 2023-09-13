package org.firstinspires.ftc.teamcode.subsystem.tensorflow;

import static org.firstinspires.ftc.teamcode.subsystem.tensorflow.SubSys_Tensorflow_Constants.ASSET_NAME;
import static org.firstinspires.ftc.teamcode.subsystem.tensorflow.SubSys_Tensorflow_Constants.MODEL_LABELS;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.subsystem.tensorflow.SubSys_Tensorflow_Constants.ProcessorOptions;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class SubSys_Tensorflow extends SubsystemBase
{
    TfodProcessor processor;

    public SubSys_Tensorflow() {
        // Create new Tfod processor
        processor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(ProcessorOptions.MAX_RECOGNITIONS)
                .setUseObjectTracker(ProcessorOptions.USE_OBJECT_TRACKER)
                .setTrackerMaxOverlap(ProcessorOptions.MAX_BOX_OVERLAP)
                .setTrackerMinSize(ProcessorOptions.MIN_BOX_SIZE)
                .setIsModelTensorFlow2(ProcessorOptions.Advanced.TENSORFLOW_2)
                .setIsModelQuantized(ProcessorOptions.Advanced.IS_QUANTIZED)
                .setModelAssetName(ASSET_NAME)
                .setModelLabels(MODEL_LABELS)
                .build();
    }

    @Override
    public void periodic() {
        // Runs once every scheduler run
    }

    public TfodProcessor getProcessor() {
        return processor;
    }

    public List<Recognition> getRecognitions() {
        return processor.getRecognitions();
    }


}
