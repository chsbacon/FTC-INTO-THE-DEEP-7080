//package org.firstinspires.ftc.teamcode.drive.modules.Tfod;
//
//
//import org.firstinspires.ftc.teamcode.ml.FTCVisionModelV1;
//import org.tensorflow.lite.support.image.TensorImage;
//import org.tensorflow.lite.support.label.Category;
//
//import java.io.IOException;
//import java.util.*;
//
//public class TfodController {
//
//    private List<Category> output;
//
//    public TfodController() {
//        try {
//            FTCVisionModelV1 model = FTCVisionModelV1.newInstance(context);
//
//            // Creates inputs for reference.
////            TensorImage image = TensorImage.fromBitmap(bitmap);
//
//            // Runs model inference and gets result.
//            FTCVisionModelV1.Outputs outputs = model.process(image);
//            output = outputs.getOutputAsCategoryList();
//
//            // Releases model resources if no longer used.
//            model.close();
//        } catch (IOException e) {
//            // TODO Handle the exception
//        }
//    }
//
//    public List<Category> getOutput() {
//        return output;
//    }
//
//    public void cycle
//
//
//}
