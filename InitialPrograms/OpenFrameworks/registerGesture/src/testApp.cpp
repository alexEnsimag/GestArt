#define PRECISION_RECO 0.2
/*
 GRT MIT License
 Copyright (c) <2012> <Nicholas Gillian, Media Lab, MIT>

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all copies or substantial
 portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
 LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/*
 DTW Example

 This example shows you how to:
 - setup a gesture recognition pipeline
 - record your own dataset and save it to a file
 - load the dataset back from a file
 - train a DTW classification algorithm using the training dataset
 - use the trained DTW algorithm to predict the class of real-time data

 This example uses the 2-dimensional [x y] coordinates from your mouse as input, but you can easily change this to whatever sensor input
 you have access to.

 To compile this example:
 - use the Openframeworks project builder to create a new project
 - when you have created the new project, override the default testApp.h, testApp.cpp, and main.cpp files with the files from this example
 - open the project in your favorite IDE (XCode, Visual Studio, Code Blocks, etc.) and add the main GRT source folder to the project. You
   can find the main GRT source folder by looking for the folder called GRT in the directory you downloaded from google code. Most IDE's let
   you just drag and drop the entire GRT code folder into your project.
 - note that some IDE's make you specify the location of the GRT source code folder (for example Visual Studio). To do this, open the project's
   properties or setting pane and add the path to the GRT folder to your project's cpp Include section. In XCode you can just drag and drop the
   GRT folder directly from finder into your project.
 - compile openframeworks
 - compile this project

 When you have compiled this project, this is how you use it:
 - run the project
 - when you start the project, you will have no training data and the classifier will not be trained so you need to do three things:
   (1) record some training data
   (2) train your pipeline
   (3) use the pipeline to predict the class of real-time data
 - Step 1:
   - to record some training data, first make sure the value beside the TrainingClassLabel is set to the class you want to record the data for
   - to change the training class label you can use the '[' and ']' keys, [ to decrease the label and ] to increase the label
   - press the 'r' key to start recording the training data
   - move your mouse to make a gesture (for example, you could draw the letter G)
   - press the 'r' key to stop recording the training data
   - repeat this a few times (e.g. 5-10 times)
   - change the training class label to a new label
   - press the 'r' key to start the recording, perform the next gesture, stop the recording
   - keep repeating these steps until you have recorded all the training data you want
   - when you have finished, press the 's' key to save the training data to a file
   - if you need to load the training data at a later stage, for instance when you next restart the program, press the 'l' key
 - Step 2:
   - after you have recorded your training data, you can now train your pipeline
   - to train your pipeline, press the 't' key
   - if the pipeline trained a classification model successfully then you will see the info message: Pipeline Trained, otherwise you will see the
     warning message WARNING: Failed to train pipeline. If the training failed, then make sure you have successfully recorded the training data
 - Step 3:
   - after you have trained the pipeline, you can now use the pipeline to predict the class of real-time data
   - if the pipeline was trained, it will automatically start to predict the class of real-time data
   - move your mouse around the screen and you should see the predicted class label change through the various classes you trained the model to predict
   - note that you might also see the predicted class label of 0. This is the special NULL GESTURE LABEL, which is output by the classifier when the
     likelihood of a gesture is too low. See this tutorial for more info: http://www.nickgillian.com/wiki/pmwiki.php?n=GRT.AutomaticGestureSpotting
 */


#include "testApp.h"

#include <iostream>
#include <fstream>
#include <algorithm>

//--------------------------------------------------------------
void testApp::setup()
{

    ofSetFrameRate(30);

    //Initialize the training and info variables
    infoText = "";
    trainingClassLabel = 1;
    record = false;

    //Open the connection with Synapse
    synapseStreamer.openSynapseConnection();

    //Set which joints we want to track
    synapseStreamer.trackAllJoints(false);
    synapseStreamer.trackLeftHand(true);
    synapseStreamer.trackRightHand(true);
    synapseStreamer.computeHandDistFeature(true);

    //The input to the training data will be the [x y] from the mouse, so we set the number of dimensions to 2
    trainingData.setNumDimensions( 6 );
    performTrainingLabel();
    //trainingClassLabel = 1;

    //Initialize the DTW classifier
    DTW dtw;

    //Turn on null rejection, this lets the classifier output the predicted class label of 0 when the likelihood of a gesture is low
    dtw.enableNullRejection( true );

    //Set the null rejection coefficient to 3, this controls the thresholds for the automatic null rejection
    //You can increase this value if you find that your real-time gestures are not being recognized
    //If you are getting too many false positives then you should decrease this value
    dtw.setNullRejectionCoeff( PRECISION_RECO );

    //Turn on the automatic data triming, this will remove any sections of none movement from the start and end of the training samples
    dtw.enableTrimTrainingData(true, 0.1, 90);

    //Offset the timeseries data by the first sample, this makes your gestures (more) invariant to the location the gesture is performed
    dtw.setOffsetTimeseriesUsingFirstSample(true);

    //Add the classifier to the pipeline (after we do this, we don't need the DTW classifier anymore)
    pipeline.setClassifier( dtw );

    //Chargement des données déjà sauvegardées
    if( trainingData.loadDatasetFromFile("TrainingData.txt") )
    {
        infoText = "Training data saved to file";
    }
    else infoText = "WARNING: Failed to load training data from file";
}

//--------------------------------------------------------------
void testApp::update()
{
    //Parse any new messages from the synapse streamer
    synapseStreamer.parseIncomingMessages();

    if( synapseStreamer.getNewMessage() )
    {

        //Get the left hand and right hand joints
        leftHand = synapseStreamer.getLeftHandJointBody();
        rightHand = synapseStreamer.getRightHandJointBody();

        vector< double > inputVector(6);
        inputVector[0] = leftHand[0];
        inputVector[1] = leftHand[1];
        inputVector[2] = leftHand[2];
        inputVector[3] = rightHand[0];
        inputVector[4] = rightHand[1];
        inputVector[5] = rightHand[2];

        // If recording, save hand positions
        if(record)
        {
            timeseries.push_back( inputVector );
        }

        // If data trained, predict the movement
        if( pipeline.getTrained() ){
            if( !pipeline.predict(inputVector) ){
                infoText = "Failed to make prediction";
            }
        }
    }
}

//--------------------------------------------------------------
void testApp::draw()
{

    ofBackground(0, 0, 0);

    string text;
    int textX = 20;
    int textY = 20;

    ofSetColor(255, 255, 255);

    //Draw the training info
    textY += 30;
    text = "---- TrainingInfo ----";
    ofDrawBitmapString(text, textX,textY);

    if( record ) ofSetColor(255, 0, 0);
    else ofSetColor(255, 255, 255);
    textY += 15;
    text = record ? "RECORDING" : "Not Recording";
    ofDrawBitmapString(text, textX,textY);

    ofSetColor(255, 255, 255);
    textY += 15;
    text = "TrainingClassLabel: " + ofToString(trainingClassLabel);
    ofDrawBitmapString(text, textX,textY);

    textY += 15;
    text = "NumTrainingSamples: " + ofToString(trainingData.getNumSamples());
    ofDrawBitmapString(text, textX,textY);


    //Draw the prediction info
    textY += 30;
    text = "------------------- Prediction Info -------------------";
    ofDrawBitmapString(text, textX,textY);

    textY += 15;
    text =  pipeline.getTrained() ? "Model Trained: YES" : "Model Trained: NO";
    ofDrawBitmapString(text, textX,textY);

    textY += 15;
    text = "PredictedClassLabel: " + ofToString(pipeline.getPredictedClassLabel());
    ofDrawBitmapString(text, textX,textY);

    textY += 15;
    text = "Likelihood: " + ofToString(pipeline.getMaximumLikelihood());
    ofDrawBitmapString(text, textX,textY);

    textY += 15;
    text = "SampleRate: " + ofToString(ofGetFrameRate(),2);
    ofDrawBitmapString(text, textX,textY);


    //Draw the info text
    textY += 30;
    text = "InfoText: " + infoText;
    ofDrawBitmapString(text, textX,textY);
}

//--------------------------------------------------------------
void testApp::keyPressed(int key)
{

    infoText = "";

    switch ( key)
    {
    case 'r':
        record = !record;
        if( !record )
        {
            trainingData.addSample(trainingClassLabel, timeseries);

            //Clear the timeseries for the next recording
            timeseries.clear();
        }
        break;
    case '[':
        if( trainingClassLabel > 1 )
            trainingClassLabel--;
        break;
    case ']':
        trainingClassLabel++;
        break;
    case 't':
        if( pipeline.train( trainingData ) )
        {
            infoText = "Pipeline Trained";
        }
        else infoText = "WARNING: Failed to train pipeline";
        break;
    case 's':
        if( trainingData.saveDatasetToFile("TrainingData.txt") )
        {
            infoText = "Training data saved to file";
        }
        else infoText = "WARNING: Failed to save training data to file";
        break;
    case 'l':
        if( trainingData.loadDatasetFromFile("TrainingData.txt") )
        {
            infoText = "Training data saved to file";
        }
        else infoText = "WARNING: Failed to load training data from file";
        break;
    case 'c':
        trainingData.clear();
        infoText = "Training data cleared";
        break;
    default:
        break;
    }

}

//--------------------------------------------------------------
void testApp::keyReleased(int key)
{

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y )
{

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg)
{

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo)
{

}

// Place le trainingClassLabel sur le numéro du geste où enregistrer les échantillons
void testApp::performTrainingLabel()
{
    //string gestFileName ="../../../../../../../src/Model/Account/gestFile.txt"; // si testé à part
    string gestFileName = "gestFile.txt";
    ifstream fichier(gestFileName.c_str(), ios::in);
    if(!fichier)
    {
        cout << "Erreur à l'ouverture du fichier gestes" << endl;
        trainingClassLabel = 1;
    }
    else
    {
        int nbGestures = 0;
        char buffer[256];
        while(!fichier.eof())
        {
            fichier.getline(buffer, 256);
            cout << buffer << endl;
            nbGestures++;
        }
        if(nbGestures > 0) { // si fichier vide, on ne fait rien
            trainingClassLabel = nbGestures - 2; // on retire la dernière ligne + la ligne du geste déjà écrit avant l'ouverture de ce programme
            cout << trainingClassLabel << " gestes charges" << endl;
            trainingClassLabel++; // on incrémente pour l'enregistrement du nouveau geste
        }
    }
    fichier.close();
}
