#ifndef _TEST_APP
#define _TEST_APP

#include "ofMain.h"

#include "ofxOpenCv.h"
#include "ofxOpenNI.h"

#include "ofxControlPanel.h"
using std::swap;

using namespace cv;

#include "ofxKinectSegmentation.h"




class testApp : public ofBaseApp
{
	
    
public:
    
    void setup();
    void update();
    void draw();
    void exit();
    
    void keyPressed  (int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    
    ofxKinectSegmentation oks;
	ofxControlPanel			gui;

	int						KW;
	int						KH;
    
    ofxCvColorImage         rgbOfxCv;              // Original RGB image
    ofxCvGrayscaleImage     irOfxCv;               // Original IR (infrared) depth image
    ofImage finalRGBAOf;
    
	int					gW;
	int					gH;
	int					gM;
	void				computeFrameRate();
	float				kinectPrevFrameMillis;
	float				kinectFrameRate;
    
    //------------------OpenNI--------------------
	ofxOpenNIContext	recordContext;
	ofxDepthGenerator	recordDepth;
    
#ifdef USE_IR
	ofxIRGenerator		recordImage;
#else
	ofxImageGenerator	recordImage;
#endif
	ofxOpenNIRecorder	oniRecorder;
    
#if defined (TARGET_OSX) //|| defined(TARGET_LINUX) // only working on Mac/Linux at the moment (but on Linux you need to run as sudo...)
	ofxHardwareDriver	hardware;
#endif
    
};

#endif
