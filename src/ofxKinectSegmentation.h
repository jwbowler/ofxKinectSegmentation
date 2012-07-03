#pragma once

#include "ofxOpenCv.h"

class ofxKinectSegmentation {
    
public:
    ofxKinectSegmentation();
    void init();
    
    void getBitMask(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage, ofxCvGrayscaleImage *output);
    void getGrayscaleMask(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage, ofxCvGrayscaleImage *output);
    void getRGB(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage, ofxCvColorImage *output);
    void getRGBA(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage, ofImage *output);
    
    int	KW;
	int	KH; 
    
    int minContourArea;
    int maxContourArea;
    int maxNumContours;
    int extraForegroundErosion;
    int edgeSearchAreaWidth;
    int colorSearchAreaWidth;
    int boxSize2;
    int finalAlphaErosion;
    int finalAlphaBlur;
    int boxSize;
    int boxSparsity;
    
private:
    
    void helper(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage);
    
    Boolean bBlur;
    
    vector<ofRectangle> getBoxes();
    int numBoxes;
    
	//--------------------------------------------
	   
    ofxCvColorImage         rgbSample;              // RGB image provided as input
    ofxCvGrayscaleImage     rgbSampleGray;              // Converted to single-channel (brightness)
    ofxCvGrayscaleImage     irSample;               // Depth image provided as input
    ofxCvGrayscaleImage     irErodedFG;             // Mask for certain foreground
    ofxCvGrayscaleImage     irErodedBG;             // Mask for certain background
    ofxCvGrayscaleImage     irMask;                 // Mask for unknown area
    
    IplImage                *laplace;               // Result of edge-detection performed on RGB image
    IplImage                *laplaceC1;                 // a channel
    IplImage                *laplace8UC1;               // converted to 8-bit
    IplImage                *contourWindow;         // irMask, with optional blur
    IplImage                *windowedLaplace;       // laplace * contourWindow
    ofxCvGrayscaleImage     contourImg2;            // "corrected" depth image
    
    ofxCvGrayscaleImage     finalMask;              // result of color-matching segmentation
    ofxCvGrayscaleImage     finalMask1;             // with whited-out foreground
    ofxCvGrayscaleImage     finalMask2;             // with blacked-out background
    ofxCvColorImage         finalMaskInRgb;             // Converted to three (identical) channels
    ofxCvColorImage         final;                  // RGB result
    IplImage                *finalC1;                   // R
    IplImage                *finalC2;                   // G
    IplImage                *finalC3;                   // B
    
    IplImage                *finalRGBA;             // RGBA result
    ofImage                 finalRGBAOf;                // as an object drawable in OF

	
	//--------------------------------------------
	ofxCvContourFinder		contourFinder;
    
	vector<ofRectangle> boxes;
    queue<ofPoint> blobCentroids;
    queue<ofPoint> blobIndices;
    int *npts;
    CvMoments *moments;
    vector<ofRectangle> newRects;
    
};
