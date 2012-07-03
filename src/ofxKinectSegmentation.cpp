#include "ofxKinectSegmentation.h"

ofxKinectSegmentation::ofxKinectSegmentation() {
    KW = 640;
    KH = 480;
    
    minContourArea = 50;
    maxContourArea = KW*KH;
    maxNumContours = 100;
    extraForegroundErosion = 0;
    edgeSearchAreaWidth = 12;
    colorSearchAreaWidth = 1;
    boxSize2 = 20;
    finalAlphaErosion = 2;
    finalAlphaBlur = 7;
    boxSize = 20;
    boxSparsity = 1;
}

void ofxKinectSegmentation::init() {
    rgbSample.allocate(KW, KH);
    rgbSampleGray.allocate(KW, KH);
    irSample.allocate(KW, KH);
    irErodedFG.allocate(KW, KH);
    irErodedBG.allocate(KW, KH);
    irMask.allocate(KW, KH);
    finalMask.allocate(KW, KH);
    finalMask1.allocate(KW, KH);
    finalMask2.allocate(KW, KH);
    finalMaskInRgb.allocate(KW, KH);
    final.allocate(KW, KH);
    finalC1 = cvCreateImage(cvSize(KW, KH), IPL_DEPTH_8U, 1);
    finalC2 = cvCreateImage(cvSize(KW, KH), IPL_DEPTH_8U, 1);
    finalC3 = cvCreateImage(cvSize(KW, KH), IPL_DEPTH_8U, 1);
    
    laplaceC1 = cvCreateImage(cvSize(KW, KH), IPL_DEPTH_32F, 1);
    laplace8UC1 = cvCreateImage(cvSize(KW, KH), IPL_DEPTH_8U, 1);
    contourWindow = cvCreateImage(cvSize(KW, KH), IPL_DEPTH_8U, 1);
    windowedLaplace = cvCreateImage(cvSize(KW, KH), IPL_DEPTH_8U, 1);
    
    finalRGBA = cvCreateImage(cvSize(KW, KH), IPL_DEPTH_8U, 4);
    finalRGBAOf.allocate(KW, KH, OF_IMAGE_COLOR_ALPHA);
    
    contourImg2.allocate(KW, KH);
}

void ofxKinectSegmentation::getBitMask(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage, ofxCvGrayscaleImage *output) {
    bBlur = false;
    helper(rgbImage, depthImage);
    cvCopy(finalMask.getCvImage(), output->getCvImage());
}

void ofxKinectSegmentation::getGrayscaleMask(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage, ofxCvGrayscaleImage *output) {
    bBlur = true;
    helper(rgbImage, depthImage);
    cvCopy(finalMask.getCvImage(), output->getCvImage());
}


void ofxKinectSegmentation::getRGB(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage, ofxCvColorImage *output) {
    bBlur = true;
    helper(rgbImage, depthImage);
    cvCvtColor(finalMask.getCvImage(), finalMaskInRgb.getCvImage(), CV_GRAY2RGB);
    cvAnd(rgbSample.getCvImage(), finalMaskInRgb.getCvImage(), final.getCvImage());
}


void ofxKinectSegmentation::getRGBA(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage, ofImage *output) {
    bBlur = true;
    helper(rgbImage, depthImage);
    cvSplit(rgbSample.getCvImage(), finalC1, finalC2, finalC3, NULL);
    cvMerge(finalC1, finalC2, finalC3, finalMask.getCvImage(), finalRGBA);
    (*output).setFromPixels((unsigned char *) finalRGBA->imageData, KW, KH, OF_IMAGE_COLOR_ALPHA);
}

void ofxKinectSegmentation::helper(ofxCvColorImage rgbImage, ofxCvGrayscaleImage depthImage) {
    
    rgbSample = rgbImage;
    irSample = depthImage;
    cvCvtColor(rgbSample.getCvImage(), rgbSampleGray.getCvImage(), CV_RGB2GRAY);
    
    //finalMask = depthImage;
    //return; // 33 ms
    
    // first pass
    // Create masks for known foreground, known background, and the stripe-like unknown area between them
    irErodedFG = irSample;
    cvDilate(irErodedFG.getCvImage(), irErodedFG.getCvImage(), NULL, 4);
    cvErode(irErodedFG.getCvImage(), irErodedFG.getCvImage(), NULL, 4 + extraForegroundErosion + edgeSearchAreaWidth/2.0 + 0.5);
    irErodedBG = irSample;
    irErodedBG.invert();
    cvErode(irErodedBG.getCvImage(), irErodedBG.getCvImage(), NULL, edgeSearchAreaWidth/2.0);
    cvOr(irErodedFG.getCvImage(), irErodedBG.getCvImage(), irMask.getCvImage());
    irMask.invert();
    
    // Perform edge detection on the RGB image and apply the unknown-area mask
    cvLaplace(rgbSampleGray.getCvImage(), laplaceC1);
    cvConvertScale(laplaceC1, laplace8UC1);
    cvAnd(irMask.getCvImage(), laplace8UC1, windowedLaplace);
    
    // Find contour points around depth image and get a set of squares around them
    contourFinder.findContours(irSample, minContourArea, maxContourArea, maxNumContours, true, true);
    boxes = getBoxes();
    
    // create space for a new set of contour points, adjusted by RGB edge detection
    CvPoint *polygons[contourFinder.nBlobs];
    for (int i = 0; i < contourFinder.nBlobs; i++) {
        polygons[i] = new CvPoint[contourFinder.blobs.at(i).nPts];
    }
    
    // get a new contour point from each ROI in "boxes", by finding the mean of the RGB edge image inside the ROI
    moments = new CvMoments();
    int pointCounter = 0, polyCounter = 0;
    for (int i = 0; i < boxes.size(); i++) {
        ofRectangle roi = boxes.at(i);
        cvSetImageROI(windowedLaplace, cvRect(roi.x, roi.y, roi.width, roi.height));
        cvMoments(windowedLaplace, moments);
        if (pointCounter >= npts[polyCounter]) {
            pointCounter = 0;
            polyCounter++;
        }
        if (moments->m00) {
            polygons[polyCounter][pointCounter] = cvPoint(roi.x + moments->m10/moments->m00, roi.y + moments->m01/moments->m00);
            pointCounter++;
        } else {
            npts[polyCounter]--;
        }
    }
    cvResetImageROI(windowedLaplace);
    
    // construct a "corrected depth image" from these new contour points
    contourImg2.set(0);
    if (contourFinder.nBlobs) {
        cvFillPoly(contourImg2.getCvImage(), polygons, npts, contourFinder.nBlobs, CvScalar(CV_RGB(255, 255, 255)));
    }
    
    //finalMask = contourImg2;
    //return; // 53 ms
    
    // second pass
    // Create updated masks for foreground, background, and unknown area
    irErodedFG = contourImg2;
    cvErode(irErodedFG.getCvImage(), irErodedFG.getCvImage(), NULL, colorSearchAreaWidth);
    irErodedBG = contourImg2;
    irErodedBG.invert();
    cvErode(irErodedFG.getCvImage(), irErodedFG.getCvImage(), NULL, colorSearchAreaWidth);
    
    newRects.clear();
    
    // get squares around points in updated contour (getBoxes() abridged)
    vector<ofPoint> pts;
    int boxSize = boxSize2;
    for (int i = 0; i < contourFinder.nBlobs; i++) {
        CvPoint *pts = polygons[i];
        for (int j = 0; j < npts[i]; j++) {
            ofRectangle r(pts[j].x - boxSize/2, pts[j].y - boxSize/2, boxSize, boxSize);
            //ofRectangle r(pts[j].x - boxSize/2, pts[j].y - 5, boxSize, 10);
            newRects.push_back(r);
        }
    }
    boxes = newRects;
    
    // loop through boxes and segment unknown area by comparings the colors of the pixels in the unknown area
    // to the colors of the pixels in the known FG and the known BG in that particular box-ROI
    finalMask = irSample;
    for (int i = 0; i < boxes.size(); i++) {
        ofRectangle roi = boxes.at(i);
        
        rgbSampleGray.setROI(roi);
        irErodedFG.setROI(roi);
        irErodedBG.setROI(roi);
        finalMask.setROI(roi);
        
        double meanF;
        double stdDevF;
        double meanB;
        double stdDevB;
        
        double tempF;
        double tempB;
        
        cvMean_StdDevMask(rgbSampleGray.getCvImage(), irErodedFG.getCvImage(), &tempF, &stdDevF);
        cvMean_StdDevMask(rgbSampleGray.getCvImage(), irErodedBG.getCvImage(), &tempB, &stdDevB);
        
        if (tempF) {
            meanF = tempF;
        }
        if (tempB) {
            meanB = tempB;
        }
        
        double threshold = (meanF + meanB) / 2;
        
        cvCopy(rgbSampleGray.getCvImage(), finalMask.getCvImage());
        finalMask.threshold(round(threshold));
        
        if (meanB > meanF) {
            finalMask.invert();
        }
    }
    rgbSampleGray.resetROI();
    irErodedFG.resetROI();
    irErodedBG.resetROI();
    finalMask.resetROI();
    
    // reapply known foreground and known background
    cvOr(finalMask.getCvImage(), irErodedFG.getCvImage(), finalMask1.getCvImage());
    irErodedBG.invert();
    cvAnd(finalMask1.getCvImage(), irErodedBG.getCvImage(), finalMask2.getCvImage());
    irErodedBG.invert();
    
    if (bBlur) {
        // blur edges
        cvErode(finalMask2.getCvImage(), finalMask1.getCvImage(), NULL, finalAlphaErosion);
        cvSmooth(finalMask1.getCvImage(), finalMask.getCvImage(), CV_BLUR, 2*finalAlphaBlur + 1);
    } else {
        cvCopy(finalMask2.getCvImage(), finalMask.getCvImage());
    }
    
    while (!blobCentroids.empty()) {
        blobCentroids.pop();
    }
    for (int i = 0; i < contourFinder.nBlobs; i++) {
        delete polygons[i];
    }
    if (moments != NULL) {
        delete moments;
    }
    
    // 70 ms
}

// checks contourFinder for contour points and returns ROI boxes around them
// lengthy to accomodate for the possibility of boxSparsity > 1
vector<ofRectangle> ofxKinectSegmentation::getBoxes() {
    if (npts != NULL) {
        delete npts;
    }
    npts = new int[contourFinder.nBlobs];
    vector<ofRectangle> out;
    vector<ofPoint> pts;
    ofPoint lastPoint;
    while (!blobIndices.empty()) {
        blobIndices.pop();
    }
    while (!blobCentroids.empty()) {
        blobCentroids.pop();
    }
    int k = 0;
    for (int i = 0; i < contourFinder.nBlobs; i++) {
        blobIndices.push(k);
        pts = contourFinder.blobs.at(i).pts;
        lastPoint = pts.at(0);
        int lastPointIndex = 0;
        int count = 0;
        for (int j = 0; j < pts.size(); j++) {
            if (pts.at(j).distance(lastPoint) >= boxSparsity || j - lastPointIndex >= boxSparsity) {
                ofRectangle r(pts.at(j).x - boxSize/2, pts.at(j).y - boxSize/2, boxSize, boxSize);
                //ofRectangle r(pts.at(j).x - boxSize/2, pts.at(j).y, boxSize, 1);
                out.push_back(r);
                lastPoint = pts.at(j);
                lastPointIndex = j;
                k++;
                count++;
            }
        }
        npts[i] = count;
    }
    return out;
}
