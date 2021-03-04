#pragma once

// Camera intrinsic (GML Camera Calibration Toolbox)
// 1280 X 720
#define fx 1139.9981584289369
#define fy 1137.6229966388676
#define cx 634.8484343572419
#define cy 347.44221572365188
#define k1 0.0090647293510975422
#define k2 0.16017870180011257
#define p1 -0.00498025879131914 
#define p2 -0.00016744443318946387


// HSV
#define LowH 25
#define HighH 35
#define LowS 50
#define HighS 255
#define LowV 0
#define HighV 255

// Window
#define WINDOW_WIDTH 320
#define WINDOW_HEIGHT 240

// MARKER
#define NUM_MARKER_POINTS 6
#define EPSILON 0.03 //epsilon for approxPolyDP

// Calculation
// Contour min area (for processContours)
#define minCntSize 150

// isPointOnVectorLine Error
#define err 20 //pixel
