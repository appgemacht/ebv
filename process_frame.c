/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions, uint8 Color);

void ProcessFrame(uint8 *pInputImg)
{
	int c; // column index
	int r; // row index
	int i; // point index
	int K; // possible threshold
	int W0; // number of points in class 0
	int W1; // number of points in class 1
	int M0; // sum of point values in class 0
	int M1; // sum of point values in class 1
	float Var; // inter class variance
	float MaxVar; // maximal inter class variance
	int Threshold; // threshold with maximal inter class variance

	int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);

	struct OSC_PICTURE Pic1, Pic2;//we require these structures to use Oscar functions
	struct OSC_VIS_REGIONS ImgRegions;//these contain the foreground objects

	uint32 Hist[256];
	uint8* p;

	if(data.ipc.state.nThreshold != 0) { /* manual threshold */
		Threshold = data.ipc.state.nThreshold * 255 / 100;
		OscLog(INFO, "Manual Threshold: %d\n", Threshold);
	}
	else { /* automatic threshold */
		/* get grayscale histogram */
		p = data.u8TempImage[GRAYSCALE];
		memset(Hist, 0, sizeof(Hist));

		for(i = 0; i < siz; i++) {
			Hist[p[i]] += 1;
		}

		/* OscLog(INFO, "Histogram%d = [", data.ipc.state.nStepCounter);
		for(i = 0; i < 256; i++) {
			OscLog(INFO, "%d ", Hist[i]);
		}
		OscLog(INFO, "]\n"); */

		Threshold = 64;
		MaxVar = 0;
		for(K = 0; K < 256; K++) {
			W0 = 0;
			M0 = 0;
			for(i = 0; i < K; i++) {
				// count points in class 0
				W0 += Hist[i];
				// sum point values in class 0
				M0 += Hist[i]*i;
			}

			W1 = 0;
			M1 = 0;
			for(i = K; i < 256; i++) {
				// count points in class 1
				W1 += Hist[i];
				// sum point values in class 1
				M1 += Hist[i]*i;
			}

			// calculate inter class variance
			Var = W0 * W1 * pow(M0 / (float) W0 - M1 / (float) W1, 2.0);

			if(Var > MaxVar) {
				MaxVar = Var;
				Threshold = K;
			}
		}
		OscLog(INFO, "Automatic Threshold: %d; MaxVar: %f\n", Threshold, MaxVar);

		/* this is the default case */
		for(r = 0; r < siz; r+= nc)/* we strongly rely on the fact that them images have the same size */
		{
			for(c = 0; c < nc; c++)
			{
				/* first determine the foreground estimate */
				data.u8TempImage[THRESHOLD][r+c] = (short) data.u8TempImage[GRAYSCALE][r+c] < Threshold ? 0xff : 0;
			}
		}

		for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
		{
			for(c = 1; c < nc-1; c++)/* we skip the first and last column */
			{
				unsigned char* p = &data.u8TempImage[THRESHOLD][r+c];
				data.u8TempImage[EROSION][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
												 *(p-1)    & *p      & *(p+1)    &
												 *(p+nc-1) & *(p+nc) & *(p+nc+1);
			}
		}

		for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
		{
			for(c = 1; c < nc-1; c++)/* we skip the first and last column */
			{
				unsigned char* p = &data.u8TempImage[EROSION][r+c];
				data.u8TempImage[DILATION][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
												  *(p-1)    | *p      | *(p+1)    |
												  *(p+nc-1) | *(p+nc) | *(p+nc+1);
			}
		}

		//wrap image DILATION in picture struct
		Pic1.data = data.u8TempImage[DILATION];
		Pic1.width = nc;
		Pic1.height = OSC_CAM_MAX_IMAGE_HEIGHT/2;
		Pic1.type = OSC_PICTURE_GREYSCALE;
		//as well as EROSION (will be used as output)
		Pic2.data = data.u8TempImage[EROSION];
		Pic2.width = nc;
		Pic2.height = OSC_CAM_MAX_IMAGE_HEIGHT/2;
		Pic2.type = OSC_PICTURE_BINARY;//probably has no consequences
		//have to convert to OSC_PICTURE_BINARY which has values 0x01 (and not 0xff)
		OscVisGrey2BW(&Pic1, &Pic2, 0x80, false);

		//now do region labeling and feature extraction
		OscVisLabelBinary( &Pic2, &ImgRegions);
		OscVisGetRegionProperties( &ImgRegions);

		//OscLog(INFO, "number of objects %d\n", ImgRegions.noOfObjects);
		//plot bounding boxes both in gray and dilation image
		Pic2.data = data.u8TempImage[GRAYSCALE];
		OscVisDrawBoundingBoxBW( &Pic2, &ImgRegions, 255);
		OscVisDrawBoundingBoxBW( &Pic1, &ImgRegions, 128);
	}
}


/* Drawing Function for Bounding Boxes; own implementation because Oscar only allows colored boxes; here in Gray value "Color"  */
/* should only be used for debugging purposes because we should not drawn into a gray scale image */
OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions, uint8 Color)
{
	 uint16 i, o;
	 uint8 *pImg = (uint8*)picIn->data;
	 const uint16 width = picIn->width;
	 for(o = 0; o < regions->noOfObjects; o++)//loop over regions
	 {
		 /* Draw the horizontal lines. */
		 for (i = regions->objects[o].bboxLeft; i < regions->objects[o].bboxRight; i += 1)
		 {
				 pImg[width * regions->objects[o].bboxTop + i] = Color;
				 pImg[width * (regions->objects[o].bboxBottom - 1) + i] = Color;
		 }

		 /* Draw the vertical lines. */
		 for (i = regions->objects[o].bboxTop; i < regions->objects[o].bboxBottom-1; i += 1)
		 {
				 pImg[width * i + regions->objects[o].bboxLeft] = Color;
				 pImg[width * i + regions->objects[o].bboxRight] = Color;
		 }
	 }
	 return SUCCESS;
}


