#pragma once
#define OBJECTSCANDLL_EXPORTS

#ifdef OBJECTSCANDLL_EXPORTS
#define OBJECTSCANDLL_API __declspec(dllexport) 
#else
#define OBJECTSCANDLL_API __declspec(dllimport)
#endif

#include <math.h> 
#include <iostream>
#include "opencv2/opencv.hpp""
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>"


using namespace cv;
using namespace std;

namespace ObjectScan {


	class ObjectScan
	{
		private:
		// This class is exported from the MathFuncsDll.dll
		static Mat dst;

		// to store infos about the max postions
		static int**   maxPosArray;
		static double** globalPositons;

		// store information of the linear equation system
		static double** formelArray;
		static bool* formelFlip;
		static int** formelborder;

		// store of information about the graph
		static double*** graphArray;
		static double** resultPoints;
		static bool* graphVisited;


		static int resultCounter;

		private:
		//inernal functions
		static Mat Hough( Mat newimage );
		static Mat CannyThreshold( Mat orginal, int ratio, int lowThreshold );
		static Mat GenerateLinearSystem( int numberOfMax, Mat newimage, Mat image, int offsetx, int offsety );
		static void GenerateGraph( int numberOfMax, Mat newimage );
		static bool recusivGraph( int corners, int last, int maximas );
		static void FindOverlayEdge( int numberOfMax, int diversity, Mat newimage, int offsetx, int offsety );
		static bool SolveGraph( int corners, int maximas );
		static int* NextMaximum( Mat localimage );
		static Mat FindAllMaximas( int numberOfMax, Mat space, Mat image );
		static int CollectMaximas( int brightness, Mat space );

		public:
		// Returns a + b
		static OBJECTSCANDLL_API bool scan( Mat source, int corners, int pixeloffset , int maximasa, int upperlimita );
		static OBJECTSCANDLL_API bool scanDebug( Mat source, int corners, int pixeloffset, int maximasa, int upperlimita, int offsestx, int offsety, int debugbitmap );
		

	};
}