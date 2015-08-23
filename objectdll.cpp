// objectdll.cpp : Definiert die exportierten Funktionen für die DLL-Anwendung.
//

#include "stdafx.h"
#include "objectdll.h"
using namespace cv;

namespace ObjectScan {
	// OpenCV-G.cpp : Definiert den Einstiegspunkt für die Konsolenanwendung.
	//


	Mat ObjectScan::dst;

	// to store infos about the max postions
	int**   ObjectScan::maxPosArray;
	double** ObjectScan::globalPositons;

	// store information of the linear equation system
	double** ObjectScan::formelArray;
	bool* ObjectScan::formelFlip;
	int** ObjectScan::formelborder;

	// store of information about the graph
	double*** ObjectScan::graphArray;
	double** ObjectScan::resultPoints;
	bool* ObjectScan::graphVisited;


	int ObjectScan::resultCounter = 0;



	/*
	Hough transform Implementation
	@param m Matrix of the corners
	*/
	Mat ObjectScan::Hough( Mat newimage ) {

		Mat mat;
		int sizeX = newimage.cols;
		int sizeY = newimage.rows;
		int rangeofTheta = 360;
		int r = 0;
		int rMax = 0;
		int thetaMax;
		int thetaMin;
		double theta;
		//Mat gs_rgb(m.size(), CV_8UC1);
		//	cvtColor(m,gs_rgb, CV_GRAY2BGR);
		//namedWindow( "Input-Window", WINDOW_AUTOSIZE );// Create a window for display.
		//imshow( "Input-Window", m );

		rMax = (int) ( sqrt( ( (double) sizeX / 2.0*(double) sizeX / 2.0 ) + ( (double) sizeY / 2.0*(double) sizeY / 2.0 ) ) );///2);
																															   //CvMat* mat = cvCreateMatHeader(,(, m.type);
		mat.create( rMax + 1, rangeofTheta + 1, CV_8UC1 );
		mat = Scalar::all( 0 );
		//cvCreateData(mat);

		thetaMin = 0;
		thetaMax = 360;

		for ( int x = 0; x < sizeX; x++ )
		{
			for ( int y = 0; y < sizeY; y++ )
			{
				if ( newimage.at<uchar>( y, x ) > 0 )//pixel gray?
				{
					for ( int t = 0; t < rangeofTheta; t += 1 )
					{
						theta = CV_PI*( (double) t / 180.0 );
						double C = cos( ( theta ) );
						double S = sin( ( theta ) );
						double f1 = (double) x - ( (double) floor( sizeX / 2.0 ) );
						double f2 = (double) y - ( (double) floor( sizeY / 2.0 ) );
						double dy = ( f1 * C ) + ( f2* S );
						r = (int) floor( dy + .5 );
						r = -( r - rMax );
						if ( !( r<0 ) && !( r>rMax ) )
						{
							if ( mat.at<uchar>( r, t ) < 255 ) {
								mat.at<uchar>( r, t ) = mat.at<uchar>( r, t ) + 1;
							}

						}


					}
				}
			}
		}
		//mat.copyTo( space, mat);
		//return hog space
		return mat;
	}
	/*


	@param m Matrix of the gray picture
	*/
	Mat ObjectScan::CannyThreshold( Mat orginal, int ratio, int lowThreshold ) {
		/// Reduce noise with a kernel 3x3
		int edgeThresh = 1;

		int const max_lowThreshold = 100;

		int kernel_size = 3;

		Mat detected_edges;
		Mat gray_src;
		/// Create a matrix of the same type and size as src (for dst)

		/// Convert the image to grayscale
		cvtColor( orginal, gray_src, CV_BGR2GRAY );
		detected_edges.create( orginal.size(), gray_src.type() );

		blur( gray_src, detected_edges, Size( 3, 3 ) );
		/// Using Canny's output as a mask, we display our result
		Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );


		gray_src.release();
		return detected_edges;
	}
	/*
	Generate the formels for the picture on hand of the numbers of maximums
	@param numberOfMax The number of points you seek in the hough space
	*/
	Mat ObjectScan::GenerateLinearSystem( int numberOfMax, Mat newimage, Mat image, int offsetx, int offsety ) {
		int xPixel, yPixel = 0;
		//	int yPos,XPos;
		int x1, x2, y1, y2 = 0;
		int r;
		double t, m, b;
		bool weHavefirst, flip, weHavesecound;
		int thickness = 1;
		int lineType = 8;

		Mat empty;
		empty.create( image.size(), image.type() );
		empty = Scalar::all( 0 );

		for ( int i = 0;i < numberOfMax; i++ ) {

			t = globalPositons[ i ][ 0 ]; //Winkel
			r = (int) globalPositons[ i ][ 1 ]; //Distanze

			weHavefirst = false;
			weHavesecound = false;
			for ( int x = 0 + offsetx; x < newimage.cols - offsetx; x++ ) {

				yPixel = (int) ( ( (double) r / sin( t ) ) - ( ( (double) x - (double) newimage.cols / 2.0 )*( cos( t ) / sin( t ) ) ) + (double) newimage.rows / 2.0 );
				yPixel = yPixel + offsety;
				x = x + offsetx;

				if ( yPixel < newimage.rows && yPixel >= 0 && x >= 0 && x > newimage.cols ) {
					if ( weHavefirst == true ) {
						//P1
						x2 = x;
						y2 = yPixel;
						weHavesecound = true;

					}
					else
					{
						//P2
						x1 = x;
						y1 = yPixel;
						weHavefirst = true;

					}

				}
				x = x - offsetx;
			}
			if ( weHavesecound == false || weHavefirst == false ) {
				weHavefirst = false;
				weHavesecound = false;

				for ( int y = 0 + offsety; y < newimage.rows - offsety;y++ ) {

					xPixel = (int) ( ( (double) r / cos( t ) ) - ( ( (double) y - (double) newimage.rows / 2.0 )*tan( t ) ) + (double) newimage.cols / 2.0 );
					xPixel = xPixel + offsetx;
					y = y + offsety;
					if ( xPixel < newimage.cols && xPixel >= 0 && y >= 0 && y < newimage.rows ) {
						if ( weHavefirst == true ) {
							//P1
							x2 = xPixel;
							y2 = y;
							weHavesecound = true;

						}
						else
						{
							//P2
							x1 = xPixel;
							y1 = y;
							weHavefirst = true;

						}
					}
					y = y - offsety;
				}

			}
			Point P1 = Point( x1, y1 );
			Point P2 = Point( x2, y2 );

			line( empty, P1, P2, Scalar( 0, 255, 0 ), thickness, lineType );

			if ( ( x2 - x1 ) != 0 ) {
				m = ( (double) ( y2 - y1 ) / ( x2 - x1 ) );
				b = (double) y1 - ( (double) ( m * x1 ) );
				flip = false;
			}
			else
			{

				m = ( (double) ( x2 - x1 ) / ( y2 - y1 ) );
				b = (double) x1 - ( (double) ( m * y1 ) );
				flip = true;

			}
			//	cout <<  "Formel  Nr i : "<< i << " steigung: " << m << " abstand: "<< b << std::endl ;
			if ( flip ) {
				//cout <<  "Formel wurde gedreht "<< std::endl ;
			}
			else
			{
				//cout <<  "Formel wurde nicht gedreht "<< std::endl ;
			}
			formelArray[ i ][ 0 ] = m;
			formelArray[ i ][ 1 ] = b;
			formelFlip[ i ] = flip;
		}
		return empty;
	}
	/*
	Seek the number of maximus in the hough space
	@param numberOfMax The number of point you have from the hough space
	@param abweichung Size of the space that wille be checked around the point
	*/
	void ObjectScan::FindOverlayEdge( int numberOfMax, int diversity, Mat newimage, int offsetx, int offsety ) {
		int xPixel, yPixel = 0;
		//	int yPos,XPos;
		int x1, x2, y1, y2 = 0;
		int r;
		double t;
		bool weHavefirst, weHavesecound;
		int thickness = 1;
		int lineType = 8;
		int startx, starty = 0;
		int endx, endy = 0;

		//Mat empty;

		//empty.create(newimage.size(),newimage.type() );
		//empty = Scalar::all( 0 );



		for ( int i = 0;i < numberOfMax; i++ ) {

			t = globalPositons[ i ][ 0 ];
			r = (int) globalPositons[ i ][ 1 ];
			startx = 0;
			starty = 0;
			endx = 0;
			endy = 0;
			bool lost = false;
			weHavefirst = false;
			weHavesecound = false;
			int num_entries = 0;
			int entry_start_X = 0;
			int entry_start_Y = 0;
			int entry_ende_X = 0;
			int entry_ende_Y = 0;
			int entry_lenght = 0;

			for ( int x = 0 + offsetx; x < newimage.cols - offsetx; x++ ) {
				bool need_new_entry = false;
				bool skip = false;
				yPixel = (int) ( ( (double) r / sin( t ) ) - ( ( (double) x - (double) newimage.cols / 2.0 )*( cos( t ) / sin( t ) ) ) + (double) newimage.rows / 2.0 );
				yPixel = yPixel + offsety;
				x = x + offsetx;
				if ( yPixel < newimage.rows && yPixel >= 0 && x < newimage.cols && x >= 0 ) {

					if ( weHavefirst == true ) {


						if ( newimage.at<uchar>( yPixel, x ) > 0 ) {
							endx = x;
							endy = yPixel;
							weHavesecound = true;
						}
						else
						{
							bool foundsome = false;
							for ( int m = x - diversity; m < x + diversity; m++ ) {
								for ( int j = yPixel - diversity; j < yPixel + diversity; j++ ) {
									if ( m >= 0 && j >= 0 && m < newimage.cols && j < newimage.rows ) {
										if ( newimage.at<uchar>( j, m ) > 0 ) {
											endx = x;
											endy = yPixel;
											weHavesecound = true;
											foundsome = true;
										}
									}
								}
							}
							if ( foundsome == false ) {
								weHavesecound = true;
								need_new_entry = true;
							}
						}
					}
					else
					{
						//P2
						x1 = x;
						y1 = yPixel;

						if ( newimage.at<uchar>( yPixel, x ) > 0 ) {
							startx = x;
							starty = yPixel;
							weHavefirst = true;
						}
						else
						{
							for ( int m = x - diversity; m < x + diversity; m++ ) {
								for ( int j = yPixel - diversity; j < yPixel + diversity; j++ ) {
									if ( m >= 0 && j >= 0 && m < newimage.cols && j < newimage.rows ) {
										if ( newimage.at<uchar>( j, m ) > 0 ) {
											startx = x;
											starty = yPixel;
											weHavefirst = true;
										}
									}
								}
							}
						}
					}

				}
				else
				{
					if ( weHavefirst == true && weHavesecound == true ) {
						need_new_entry = true;
					}
				}
				/*if( i != 0){
				cout <<  "[DEBUG]  entry_start_x: "<< entry_start_X << "  \n";
				cout <<  "[DEBUG]  entry_start_y: "<< entry_start_Y << "  \n";
				cout <<  "[DEBUG]  entry_ende_x: "<< entry_ende_X << "  \n";
				cout <<  "[DEBUG]  entry_ende_Y: "<< entry_ende_Y << "  \n";
				cout <<  "[DEBUG]  startx: "<< startx << "  \n";
				cout <<  "[DEBUG]  starty: "<< starty << "  \n";
				cout <<  "[DEBUG]  endex: "<< endx << "  \n";
				cout <<  "[DEBUG]  endy: "<< endy << "  \n" << std::endl ;
				if(weHavefirst == true){
				cout <<  "[DEBUG]  weHavefirst true  \n";
				}
				else
				{
				cout <<  "[DEBUG]  weHavefirst false  \n";
				}
				if(weHavesecound == true){
				cout <<  "[DEBUG]  weHavesecound true  \n";
				}
				else
				{
				cout <<  "[DEBUG]  weHavesecound false  \n";
				}

				}*/
				if ( weHavesecound == true && endx == 0 && endy == 0 && weHavefirst == true ) {
					if ( startx != 0 ) {
						endx = startx;
					}
					else
					{
						if ( entry_ende_X != 0 ) {
							endx = entry_ende_X;
						}
						else
						{
							//	startx = 0;
						}

					}



					if ( starty != 0 ) {
						endy = starty;
					}
					else
					{
						if ( entry_ende_Y != 0 ) {
							endy = entry_ende_Y;
						}
						else
						{
							//	starty = 0;
						}
					}
					skip = true;
					weHavefirst = false;
					//need_new_entry = true;
				}
				if ( need_new_entry == true ) {

					int newlenght = (int) sqrt( pow( ( startx - endx ), 2 ) + pow( ( starty - endy ), 2 ) );

					if ( newlenght > entry_lenght ) {
						entry_lenght = newlenght;
						entry_start_X = startx;
						entry_start_Y = starty;
						entry_ende_X = endx;
						entry_ende_Y = endy;
					}
					if ( newlenght > 0 ) {
						num_entries = num_entries + 1;
					}
					weHavefirst = false;
					weHavesecound = false;
					startx = 0;
					starty = 0;
					endy = 0;
					endx = 0;
				}
				else
				{
					if ( skip == true ) {
						skip = false;
						weHavefirst = false;
						weHavesecound = false;
						//need_new_entry = false;

						endy = 0;
						endx = 0;
					}
				}
				x = x - offsetx;
			}


			/*if( i != 0){
			cout <<  "[DEBUG] alpha entry_start_x: "<< entry_start_X << "  \n";
			cout <<  "[DEBUG]  entry_start_y: "<< entry_start_Y << "  \n";
			cout <<  "[DEBUG]  entry_ende_x: "<< entry_ende_X << "  \n";
			cout <<  "[DEBUG]  entry_ende_Y: "<< entry_ende_Y << "  \n";
			cout <<  "[DEBUG]  startx: "<< startx << "  \n";
			cout <<  "[DEBUG]  starty: "<< starty << "  \n";
			cout <<  "[DEBUG]  endex: "<< endx << "  \n";
			cout <<  "[DEBUG]  endy: "<< endy << "  \n" << std::endl ;
			if(weHavefirst == true){
			cout <<  "[DEBUG]  weHavefirst true  \n";
			}
			else
			{
			cout <<  "[DEBUG]  weHavefirst false  \n";
			}
			if(weHavesecound == true){
			cout <<  "[DEBUG]  weHavesecound true  \n";
			}
			else
			{
			cout <<  "[DEBUG]  weHavesecound false  \n";
			}

			}*/
			if ( ( weHavesecound == false || weHavefirst == false ) && num_entries < 1 ) {
				weHavefirst = false;
				weHavesecound = false;

				for ( int y = 0 + offsety; y < newimage.rows - offsety;y++ ) {
					bool need_new_entry = false;
					bool skip = false;

					xPixel = (int) ( ( (double) r / cos( t ) ) - ( ( (double) y - (double) newimage.rows / 2.0 )*tan( t ) ) + (double) newimage.cols / 2.0 );
					xPixel = xPixel + offsetx;
					y = y + offsety;
					if ( xPixel < newimage.cols && xPixel >= 0 && y < newimage.rows && y >= 0 ) {
						if ( weHavefirst == true ) {

							if ( newimage.at<uchar>( y, xPixel ) > 0 ) {
								endx = xPixel;
								endy = y;
								weHavesecound = true;
							}
							else
							{
								bool foundsome = false;
								for ( int m = xPixel - diversity; m < xPixel + diversity; m++ ) {
									for ( int j = y - diversity; j < y + diversity; j++ ) {
										if ( m >= 0 && j >= 0 && m < newimage.cols && j < newimage.rows && j >= 0 ) {
											if ( newimage.at<uchar>( j, m ) > 0 ) {
												endx = xPixel;
												endy = y;
												weHavesecound = true;
												foundsome = true;
											}
										}
									}
								}
								if ( foundsome == false ) {
									weHavesecound = true;
									need_new_entry = true;
								}
							}
						}
						else
						{
							//P2
							x1 = xPixel;
							y1 = y;

							if ( newimage.at<uchar>( y, xPixel ) > 0 ) {
								startx = xPixel;
								starty = y;
								weHavefirst = true;
							}
							else
							{
								for ( int m = xPixel - diversity; m < xPixel + diversity; m++ ) {
									for ( int j = y - diversity; j < y + diversity; j++ ) {
										if ( m >= 0 && j >= 0 && m < newimage.cols && j < newimage.rows && j >= 0 ) {
											if ( newimage.at<uchar>( j, m ) > 0 ) {
												startx = xPixel;
												starty = y;
												weHavefirst = true;
											}
										}
									}
								}
							}
						}
					}
					else
					{
						if ( weHavefirst == true && weHavesecound == true ) {
							need_new_entry = true;
						}
					}
					/*if( i != 0){
					cout <<  "[DEBUG]  beta entry_start_x: "<< entry_start_X << "  \n";
					cout <<  "[DEBUG]  entry_start_y: "<< entry_start_Y << "  \n";
					cout <<  "[DEBUG]  entry_ende_x: "<< entry_ende_X << "  \n";
					cout <<  "[DEBUG]  entry_ende_Y: "<< entry_ende_Y << "  \n";
					cout <<  "[DEBUG]  startx: "<< startx << "  \n";
					cout <<  "[DEBUG]  starty: "<< starty << "  \n";
					cout <<  "[DEBUG]  endex: "<< endx << "  \n";
					cout <<  "[DEBUG]  endy: "<< endy << "  \n" << std::endl ;
					if(weHavefirst == true){
					cout <<  "[DEBUG]  weHavefirst true  \n";
					}
					else
					{
					cout <<  "[DEBUG]  weHavefirst false  \n";
					}
					if(weHavesecound == true){
					cout <<  "[DEBUG]  weHavesecound true  \n";
					}
					else
					{
					cout <<  "[DEBUG]  weHavesecound false  \n";
					}

					}*/
					if ( weHavesecound == true && endx == 0 && endy == 0 && weHavefirst == true ) {
						if ( startx != 0 ) {
							endx = startx;
						}
						else
						{
							if ( entry_ende_X != 0 ) {
								endx = entry_ende_X;
							}
							else
							{
								//	startx = 0;
							}

						}



						if ( starty != 0 ) {
							endy = starty;
						}
						else
						{
							if ( entry_ende_Y != 0 ) {
								endy = entry_ende_Y;
							}
							else
							{
								//	starty = 0;
							}
						}
						skip = true;
						weHavefirst = false;
						//need_new_entry = true;
					}
					if ( need_new_entry == true ) {
						int newlenght = (int) sqrt( pow( ( startx - endx ), 2 ) + pow( ( starty - endy ), 2 ) );

						if ( newlenght > entry_lenght ) {
							entry_lenght = newlenght;
							entry_start_X = startx;
							entry_start_Y = starty;
							entry_ende_X = endx;
							entry_ende_Y = endy;
						}
						startx = 0;
						starty = 0;
						endy = 0;
						endx = 0;
						weHavefirst = false;
						weHavesecound = false;
						num_entries = num_entries + 1;
					}
					else
					{
						if ( skip == true ) {
							skip = false;
							//startx= 0;
							//starty =0;
							endy = 0;
							endx = 0;
							weHavefirst = false;
							weHavesecound = false;
							//need_new_entry = false;
						}
					}
					y = y - offsety;
				}

			}
			/*	if( i != 0){
			cout <<  "[DEBUG] gamma entry_start_x: "<< entry_start_X << "  \n";
			cout <<  "[DEBUG]  entry_start_y: "<< entry_start_Y << "  \n";
			cout <<  "[DEBUG]  entry_ende_x: "<< entry_ende_X << "  \n";
			cout <<  "[DEBUG]  entry_ende_Y: "<< entry_ende_Y << "  \n";
			cout <<  "[DEBUG]  startx: "<< startx << "  \n";
			cout <<  "[DEBUG]  starty: "<< starty << "  \n";
			cout <<  "[DEBUG]  endex: "<< endx << "  \n";
			cout <<  "[DEBUG]  endy: "<< endy << "  \n" << std::endl ;
			if(weHavefirst == true){
			cout <<  "[DEBUG]  weHavefirst true  \n";
			}
			else
			{
			cout <<  "[DEBUG]  weHavefirst false  \n";
			}
			if(weHavesecound == true){
			cout <<  "[DEBUG]  weHavesecound true  \n";
			}
			else
			{
			cout <<  "[DEBUG]  weHavesecound false  \n";
			}

			}*/
			if ( endx != 0 && endy != 0 && weHavesecound == true ) {
				int newlenght = (int) sqrt( pow( ( startx - endx ), 2 ) + pow( ( starty - endy ), 2 ) );

				if ( newlenght > entry_lenght ) {
					entry_lenght = newlenght;
					entry_start_X = startx;
					entry_start_Y = starty;
					entry_ende_X = endx;
					entry_ende_Y = endy;
				}

				num_entries = num_entries + 1;
			}

			startx = entry_start_X;
			starty = entry_start_Y;
			endx = entry_ende_X;
			endy = entry_ende_Y;

			int minx, maxx, miny, maxy;

			if ( endx > startx ) {
				maxx = endx; minx = startx;
			}
			else {
				maxx = startx; minx = endx;
			};
			if ( endy > starty ) {
				maxy = endy; miny = starty;
			}
			else {
				maxy = starty; miny = endy;
			};

			formelborder[ i ][ 0 ] = minx;
			formelborder[ i ][ 1 ] = maxx;
			formelborder[ i ][ 2 ] = miny;
			formelborder[ i ][ 3 ] = maxy;
			Point P3 = Point( startx, starty );
			Point P4 = Point( endx, endy );

			line( dst, P3, P4, Scalar( 0, 255, 0 ), thickness, lineType );
		}



	}
	/*
	Find the points where the lines cross
	@param numberOfMax The number of point you have from the hough space

	*/
	void ObjectScan::GenerateGraph( int numberOfMax, Mat newimage ) {
		double m1, b1, m2, b2, m3, b3;
		bool flip1, flip2;
		double x1, y1;


		for ( int x = 0; x < numberOfMax; x++ )
		{
			m1 = formelArray[ x ][ 0 ];
			b1 = formelArray[ x ][ 1 ];
			flip1 = formelFlip[ x ];

			for ( int y = 0; y < numberOfMax; y++ )
			{
				bool found = false;
				if ( x != y ) {
					m2 = formelArray[ y ][ 0 ];
					b2 = formelArray[ y ][ 1 ];
					flip2 = formelFlip[ y ];
					if ( flip1 && !flip2 ) {

						y1 = m2*b1 + b2;
						x1 = b1;
						found = true;
					}
					else
					{
						if ( flip2 && !flip1 ) {

							y1 = m1*b2 + b1;
							x1 = b2;
							found = true;
						}
						else
						{
							if ( !flip1 && !flip2 ) {
								m3 = m1 + ( m2*( -1 ) );
								b3 = b1 + ( b2*( -1 ) );
								m3 = m3 *-1;
								if ( m3 != 0.0 ) {
									x1 = b3 / m3;

									y1 = m1*x1 + b1;
									found = true;
								}
								else
								{

									//	cout <<  "Same linear systeme never cross or infinity solutions "<< std::endl ;
									//Same linear systeme never cross or infinity solutions
								}

							}
							else
							{
								//		cout <<  "Never cross both infinity to y  "<< std::endl ;
								//Never cross both infinity to y
							}

						}

					}
				}
				//add
				//make sure we don't have it yet and we found a new point;
				if ( graphArray[ y ][ x ][ 4 ] == 0.0 && found == true ) {
					//cout <<  "Try to add new  crossing  x1: "<< x1 << " y1: "<< y1 <<  std::endl ;
					//we only count crossing point in the window
					int minx1 = formelborder[ x ][ 0 ];
					int maxx1 = formelborder[ x ][ 1 ];
					int miny1 = formelborder[ x ][ 2 ];
					int maxy1 = formelborder[ x ][ 3 ];

					int minx2 = formelborder[ y ][ 0 ];
					int maxx2 = formelborder[ y ][ 1 ];
					int miny2 = formelborder[ y ][ 2 ];
					int maxy2 = formelborder[ y ][ 3 ];

					if ( x1 >= 0 && x1 < newimage.cols && y1 >= 0 && y1 < newimage.rows && x1 >= minx1 && x1 >= minx2 && x1 <= maxx1 && x1 <= maxx2 && y1 >= miny1 && y1 >= miny2 && y1 <= maxy1 && y1 <= maxy2 ) {
						//	cout <<  "add new crossing  x1: "<< x1 << " y1: "<< y1 << "  x: "<< x << " y: "<< y << std::endl ;
						graphArray[ x ][ y ][ 0 ] = x1;
						graphArray[ x ][ y ][ 1 ] = y1;
						graphArray[ x ][ y ][ 2 ] = x;
						graphArray[ x ][ y ][ 3 ] = y;
						graphArray[ x ][ y ][ 4 ] = 1.0;
					}
				}
			}
		}

	}
	/*
	Recursiv helper function
	@param corners remaining length of the graph
	@param last index of the last visted edge
	@param maximas total number of found linear systems
	*/
	bool ObjectScan::recusivGraph( int corners, int last, int maximas ) {

		int length = maximas;
		int length2 = maximas;
		//cout <<  "recusiveGraph length: "<< length << std::endl ;
		//cout <<  "recusiveGraph lenght2: "<< length2 << std::endl ;
		bool found = false;
		for ( int i = 0;( i < length ) && ( i < length2 ) && found == false;i++ ) {
			if ( last < length2 &&graphArray[ last ][ i ] != NULL && graphArray[ last ][ i ][ 4 ] == 1.0 ) {
				if ( graphVisited[ i ] == true && corners == 0 ) {
					resultPoints[ resultCounter ][ 0 ] = graphArray[ last ][ i ][ 0 ];
					resultPoints[ resultCounter ][ 1 ] = graphArray[ last ][ i ][ 1 ];
					resultCounter++;
					return true;
				}
			}
			if ( last < length2 &&graphArray[ last ][ i ] != NULL ) {
				if ( graphVisited[ i ] == false && corners > 0 ) {
					//	cout <<  "recusiveGraph vist i case 2: "<< i << std::endl ;
					graphVisited[ i ] = true;
					found = recusivGraph( corners - 1, i, maximas );
				}

			}
			if ( found == true ) {
				// cout <<  "recusiveGraph vist i case 3: "<< i << std::endl ;
				resultPoints[ resultCounter ][ 0 ] = graphArray[ last ][ i ][ 0 ];
				resultPoints[ resultCounter ][ 1 ] = graphArray[ last ][ i ][ 1 ];
				resultCounter++;
				return found;
			}
		}
		return found;

	}
	/*
	Recursiv helper function
	@param corners seeked amount of corners
	@param maximas the total numbers of found linear systems
	*/
	bool ObjectScan::SolveGraph( int corners, int maximas ) {

		int length = maximas;
		//cout <<  "Length array: "<< length << std::endl ;
		bool found = false;
		for ( int i = 0;( i < length ) && found == false;i++ ) {

			// clear up the try always do this first
			for ( int m = 0; m < length; m++ )
			{
				//cout <<  "graphSolve vist i : "<< i << "visit m: "<< m << std::endl ;

				graphVisited[ m ] = false;
			}
			//cout <<  "graphSolve vist i : "<< i  << std::endl ;
			// we started with the node i
			graphVisited[ i ] = true;
			// seek graph with startpoint i
			found = recusivGraph( corners, i, maximas );
			// we found the graph jump out.
			if ( found == true ) {

				return found;
			}
		}

		return found;
	}
	/*
	Recursiv helper function
	@param Mat localimage
	@return array of the last found maxima point in the hough space

	*/
	int* ObjectScan::NextMaximum( Mat localimage ) {
		static int rvalue[ 2 ];
		int sizeX = localimage.cols;
		int sizeY = localimage.rows;
		int postions[ 300 ][ 2 ];
		int positioncounter = 0;
		int maxPosX = 0;
		int maxPosY = 0;
		int tempX = 0;
		int tempY = 0;
		localimage.at<uchar>( maxPosY, maxPosX ) = 0;
		for ( int x = 0; x < sizeX; x++ )
		{
			for ( int y = 0; y < sizeY; y++ )
			{

				if ( localimage.at<uchar>( maxPosY, maxPosX ) < localimage.at<uchar>( y, x ) )
				{

					maxPosX = x;
					maxPosY = y;
				}
			}
		}
		int br = localimage.at<uchar>( maxPosY, maxPosX );
		//cout <<  "Brightness intern : "<< br  << std::endl ;
		//cout <<  "Found x : "<< maxPosX << "\n" << std::endl ;
		//cout <<  "Found x : "<< maxPosY << "\n" << std::endl ;
		for ( int xPos = maxPosX - 5;xPos <= maxPosX + 5;xPos++ ) {
			for ( int yPos = maxPosY - 5;yPos <= maxPosY + 5;yPos++ ) {
				if ( xPos < sizeX && xPos >= 0 && yPos >= 0 && yPos < sizeY && abs( localimage.at<uchar>( yPos, xPos ) - localimage.at<uchar>( maxPosY, maxPosX ) ) < 10 )
				{
					postions[ positioncounter ][ 0 ] = xPos;
					postions[ positioncounter ][ 1 ] = yPos;
					positioncounter++;
				}

			}
		}
		if ( positioncounter > 0 ) {
			for ( int x = 0; x < positioncounter; x++ ) {
				tempX = postions[ x ][ 0 ] + tempX;
				tempY = postions[ x ][ 1 ] + tempY;
			}
			maxPosX = tempX / positioncounter;
			maxPosY = tempY / positioncounter;
		}
		rvalue[ 0 ] = maxPosX;
		rvalue[ 1 ] = maxPosY;

		return rvalue;
	}
	/*
	Recursiv helper function
	@param  numberOfMax seek the number of maxs in the hough space
	@param space the houghespace in which we seek
	@param image the image as refrence for the size
	*/
	Mat ObjectScan::FindAllMaximas( int numberOfMax, Mat space, Mat image ) {
		Mat local;
		int *max;
		int maxY, maxX;
		double theta;
		int r;
		int sizeX = image.cols;
		int sizeY = image.rows;
		int rMax = (int) ( sqrt( ( (double) sizeX / 2.0*(double) sizeX / 2.0 ) + ( (double) sizeY / 2.0*(double) sizeY / 2.0 ) ) );///2);

		local.create( space.size(), space.type() );
		local = Scalar::all( 0 );
		space.copyTo( local, space );

		int minbright = 300;
		for ( int i = 0;i < numberOfMax;i++ ) {
			max = NextMaximum( local );
			maxX = *( max );
			maxY = *( max + 1 );
			maxPosArray[ i ][ 0 ] = maxX;
			maxPosArray[ i ][ 1 ] = maxY;
			if ( local.at<uchar>( maxY, maxX ) < minbright ) {
				minbright = local.at<uchar>( maxY, maxX );
			}

			int k, l;
			for ( int xPos = maxX - 10;xPos <= maxX + 10;xPos++ ) {
				for ( int yPos = maxY - 10;yPos <= maxY + 10;yPos++ ) {
					if ( xPos < 0 ) {
						k = local.cols + xPos;
					}
					else
					{
						if ( xPos >= local.cols ) {
							k = xPos - ( local.cols );
						}
						else
						{
							k = xPos;
						}

					}
					if ( yPos < 0 ) {
						l = local.rows + yPos;
					}
					else
					{

						if ( yPos >= local.rows ) {
							l = yPos - ( local.rows );
						}
						else
						{
							l = yPos;
						}
					}

					local.at<uchar>( l, k ) = 0;

				}
			}
			theta = (double) maxX *( CV_PI / 180.0 );
			r = ( maxY - rMax )*-1;
			globalPositons[ i ][ 0 ] = theta;
			globalPositons[ i ][ 1 ] = r;
			Point a = Point( maxX - 10, maxY - 10 );
			Point b = Point( maxX + 10, maxY + 10 );
			rectangle( space, a, b, Scalar( 255, 0, 0 ), 0, 8 );


		}

		return local;
	}
	/*
	Count the numbers of points by giving a certain brightness
	@param  brightness the minmum brightness the last point must have
	@param space the houghspace
	*/
	int ObjectScan::CollectMaximas( int brightness, Mat space ) {
		Mat local;
		int *max;
		int maxY, maxX;

		//we need a copy or else we would destroy the orginal
		local.create( space.size(), space.type() );
		local = Scalar::all( 0 );
		space.copyTo( local, space );
		int minbright = 300;
		int count = 0;
		while ( minbright > brightness ) {
			//nextpoint
			max = NextMaximum( local );
			maxX = *( max );
			maxY = *( max + 1 );
			minbright = local.at<uchar>( maxY, maxX );
			int k, l;
			for ( int xPos = maxX - 10;xPos <= maxX + 10;xPos++ ) {
				for ( int yPos = maxY - 10;yPos <= maxY + 10;yPos++ ) {
					if ( xPos < 0 ) {
						k = local.cols + xPos;
					}
					else
					{
						if ( xPos >= local.cols ) {
							k = xPos - ( local.cols );
						}
						else
						{
							k = xPos;
						}

					}
					if ( yPos < 0 ) {
						l = local.rows + yPos;
					}
					else
					{

						if ( yPos >= local.rows ) {
							l = yPos - ( local.rows );
						}
						else
						{
							l = yPos;
						}
					}

					local.at<uchar>( l, k ) = 0;

				}
			}
			count = count + 1;
		}
		local.release();
		return count;
	}
	bool ObjectScan::scan( Mat source ,int cornersa, int pixel, int maximasa ,int upperlimita ) {
		int args = 0;
		int maximas = 0;
		int corners = 8;
		int pixeloffset = 2;
		int upperlimit = 75;
		int ratio = 3;
		int lowThreshold = 80;
		// debug false
		bool debug = false;
		Mat image, local;
		char* window_name = "Edge Map";


		image = source;
		pixeloffset = pixel;
		 corners = cornersa;

		if ( upperlimita != 0 ) {
			upperlimit = upperlimita;
		}

		if ( maximasa != 0 ) {
			maximas = maximasa;
		}

		if ( !image.data )                              // Check for invalid input
		{
			//cout << "Could not open or find the image" << std::endl;
			return false;
		}
		//create object for drawing lines;
		dst.create( image.size(), image.type() );
		dst = Scalar::all( 0 );
		
		//create image which contain only the corners in it
		Mat newimage = ObjectScan::CannyThreshold( image, ratio, lowThreshold );
		//create the houghespace of the image
		Mat hougespace = Hough( newimage );

		if ( maximas == 0 ) {
			if ( upperlimit != 0 ) {
				//count the numers of maximas with the help of brightness
				maximas = CollectMaximas( upperlimit, hougespace );
			}
			else
			{
				//default number of maximas for a stop shield
				maximas = 21;
			}
		}

		maxPosArray = new int*[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			maxPosArray[ i ] = new int[ 2 ];
		}

		globalPositons = new double *[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			globalPositons[ i ] = new double[ 2 ];
		}

		formelArray = new double *[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			formelArray[ i ] = new double[ 2 ];
		}
		formelFlip = new bool[ 10 ];

		formelborder = new int*[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			formelborder[ i ] = new int[ 4 ];
		}
		graphArray = new double **[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			graphArray[ i ] = new double *[ maximas ]();
			for ( int j = 0; j < maximas; ++j ) {
				graphArray[ i ][ j ] = new double[ 5 ];
				graphArray[ i ][ j ][ 4 ] = 0.0;
			}
		}
		resultPoints = new double *[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			resultPoints[ i ] = new double[ 2 ];
		}
		graphVisited = new bool[ maximas ]();

		// seek maximas
		local = FindAllMaximas( maximas, hougespace, image );
		// generate formel from maximas 
		Mat system = GenerateLinearSystem( maximas, newimage, image, 0, 0 );

		FindOverlayEdge( maximas, pixeloffset, newimage, 0, 0 );

		GenerateGraph( maximas, newimage );

		bool result = SolveGraph( corners, maximas );

		if ( debug ) {

			namedWindow( window_name, CV_WINDOW_AUTOSIZE );
			imshow( window_name, image );

			namedWindow( "Hough-Space", WINDOW_AUTOSIZE );// Create a window for display.
			imshow( "Hough-Space", hougespace );                   // Show our image inside it.

			namedWindow( "Picture", WINDOW_AUTOSIZE );// Create a window for display.
			imshow( "Picture", newimage );                   // Show our image inside it.

			namedWindow( "Lines", WINDOW_AUTOSIZE );// Create a window for display.
			imshow( "Lines", dst );

			namedWindow( "Local", WINDOW_AUTOSIZE );// Create a window for display.
			imshow( "Local", local );

			namedWindow( "System", WINDOW_AUTOSIZE );// Create a window for display.
			imshow( "System", system );
			waitKey( 0 );
		}


		//delete all objects
		image.release();
		hougespace.release();
		newimage.release();
		dst.release();
		local.release();
		system.release();

		delete[] maxPosArray;
		delete[] globalPositons;
		delete[] formelArray;
		delete[] formelFlip;
		delete[] formelborder;
		delete[] graphArray;
		delete[] resultPoints;
		delete[] graphVisited;

		return result;

	}
	bool ObjectScan::scanDebug( Mat source, int cornersa, int pixel, int maximasa, int upperlimita, int offsestx, int offsety, int debugbitmap )
	{
		int args = 0;
		int maximas = 0;
		int corners = 8;
		int pixeloffset = 2;
		int upperlimit = 75;
		int ratio = 3;
		int lowThreshold = 80;
		// debug false
		bool debug = true;
		Mat image, local;
		char* window_name = "Edge Map";


		image = source;
		pixeloffset = pixel;
		 corners = cornersa;
		if ( upperlimita != 0 ) {
			upperlimit = upperlimita;
		}

		if ( maximasa != 0 ) {
			maximas = maximasa;
		}

		if ( !image.data )                              // Check for invalid input
		{
			//cout << "Could not open or find the image" << std::endl;
			return false;
		}
		//create object for drawing lines;
		dst.create( image.size(), image.type() );
		dst = Scalar::all( 0 );

		//create image which contain only the corners in it
		Mat newimage = ObjectScan::CannyThreshold( image, ratio, lowThreshold );
		//create the houghespace of the image
		Mat hougespace = Hough( newimage );

		if ( maximas == 0 ) {
			if ( upperlimit != 0 ) {
				//count the numers of maximas with the help of brightness
				maximas = CollectMaximas( upperlimit, hougespace );
			}
			else
			{
				//default number of maximas for a stop shield
				maximas = 21;
			}
		}

		maxPosArray = new int*[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			maxPosArray[ i ] = new int[ 2 ];
		}

		globalPositons = new double *[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			globalPositons[ i ] = new double[ 2 ];
		}

		formelArray = new double *[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			formelArray[ i ] = new double[ 2 ];
		}
		formelFlip = new bool[ 10 ];

		formelborder = new int*[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			formelborder[ i ] = new int[ 4 ];
		}
		graphArray = new double **[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			graphArray[ i ] = new double *[ maximas ]();
			for ( int j = 0; j < maximas; ++j ) {
				graphArray[ i ][ j ] = new double[ 5 ];
				graphArray[ i ][ j ][ 4 ] = 0.0;
			}
		}
		resultPoints = new double *[ maximas ]();
		for ( int i = 0; i < maximas; ++i ) {
			resultPoints[ i ] = new double[ 2 ];
		}
		graphVisited = new bool[ maximas ]();

		// seek maximas
		local = FindAllMaximas( maximas, hougespace, image );
		// generate formel from maximas 
		Mat system = GenerateLinearSystem( maximas, newimage, image, 0, 0 );

		FindOverlayEdge( maximas, pixeloffset, newimage, 0, 0 );

		GenerateGraph( maximas, newimage );

		bool result = SolveGraph( corners, maximas );

		
		if ( debug ) {
			if ( debugbitmap & 0x0001 ) {
				namedWindow( window_name, CV_WINDOW_AUTOSIZE );
				imshow( window_name, image );
			}
			if ( debugbitmap & 0x0002 ) {
				namedWindow( "Hough-Space", WINDOW_AUTOSIZE );// Create a window for display.
				imshow( "Hough-Space", hougespace );                   // Show our image inside it.
			}
			if ( debugbitmap & 0x0004 ) {
				namedWindow( "Picture", WINDOW_AUTOSIZE );// Create a window for display.
				imshow( "Picture", newimage );                   // Show our image inside it.
			}
			if ( debugbitmap & 0x0008 ) {
				namedWindow( "Lines", WINDOW_AUTOSIZE );// Create a window for display.
				imshow( "Lines", dst );
			}
			if ( debugbitmap & 0x0010 ) {
				namedWindow( "Local", WINDOW_AUTOSIZE );// Create a window for display.
				imshow( "Local", local );
			}
			if ( debugbitmap & 0x0020 ) {
				namedWindow( "System", WINDOW_AUTOSIZE );// Create a window for display.
				imshow( "System", system );
			}
			waitKey( 0 );
		}


		//delete all objects
		image.release();
		hougespace.release();
		newimage.release();
		dst.release();
		local.release();
		system.release();

		delete[] maxPosArray;
		delete[] globalPositons;
		delete[] formelArray;
		delete[] formelFlip;
		delete[] formelborder;
		delete[] graphArray;
		delete[] resultPoints;
		delete[] graphVisited;

		return result;

	}



}


