//canon chdk raw analyser and data transformer
//based on dtr_newest.cpp

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <string>
#include <cmath>

using namespace std;
///////////////////////////////////////////////////////////////////////////////
//		CONSTANTS									///////////////////////////
///////////////////////////////////////////////////////////////////////////////
//Sensor dimensions for Canon a480, a490, a495 and a800 cameras;
//should be changed for other cameras
const int IMAGE_HEIGHT = 2772;
const int IMAGE_WIDTH  = 3720;


///////////////////////////////////////////////////////////////////////////////
//		TYPE DEFINITIONS							///////////////////////////
///////////////////////////////////////////////////////////////////////////////

typedef unsigned char BYTE;
typedef unsigned short int WORD;
typedef unsigned int DWORD;

///////////////////////////////////////////////////////////////////////////////
//		STRUCTURES IN USE							///////////////////////////
///////////////////////////////////////////////////////////////////////////////
struct CARR //char array
{
	string	name;		//text value for the name of array
	DWORD 	size;		//Total number of elements in array
	DWORD	height;		//number of rows
	DWORD	width;		//number of columns
	BYTE 	*p_Array;	//Here all the the data is contained
};

struct WARR //word array
{
	string	name;		//text value for the name of array
	DWORD 	size;		//Total number of elements in array
	DWORD	height;		//number of rows
	DWORD	width;		//number of columns
	WORD 	*p_Array;	//Here all the the data is contained
	bool 	isReflected;//indicates if image was mirrored -- affects color output
};

struct IARR //int array
{
	string	name;		//text value for the name of array
	DWORD 	size;		//Total number of elements in array
	int 	*p_Array;	//Here all the the data is contained
};

struct FARR //float array
{
	string	name;		//text value for the name of array
	DWORD 	size;		//Total number of elements in array
	float 	*p_Array;	//Here all the the data is contained
};

struct PIXC //pixel coordinate
{
	float x;
	float y;
};


//Following is to be defined further
struct SPCTR //spectral data 
{
	string	name;	//name of struct
	int		length;	//length of arrays 
	float	*p_channel_R;	//
	float	*p_channel_G1;
	float	*p_channel_G2;
	float	*p_channel_B;
	float	*p_channel_AVG; //average of four channels
};

///////////////////////////////////////////////////////////////////////////////
//		FUNCTION DECLARATIONS						///////////////////////////
///////////////////////////////////////////////////////////////////////////////



//data import
CARR ReadFileToCharArray ( string fileName );
WARR ReadBMP16FileToWordArray ( string fileName );
WARR InputDataFromFile ( string s_inputFile );

//data arrays manipulation
WARR ArrangeBytesToWords ( CARR charArray );
WARR QuarterSizeArray ( WARR inputArray, int width, int height );
void RemoveBadPixels ( WARR wordArray );

void FindHotPixels ( WARR wordArray, WORD threshold );

int FindMedianValue( int *p_values, int numValues );
int *FindMinMaxAvgMedValue ( int *p_Values, int num );

float FindMedianValue( float *p_values, int numValues );
float *FindMinMaxAvgMedValue ( float *p_Values, int num );

//data arrays to DIB arrays
int *CharArrayToDIB( CARR charArray, int height, int width, int bpp );
int *WordArrayToDIB( WARR wordArray, int bpp );
int *WordArrayToDIB8( WARR wordArray );


//interpolators
//assuming different pixels have data for different colors (Bayer array)
//interpolates between them and outputs full-color image
int *Dib8toColorDib32 ( int *p_DIb8Canvas, bool isReflected );

//focus function for import data
int *ReadCR2toDIB( string fileName );

//focus function for writing data as visually viewable file
void BMPWrite ( int *p_DibCanvas, int targetBpp, string s_fileName );
void OutputDataAsBMP ( WARR wordArray, string s_outputFile, string s_outputType );

//helpers
int *DibHeader ( int width, int height, int bpp, int Resolution );	
int *DibCanvasArray ( int *p_DIBHeader, int noPal );
void BMPInfo ( string s_fileName );

void Interpolate ( float *p_Array, int length, bool even );

int FindPeakDerivative ( float *p_array, int length );
float FindPeakCenterOfMass ( float *p_array, int length );

BYTE Shrink12To8Bit ( WORD val );
IARR FindPeaks ( int *data, int length );
//IARR FindPeaks ( float *data, int length );
FARR FindPeaks ( float *data, int length );

PIXC FindPeakCenterOfMass2D ( WARR wordArray, int peakStartX, int peakStartY, int peakWidth, int peakHeight );

int *BinnedValues ( WARR wordArray, int max, int numBins );

float *CalculateParabolicCoefficients ( float *p_coordinates, int length );

double Exp ( double n, int e );


//palletes
//---Palettes
	//--menu
int *PaletteChoice( int bpp, int choice ); //menu for choosing palettes

	//--multi bit depth palette generators
int *GrayPaletteGenerator ( int bpp );//for 1, 4 and 8 bpp
int *EmptyPalette( int bpp );//for 1, 4 and 8 bpp
int *CustomPalette( int bpp, int *p_PopularColors );
int *ManualPalette(int bpp); // for 1, 4 bpp
//int *PaletteTopo (int bpp );  //
	//-- 8-bit palettes
int *RRRGGGBBPalette ();
int *ZZRRBBGGPalette ();
int *gRRGGGBBPalette();
	//-- 4-bit palettes
int *RGGBPalette ();
int *RGGBmodPalette (); // two pink shades replaced by greys
int *gRGBPalette (); //8 colors + 8 greys

int *EmptyPalette( int bpp );

	//---Colors
int ColorLUT();
void ListColors ();

void ShowIntHex ( int value, int n );
//--- Matrix operations
void RowEchelonForm ( double* p_data, int rows, int columns );
void NormalizeRows ( double* p_data, int rows, int columns );
void SubtractRowDown ( double* p_data, int rows, int columns, int rowToSubtract );
void SubtractRowUp ( double* p_data, int rows, int columns, int rowToSubtract );

//-- program functionality
void Develop ( string *s_fileNames, int numFileNames, string s_outputType );
void DisplayHelp();
void ScalarMath( WARR wordArray, string s_mathOp, double value ); 
WARR MatrixOperation( WARR wordArray1, WARR wordArray2, string s_mathOp );
WARR MultiMatrixOperation( WARR *p_wordArrays, int numArrays, string s_mathOp ); 
void OutputSpectrumAsCSVdata ( WARR wordArray );
void ListPeaks ( WARR wordArray );
void GetStatistics( WARR wordArray );

////////////////////////////////////////////////////////////////////////////////
////	MAIN FUNCTION		////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main( int argc, char *argv[] )
{	
	/************************************************************
	*															*	
	*	What the program should do:								*
	*															*
	*	Program processes data from file(s) on disk				*
	*	and outputs a file(s) with results. 					*
	*	So, it needs name(s) of input and output files			*
	*															*
++	*	Mode 1: FastDevelop: take in cr2, hotpixels.txt			*
	*	(if exists), output bmp -- 3 types:	 					*
	*	  bmp8  -- b/w 8-bit log presentation of sensor values.	*
	*	  bmp16 -- highcolor bmp with full 12-bit sensor values.*
	*			   (false-color viewable image).				*
	*	  bmp32 -- truecolor log presentatiton of sensor values *
	*			   with width and height halved.				*
	*	-d	| -develop											*
	*															*
-	! Mode 2 is superfluous and does not comply to 1 input rule	!
-	*	Mode 2: Develop: take in cr2, hotpixels.txt(if exist),	*
-	*	take in "offset" bmp16, take in "dark" bmp16,			*
-	*	output bmp -- 3 types, same as in mode 1.				*
	*															*
-+	*	Mode 3: FullDevelop: Take in cr2 file and txt file with	*
	*	the list of dependency files ( "hotpixels.txt", "offset"*
	*	bmp16 file, "dark" bmp16 file, "flat" bmp16 file ),		*
	*	output bmp (3 types, same as in Mode 1 ).				*
	*	-d	| -develop											*
	*															*
++	*	Mode 4: FindHotPixels: take in cr2, threshold value, 	*
	*	output list of hot pixels in "hotpixels.txt" file.		*
	*	-hp | -hotpixels										*
	*															*
++-	*	Mode 5: Scalar operations: take in cr2 or bmp16, 		*
	*	number, and do arithmetics on each pixel, output bmp	*
	*	(3 types, same as in Mode 1 ).							*
	*		Operaions:											*
	*		- add same value to each pixel 						*
	*		- subtract same value from each pixel 				*
	*		- multiply each pixel by the same value				*
	*		- divide each pixel by the same value				*
	*		(- normalize - find max for colors and scale to 64k)*
	*	-sm | -scalarmath										*
	*															*
+	*	Mode 6: Matrix operations: take in two cr2, or two 		*
	*	bmp16, or cr2 and bmp16 files and do arithmetics between*
	*	them, pixel by pixel, output bmp (3 types, same as in	*
	*	Mode 1 ).												*
	*		Operaions:											*
	*		- add two matrices									*
	*		- subtract second matrix from first					*
	*		- multiply matrix by matrix							*
	*		- divide first matrix by second (second one will be	*
	*		treated as having fixed-point values in range 0-1	*
	*		of short integers raanging from 0 to 65535)			*
	*	-mop | -matrixoperation									*
	*															*
-	*	Mode 7: Matrix operations2: take in the list of files	*
	*	to process(cr2 or bmp16), import data from files in the	*
	*	list, and do aritmetics using all of them, output bmp	*
	*	(3 types, same as in Mode 1 ).							*
	*		Operaions:											*
	*		- Find mean value between all files in list			*
	*		- Find median value between all files in list		*
	*		- Sum values between all files in list				*
	*	-mop2 | -matrixoperation2								*
	*															*
	*	Mode 8: ImageStatistics: take in bmp16, output various 	*
	*	statistical data to console or csv file:				*
	*		- min and max values								*
	*		- histogram											*
	*		- spectral peaks (?)								*
	*															*
	************************************************************/
	/*
	TO DO
	
	1 +++	Simple file import, byte by byte, into char array 	
	2 +++	Bit/byte manipulation, with conversion to WORD or int array (i.e. for unpacking framebuffer data)
	3 +++	casting the array into DibCanvas array
	4 +++	writing bmp image to Disk --- function already exists
	5 +++	Import several cr2 files, and do math on them
	6 +++	Make argument parser
	
	Primary goal is framebuffer data manipulation, but can also be used for creating visualization of any file
	
	*/
	
	string s_inputFile = "";
	string s_outputFile = "";
	string s_outputType = "";
	string *s_fileNames = NULL;
	string s_command = "";
	string s_mathOp = "";
	string *parameters = new string [argc-1];
	double numericValue = 0;
	int length = 0;
	int numFileNames = 0;
	
	if ( argc > 1 )
	{
		for ( int i = 1; i < argc; i++ )
		{
			stringstream ss_convert(argv[i]);//converting char** to string
			ss_convert >> parameters[i-1];		
		}
	}
	
	for ( int i = 0; i < argc-1; i++ )
	{
		if ( parameters[i][0] == '-' )
		{
			// command
			if ( s_command == "" )
			{
				s_command = parameters[i];
			}
			else
			{
				//only first command is parsed, all others are ignored
			}
		}
		else
		{
			length = parameters[i].length();
			if ( parameters[i][length-4] == '.' )
			{
				// counting file names provided
				numFileNames++;
				
			}
			else
			{	//if s_outputType has not been fetched, and found argument is one of the list:
				if ( s_outputType == "" && (parameters[i] == "mono" || parameters[i] == "color" || parameters[i] == "data" || 
											parameters[i] == "avgdata" || parameters[i] == "avgmono") )
				{
					s_outputType = parameters[i];
				}
				else
				{
					//command parameter or ignored value
				}
				
				if ( numericValue == 0 && (parameters[i][0] > char(48) && parameters[i][0] < char(58)) )
				{	//if first char is '1' to '9',
					//treat as number
					stringstream ss_convert(parameters[i]);//converting string to int
					ss_convert >> numericValue;	
				}
				
				if ( (s_command == "-sm" || s_command == "-scalarmath" || 
					  s_command == "-mop" || s_command == "-matrixoperation" ) && 	
					  s_mathOp == "" && //this way only the first match gets picked
					( parameters[i] == "add" || parameters[i] == "sub" || 
					  parameters[i] == "mul" || parameters[i] == "div") )
				{
					s_mathOp = parameters[i];
				}
				else if ( (	s_command == "-mop2" || "-matrixoperation2" ) &&
							s_mathOp == "" &&
						  ( parameters[i] == "sum" || parameters[i] == "mean" || parameters[i] == "median" ) )
				{
					s_mathOp = parameters[i];
				}
			}
		}
	}
	
	//grabbing all the filenames
	if ( numFileNames == 0 ) //no filenames provided
	{
		numFileNames = 2;
		s_fileNames = new string [2];
		s_fileNames[0] = "";
		s_fileNames[1] = "";
	}
	else if ( numFileNames == 1 ) //only one filename provided -- assuming inputfilename
	{
		numFileNames = 2;
		s_fileNames = new string [2];
		
		for ( int i = 0; i < argc-1; i++ )
		{
			length = parameters[i].length();
			if ( parameters[i][length-4] == '.' )
			{
				s_fileNames[0] = parameters[i];
			}
		}
		s_fileNames[1] = "";
	}
	else	//2 or more filenames are provided; 
	{
		s_fileNames = new string [numFileNames];
		int index = 0;
		for ( int i = 0; i < argc-1; i++ )
		{
			length = parameters[i].length();
			if ( parameters[i][length-4] == '.' )
			{
				s_fileNames[index] = parameters[i];
				index++;
			}
		}
	}
	s_inputFile = s_fileNames[0];
	s_outputFile = s_fileNames[numFileNames-1];
	
	
	
	
	if ( s_command == "-h" || s_command == "-help" )
	{
		DisplayHelp();
	}
	else if ( s_command == "-d" || s_command == "-develop" )
	{
		Develop ( s_fileNames, numFileNames, s_outputType );
	}
	else if ( s_command == "-hp" || s_command == "-hotpixels" )
	{
		WORD threshold = numericValue;
		while ( threshold == 0 || threshold > 4095 )
		{
			cout << "\nPlease enter threshold value for hot pixel\n"
				 << "in tange between 1 to 4095: ";
			cin >> threshold;
		}
		WARR wordArray = InputDataFromFile ( s_inputFile );
		FindHotPixels ( wordArray, threshold );
	}
	else if ( s_command == "-sm" || s_command == "-scalarmath" )
	{
		while ( s_mathOp == "" || ( s_mathOp != "add" && s_mathOp != "sub" && s_mathOp != "mul" && s_mathOp != "div" ) )
		{
			cout << "\nEnter operation to apply (add/sub/mul/div): ";
			cin >> s_mathOp;
		}
		WARR wordArray = InputDataFromFile ( s_inputFile );
		//core function - void, modifies wordArray passed to it
		ScalarMath( wordArray, s_mathOp, numericValue ); 
		
		OutputDataAsBMP ( wordArray, s_outputFile, s_outputType );	
	}
	else if ( s_command == "-mop" || s_command == "-matrixoperation" )
	{
		while ( s_mathOp == "" || ( s_mathOp != "add" && s_mathOp != "sub" && s_mathOp != "mul" && s_mathOp != "div" ) )
		{
			cout << "\nEnter operation to apply (add/sub/mul/div): ";
			cin >> s_mathOp;
		}
		//assuming two input files are provided
		WARR wordArray1 = InputDataFromFile ( s_fileNames[0] );
		WARR wordArray2;
		if ( numFileNames-1 > 1 )
		{
			wordArray2 = InputDataFromFile ( s_fileNames[1] );
		}
		else
		{
			cout << "\nInput the name of file to do " << s_mathOp << " operation with " << s_fileNames[0] << "\n";
			string s_secondInputFile = "";
			cin >> s_secondInputFile;
			wordArray2 = InputDataFromFile ( s_secondInputFile );
		}
		//core function - void, modifies wordArray passed to it
		WARR resultWA = MatrixOperation( wordArray1, wordArray2, s_mathOp );
		
		OutputDataAsBMP ( resultWA, s_outputFile, s_outputType );	
	}
	else if ( s_command == "-mop2" || s_command == "-matrixoperation2" )
	{
		while ( s_mathOp == "" || ( s_mathOp != "sum" && s_mathOp != "mean" && s_mathOp != "median" ) )
		{
			cout << "\nEnter operation to apply (sum/mean/median): ";
			cin >> s_mathOp;
		}
		
		cout << "\nTotal " << numFileNames-1 << "Input files:";
		for ( int i = 0; i < numFileNames-1; i++ )
		{
			cout << "\nInput file #" << i+1 << " -- " << s_fileNames[i];
		}
		
		WARR *p_wordArrays = new WARR [ numFileNames-1 ]; //all except the last filenames are assumed to be input files
		for ( int i = 0; i < numFileNames-1; i++ )
		{
			p_wordArrays[i] = InputDataFromFile ( s_fileNames[i] );
		}
		//core function - void, modifies wordArray passed to it
		WARR resultWA = MultiMatrixOperation( p_wordArrays, numFileNames-1, s_mathOp ); 
		
		OutputDataAsBMP ( resultWA, s_outputFile, s_outputType );	
	}
	else if ( s_command == "-lp" || s_command == "-listpeaks" )
	{
		cout << "\nUnder construction.\nShould output list of peak pixel values along X axis -- applies only to linear spectra.\n";
		cout << "-lp input.cr2/.bmp startline output.bmp\n";
		WARR wordArray = InputDataFromFile ( s_inputFile );
		ListPeaks ( wordArray );
		if ( s_outputFile != "" )
		{
			OutputDataAsBMP ( wordArray, s_outputFile, s_outputType );	
		}
	}
	else if ( s_command == "-so" || s_command == "-spectrumoutput" )
	{
		cout << "\nUnder construction.\nShould output csv file with pixel values vs X coordinate.\n";
		cout << "32 central rows are summed, 4 columns are produced, for R, G1, G2 and B channels.\n";
		cout << "Value for each channel is the sum of column of 16 values.\n";
		cout << "Expected input are .cr2 and data .bmp files with values not bigger than 4095.\n";
		cout << "This way output will lie in 0-65535 range.\n";
		cout << "Alternative modes could use mean instead of sum.\n";
		cout << "Yet other parameter could change the width of sampling,\n"
			<< "or output data for whole image by 32-px strips.\n";
		
		WARR wordArray = InputDataFromFile ( s_inputFile );
		OutputSpectrumAsCSVdata ( wordArray );
		if ( s_outputFile != "" )
		{
			OutputDataAsBMP ( wordArray, s_outputFile, s_outputType );	
		}
	}
	else if ( s_command == "-bmpinfo" )
	{
		BMPInfo( s_inputFile );
	}
	else if ( s_command == "-stat" )
	{
		cout << "\nStatistics regarding imported data:\n";
		WARR wordArray = InputDataFromFile ( s_inputFile );
		GetStatistics( wordArray );
	}
	else
	{
		cout << "use -h or -help switch to read use info.\n";
	}
	
	return 0;
}


///////////////////////////////////////////////////////////////////////////////
void DisplayHelp()
{
//	cout << "-h | -help:	this help\n"; 
	
	cout << "\nInput and output filenames are with extesions (.cr2/.bmp for input and .bmp for output) by those extensions they are recognozed as filenames\n";
	
	cout << "\nProgram usage:\n";
	
	cout << "\nThis program is meant for work with data in .cr2 files\n"
		<< "created in Canon PowerShot cameras via CHDK addon.\n"
		<< "Program is hardcoded for cameras with 2772x3270px sensors (a480, a490,\na495 and a800),"
		<< "(-TBD- can be overriden using txt file with alternative values).\n";
		
	cout << "\nFor using this program, its name along with desired operation, their\n"
		<< "parameters and input/output filenames should be entered in command prompt.\n";
	
	cout << "\nCommands ( -short | -longCommandName ):\n\n";
	
	cout << " -h | -help\tDisplay this help.\n";
	
	cout << " -d | -develop\tProcess .cr2 files to yield viewable .bmp images.\n";
	cout << "\t\t-d source.cr2 [offset.bmp] [dark.bmp] [flat.bmp] result.bmp data/mono/color/avgdata/avgmono\n";
	
	cout << "-hp | -hotpixels\tProduces txt file with the list of pixels with values higher than threshold.\n";
	cout << "\t\t-hp source.cr2/.bmp <thersholdValue>\n";
	
	cout << "-sm | -scalarmath\tchanging values by some constant.\n";
	cout << "\t\t-sm source.cr2/.bmp add/sub/mul/div <scalarValue> result.bmp data/mono/color\n";
	
	cout << "-mop | -matrixoperation\tcombination of two source images.\n";
	cout << "\t\t-mop source1.cr2/.bmp add/sub/mul/div source2.cr2/.bmp result.bmp data/mono/color\n";
	
	cout << "-mop2 | -matrixoperation2\tcombination of arbitrary number of source images.\n";
	cout << "\t\t-mop2 source1.cr2/.bmp [source2.cr2/.bmp ...] sum/mean/median  result.bmp data/mono/color\n";
	
	cout << "-so input.cr2/.bmp [output.bmp]\n";
	cout << "-lp input.cr2/.bmp [output.bmp]\n";
	cout << "-stat input.cr2/.bmp\n";
}

///////////////////////////////////////////////////////////////////////////////

void Develop ( string *s_fileNames, int numFileNames, string s_outputType )
{
	//development has several stages, one of which are optional
	//files used in consecutive stages should be referenced in correct sequence
	/**
	Stages
		
		1 - input of file for development -- mandatory
		2 - subtraction of offset -- optional, always after stage 1
		3 - subtraction of dark -- optional, always after stage 2
		4 - division by flat -- optional, always after stage 3		
		5 - output of developed file -- mandatory
		
	**/
	// -d input [offset] [dark] [flat] ... output outputType
	// above is a typical command; 
	// output filename is always the last one in sequence;
	// any filenames between flatfield correction image name and the output image name are ignored
	
	
	WARR developedArray = InputDataFromFile ( s_fileNames[0] );
	WARR offset, dark, flat;
	cout << "\n-1-original: " << s_fileNames[0] << "\n";
	if ( numFileNames > 2 ) // offset is given
	{
		cout << "\n-2-offset: " << s_fileNames[1] << "\n";
		offset = InputDataFromFile ( s_fileNames[1] );
		developedArray = MatrixOperation( developedArray, offset, "sub" );
	}
	
	if ( numFileNames > 3 ) //dark is also given
	{
		cout << "\n-3-dark: " << s_fileNames[2] << "\n";
		dark = InputDataFromFile ( s_fileNames[2] );
		developedArray = MatrixOperation( developedArray, dark, "sub" );
	}
	if ( numFileNames > 4 ) //flat is also given
	{
		cout << "\n-4-flat" << s_fileNames[3] << "\n";
		flat = InputDataFromFile ( s_fileNames[3] );
		developedArray = MatrixOperation( developedArray, flat, "div" );
	}
	
	cout << "\n-5-output: " << s_fileNames[numFileNames-1] << "\n";
	OutputDataAsBMP ( developedArray, s_fileNames[numFileNames-1], s_outputType );	
}

void ScalarMath( WARR wordArray, string s_mathOp, double value )
{
	int size = wordArray.size;
	int result = 0;
	//modify wordArray
	for ( int i = 0; i < size; i++ )
	{
		if ( s_mathOp == "add" )
		{
			result = wordArray.p_Array[i] + value;
		}
		else if ( s_mathOp == "sub" )
		{
			result = wordArray.p_Array[i] = wordArray.p_Array[i] - value;
		}
		else if ( s_mathOp == "mul" )
		{
			result = wordArray.p_Array[i] * value;
		}
		else if ( s_mathOp == "div" )
		{
			result = wordArray.p_Array[i] / value;
		}
		
		//clipping result to range 0-65535
		if ( result < 0 )
		{
			result = 0;
		}
		else if ( result > 0xffff )
		{
			result = 0xffff; //clipping at 65535
		}
		
		
		wordArray.p_Array[i] = result;
	}
}

///////////////////////////////////////////////////////////////////////////////
WARR MatrixOperation( WARR wordArray1, WARR wordArray2, string s_mathOp )
{
	int size = wordArray1.size; //assuming all arrays are the same size
	
	WARR resultWA;
	resultWA.name = "MOpResultArray";
	resultWA.size = size;
	resultWA.width = wordArray1.width;
	resultWA.height = wordArray1.height;
	resultWA.p_Array = new WORD [size];
	int result = 0; 
	double divizor;
	
	if ( wordArray1.size == wordArray2.size )
	{
		for ( int i = 0; i < size; i++ )
		{
			if ( s_mathOp == "add" )
			{
				result = wordArray1.p_Array[i] + wordArray2.p_Array[i];
			}
			else if ( s_mathOp == "sub" )
			{
				result = wordArray1.p_Array[i] - wordArray2.p_Array[i];
			}
			else if ( s_mathOp == "mul" )
			{
				result = wordArray1.p_Array[i] * wordArray2.p_Array[i];
			}
			else if ( s_mathOp == "div" )
			{ //assumes that wordArray2 has values from 0 to 1-1/65535 
				divizor = double(wordArray2.p_Array[i])/65536.0;
				result = wordArray1.p_Array[i] / divizor;
				
			}
			
			//clipping result to range 0-65535
			if ( result < 0 )
			{
				result = 0;
			}
			else if ( result > 0xffff )
			{
				result = 0xffff; //clipping at 65535
			}
			
			resultWA.p_Array[i] = result;
		}
		return resultWA;
	}
	else
	{
		cout << "\n!!!MatrixOperation ERROR!!!\nArrays of different sizes!";
		cout << "\n Returning unmodified first array as result.\n";
		return wordArray1;
	}
	
}

///////////////////////////////////////////////////////////////////////////////
WARR MultiMatrixOperation( WARR *p_wordArrays, int numArrays, string s_mathOp )
{
	int size = p_wordArrays[0].size;
	int computedValue = 0;
	int *p_values = new int [numArrays];
	
	WARR resultWA;
	resultWA.name = "MOp2ResultArray";
	resultWA.size = size;
	resultWA.width = p_wordArrays[0].width;
	resultWA.height = p_wordArrays[0].height;
	resultWA.p_Array = new WORD [size];
	
	bool error = false;
	for ( int i = 0; i < numArrays; i++ )
	{
		if ( int(p_wordArrays[i].size) != size )
		{
			error = true;
			break;
		}
	}
	
	if ( error )
	{
		cout << "\n!!!MultiMatrixOperation ERROR!!!\nArrays of different sizes!";
		cout << "\n Returning unmodified first array as result.\n";
		return p_wordArrays[0];
	}
	else
	{
		for ( int i = 0; i < size; i++ )
		{
			if ( s_mathOp == "sum" )
			{
				computedValue = 0;
				for ( int j = 0; j < numArrays; j++ )
				{
					computedValue += p_wordArrays[j].p_Array[i];
				}			
				resultWA.p_Array[i] = computedValue;
			}
			else if ( s_mathOp == "mean" )
			{
				computedValue = 0;
				for ( int j = 0; j < numArrays; j++ )
				{
					computedValue += p_wordArrays[j].p_Array[i];
				}			
				resultWA.p_Array[i] = computedValue / numArrays;
			}
			else if ( s_mathOp == "median" )
			{
				for ( int j = 0; j < numArrays; j++ )
				{
					p_values[j] = p_wordArrays[j].p_Array[i];
				}			
				resultWA.p_Array[i] = FindMedianValue( p_values, numArrays );
			}
		}	
		return resultWA;
	}
}



///////////////////////////////////////////////////////////////////////////////
WARR InputDataFromFile ( string s_inputFile )
{
	int filenameLength = s_inputFile.length();
	CARR ca_file;
	ca_file.name = s_inputFile;
	WARR wa_file;
	wa_file.name = s_inputFile;
	
	if ( s_inputFile[filenameLength-3] == 'c' && s_inputFile[filenameLength-2] == 'r' && s_inputFile[filenameLength-1] == '2' )
	{
		ca_file = ReadFileToCharArray ( s_inputFile );
		wa_file = ArrangeBytesToWords ( ca_file );
		RemoveBadPixels ( wa_file );
	}
	else if ( s_inputFile[filenameLength-3] == 'b' && s_inputFile[filenameLength-2] == 'm' && s_inputFile[filenameLength-1] == 'p' )
	{
		//call bmp to WARR function
		wa_file = ReadBMP16FileToWordArray( s_inputFile );
	}
	else
	{
		//wrong file
		cout << "\nInputDataFromFile ERROR!!! Wrong file type!\n";
		wa_file.size = 0;
		wa_file.name = "";
		wa_file.p_Array = NULL;
	}
	
	return  wa_file;
}

///////////////////////////////////////////////////////////////////////////////
void OutputDataAsBMP ( WARR wordArray, string s_outputFile, string s_outputType )
{
	int *p_DibCanvas = NULL;
	WARR halfWA;
	
	if ( s_outputType == "data" )
	{
		cout << "\n--1-output as 'data'\n";
		p_DibCanvas = WordArrayToDIB( wordArray, 16 );		
	}
	else if ( s_outputType == "avgdata" )
	{
		//output half size image with each pixel corresponding to rggb quad mean value
		cout << "\n--2-output as 'avgdata'\n";
		halfWA = QuarterSizeArray ( wordArray, IMAGE_WIDTH, IMAGE_HEIGHT );
		p_DibCanvas = WordArrayToDIB( halfWA, 16 );		
	}
	else if ( s_outputType == "avgmono" )
	{
		//output half size 8-bit viewable image
		cout << "\n--3-output as 'avgmono'\n";
		halfWA = QuarterSizeArray ( wordArray, IMAGE_WIDTH, IMAGE_HEIGHT );
		p_DibCanvas = WordArrayToDIB8( halfWA );
	}
	else if ( s_outputType == "mono" )
	{
		cout << "\n--4-output as 'mono'\n";
		p_DibCanvas = WordArrayToDIB8( wordArray );
	}
	else if ( s_outputType == "color" )
	{
		cout << "\n--5-output as 'color'\n";
		int *p_Dib8Canvas = WordArrayToDIB8( wordArray );
		p_DibCanvas = Dib8toColorDib32 ( p_Dib8Canvas, wordArray.isReflected );
		delete[] p_Dib8Canvas;
		p_Dib8Canvas = NULL;
	}
	else //"data" is default
	{
		cout << "\n--6-output as 'data'\n";
		p_DibCanvas = WordArrayToDIB( wordArray, 16 );		
	}
	
	BMPWrite( p_DibCanvas, (p_DibCanvas[3] >> 16), s_outputFile );

	delete[] p_DibCanvas;
	p_DibCanvas = NULL;
	
}

WARR QuarterSizeArray ( WARR inputArray, int width, int height )
{
	int size = inputArray.size;
	int RGGBaverage = 0;
	
	if ( size != width * height )
	{
		cout << "\n!!!ERROR!!!\nQuarterSizeArray: width and height are incorrect!\n";
		cout << "\nReturning initial array without modification.\n";
		return inputArray;
	}
	
	WARR outputArray;
	outputArray.name = "QuarterSizeArray";
	outputArray.size = size / 4;
	outputArray.p_Array = new WORD [size/4];
	
	for ( int j = 0; j < height/2; j++ )
	{
		for ( int i = 0; i < width/2; i++ )
		{
			RGGBaverage = ( inputArray.p_Array[(2*j+0)*width + 2*i + 0] +
							inputArray.p_Array[(2*j+0)*width + 2*i + 1] +
							inputArray.p_Array[(2*j+1)*width + 2*i + 0] +
							inputArray.p_Array[(2*j+1)*width + 2*i + 1] ) / 4;
			
			
			outputArray.p_Array[j*width/2 + i] = RGGBaverage;
		}
	}
	
	return outputArray;
}

void ListPeaks ( WARR wordArray )
{
	
	int startLine = 0;
//	int spectrumLength = IMAGE_WIDTH; //-- values will be interpolated
	float *R = new float [IMAGE_WIDTH];
	float *G1 = new float [IMAGE_WIDTH];
	float *G2 = new float [IMAGE_WIDTH];
	float *B = new float [IMAGE_WIDTH];
	float *AVG = new float [IMAGE_WIDTH];
	float *colorValues = new float [4];
	float *MinMaxAvgMed = NULL;
	FARR *peaks = new FARR [3];
	
	string s_yes = "y";
	
	for ( int k = 0; k < 3; k++ )
	{
		startLine = IMAGE_HEIGHT * 0.25 * (k+1) - 16;
		for ( int i = 0; i < IMAGE_WIDTH; i+=2 )
		{
			R[i] = 0;
			G1[i] = 0;
			G2[i] = 0;
			B[i] = 0;
			for ( int j = 0; j < 32; j+=2 )
			{
				if ( wordArray.isReflected )
				{
					R[i+1]  += wordArray.p_Array[(startLine + j + 0)*IMAGE_WIDTH + i + 1];
					G1[i+0] += wordArray.p_Array[(startLine + j + 0)*IMAGE_WIDTH + i + 0];
					G2[i+1] += wordArray.p_Array[(startLine + j + 1)*IMAGE_WIDTH + i + 1];
					B[i+0]  += wordArray.p_Array[(startLine + j + 1)*IMAGE_WIDTH + i + 0];
				}
				else
				{
					R[i+0]  += wordArray.p_Array[(startLine + j + 0)*IMAGE_WIDTH + i + 0];
					G1[i+1] += wordArray.p_Array[(startLine + j + 0)*IMAGE_WIDTH + i + 1];
					G2[i+0] += wordArray.p_Array[(startLine + j + 1)*IMAGE_WIDTH + i + 0];
					B[i+1]  += wordArray.p_Array[(startLine + j + 1)*IMAGE_WIDTH + i + 1];
				}
			}
		}
		
		if ( wordArray.isReflected )
		{
		//	Interpolate ( float *p_Array, int length, bool even )
			Interpolate ( R, IMAGE_WIDTH, false );
			Interpolate ( G1, IMAGE_WIDTH, true );
			Interpolate ( G2, IMAGE_WIDTH, false );
			Interpolate ( B, IMAGE_WIDTH, true );
		}
		else
		{
			Interpolate ( R, IMAGE_WIDTH, true );
			Interpolate ( G1, IMAGE_WIDTH, false );
			Interpolate ( G2, IMAGE_WIDTH, true );
			Interpolate ( B, IMAGE_WIDTH, false );
		}
		
		
		for ( int i = 0; i < IMAGE_WIDTH; i++ )
		{
			colorValues[0] = R[i];
			colorValues[1] = G1[i];
			colorValues[2] = G2[i];
			colorValues[3] = B[i];
			MinMaxAvgMed = FindMinMaxAvgMedValue( colorValues, 4 );
			AVG[i] = MinMaxAvgMed[2];
		}
		
		peaks[k] = FindPeaks( AVG, IMAGE_WIDTH ); //peaks vill only contain indices
		
	}
	
	cout << "\nShowing peak positions on 3 lines of spectrum image:\n";
	cout << "\n\th=0.25\th=0.5\th=0.75\n\n";
	WORD listPeaksLenth = peaks[0].size;
	WORD listPeaksLengthMin = peaks[0].size;
	for ( int i = 1; i < 3; i++ )
	{
		if ( listPeaksLenth < peaks[i].size )
		{
			listPeaksLenth = peaks[i].size;
		}
		if ( listPeaksLengthMin > peaks[i].size )
		{
			listPeaksLengthMin = peaks[i].size;
		}
	}
	
	int row = 0, column = 0;
	while ( s_yes == "y" || s_yes == "yes" )
	{
		for ( DWORD i = 0; i < listPeaksLenth; i++ )
		{
			cout << "# " << i+1 << "\t";
			for ( int k = 0; k < 3; k++ )
			{
				if ( i < peaks[k].size )
				{
					cout << peaks[k].p_Array[i] << "\t";
				}
				else
				{
					cout << "......\t";
				}
			}
			cout << "\n";
		}
		
		cout << "\nRemove value?(y/n): ";
		cin >> s_yes;
		if ( s_yes == "y" || s_yes == "yes" )
		{
			cout << "Which column? (1/2/3): ";
			cin >> column;
			cout << "Which row? (1-" << peaks[column-1].size << "): ";
			cin >> row;
			for ( WORD i = row-1; i < peaks[column-1].size-1; i++ )
			{
				peaks[column-1].p_Array[i] = peaks[column-1].p_Array[i+1];
			}
			
			peaks[column-1].p_Array[peaks[column-1].size-1] = 0;
		}		
	}
	
	float *p_coordinates = new float [ (listPeaksLengthMin+1)*3 ];
	p_coordinates[0] = IMAGE_HEIGHT * 0.25;
	p_coordinates[1] = IMAGE_HEIGHT * 0.5;
	p_coordinates[2] = IMAGE_HEIGHT * 0.75;
	for ( int j = 1; j <= listPeaksLengthMin; j++ )
	{
		for ( int i = 0; i < 3; i++ )
		{
			p_coordinates[j*3+i] = peaks[i].p_Array[j-1];
		}
	}
	
	
	
	float *p_coefficients = CalculateParabolicCoefficients ( p_coordinates, (listPeaksLengthMin+1)*3 );
	cout << "\nValues and parabolic coefficients:\n";
	
	for ( int i = 0; i < listPeaksLengthMin; i++ )
	{
		cout << i+1 << "\t"	<< p_coefficients[i*4 + 0] << "\t"
							<< p_coefficients[i*4 + 1] << "\t"
							<< p_coefficients[i*4 + 2] << "\t"
							<< p_coefficients[i*4 + 3] << "\n";
	}
	
	
// Adding scanlines to image and peak ticks	
	for ( int k = 0; k < 3; k++ )
	{
		int peakIndex = 0;
		bool isPeak = false;
		startLine = IMAGE_HEIGHT * 0.25 * (k+1) - 16;
		for ( int i = 0; i < IMAGE_WIDTH; i++ )
		{
			if ( i == int(peaks[k].p_Array[peakIndex]) )
			{
				if ( isPeak )
				{
					//nothing
				}
				else
				{
					isPeak = true;
				}
			}
			else
			{
				if ( isPeak )
				{
					isPeak = false;
					peakIndex++;
				}
			}
			
			for ( int j = startLine; j < startLine + 32; j++ )
			{
				wordArray.p_Array[j*IMAGE_WIDTH+i] += 32;
				if ( j >= startLine + 14 && j <= startLine + 17 )
				{
					wordArray.p_Array[j*IMAGE_WIDTH+i] = 0x01ff;
				}
				if ( j >= startLine + 10 && j <= startLine + 21 )
				{
					if ( isPeak )
					{
						wordArray.p_Array[j*IMAGE_WIDTH+i] = 0x0fff;
					}
				}
			}	
		}
	}
	
//	trying to warp the image according to coefficients	
	int indexForChoosingCoefficients = 0;
	float fcC;
	int icC;
	float interpolatedValue;
	float diff1, diff2;
	WORD *warped_Array = new WORD [IMAGE_WIDTH*IMAGE_HEIGHT];
	cout << "\nChoose the coefficients set above (1-" << listPeaksLengthMin << "): ";
	cin >> indexForChoosingCoefficients;
	indexForChoosingCoefficients--;
	
	for ( int j = 0; j < IMAGE_HEIGHT; j++ )
	{
		for ( int i = 0; i < IMAGE_WIDTH; i++ )
		{
			fcC = p_coefficients[indexForChoosingCoefficients*4 + 1]*j*j
				+ p_coefficients[indexForChoosingCoefficients*4 + 2]*j
				+ p_coefficients[indexForChoosingCoefficients*4 + 3] + i;
									
			if ( fcC >= 0 && fcC < (IMAGE_WIDTH - 2) )
			{
				icC = (int(fcC)); //
				
				if ( i & 0x1 ) //if odd
				{	//make icC odd by ensuring lsb = 1
					if ( (icC & 0x1) == 0 )
					{
						icC++;
						fcC++; //float number should also be incremented so as difference weren't negative
					}				
				}
				else	// if even
				{	//make icC even by ensuring lsb = 0
					icC = icC & 0xfffffffe;
				}
			
				diff1 = (fcC-float(icC))/2;
				diff2 = 1 - diff1;
				
				interpolatedValue = float(wordArray.p_Array[j*IMAGE_WIDTH + icC + 2]) * diff1 + 
									float(wordArray.p_Array[j*IMAGE_WIDTH + icC + 0]) * diff2;
				
				if ( WORD(interpolatedValue) >= 4095 )
				{
					cout << "Strange values:\n";
					cout << "\ni = " << i;
					cout << "\nfcC = " << fcC;
					cout << "\nicC = " << icC;
					cout << "\ndiff1 = " << diff1;
					cout << "\ndiff2 = " << diff2;
					cout << "\nVal at icC+0: " << wordArray.p_Array[j*IMAGE_WIDTH + icC + 0]; 
					cout << "\nVal at icC+2: " << wordArray.p_Array[j*IMAGE_WIDTH + icC + 2];
					cout << "\ninterpolatedValue: " << interpolatedValue << "\n";
					break;
				}
								
				warped_Array[j*IMAGE_WIDTH + i + 0] = WORD(interpolatedValue);
			}
			else
			{
				warped_Array[j*IMAGE_WIDTH + i + 0] = 0;
			}
			
		
			
		}
		
	}
	
	for ( int i = 0; i < IMAGE_WIDTH*IMAGE_HEIGHT; i++ )
	{
		wordArray.p_Array[i] = warped_Array[i];
	}
	
	delete[] warped_Array;
	warped_Array = NULL;
}

///////////////////////////////////////////////////////////////////////////////
float *PolynomialFit ( PIXC *p_coordinates, int length )
{
//	given number of coordinate pairs, output list of coefficients corresponding to coord pair number
	//length should not be less than 2 (?)
	int matrixSize = (length+1)*length;
//	int order = length - 1;
	
	
	float *p_coefficients = new float [length]; //number of coefficients equals number of coordinate pairs given
	
	for ( int i = 0; i < length; i++ )
	{
		p_coefficients[i] = 0;
	}
	
	//create matrix to be processed to Row Echelon Form
	//length+1 by length
	double *p_matrix = new double [matrixSize];
	/*
	
	Matrix to solve:
	
	1		X1		X1^2	Y1			1	0	0	C
	1		X2		X2^2	Y2	===>	0	1	0	B
	1		X3		X3^2	Y3			0	0	1	A
	
	*/
	
	for ( int j = 0; j < length; j++ ) //matrix height
	{
		for ( int i = 0; i < length; i++ )
		{
			p_matrix[j*(length+1) + i] = Exp( p_coordinates[j].x, i );
		}
		p_matrix[j*(length+1) + length] = p_coordinates[j].y;
	}
	
	RowEchelonForm ( p_matrix, length, length+1 );
	
	for ( int i = 0; i < length; i++ )
	{
		p_coefficients[length - 1 - i] = p_matrix[i*(length+1) + length];
	}
	
	
	return p_coefficients;
}

double Exp ( double n, int e )
{
	double res = 1.0;
	
	if ( e >= 0 )
	{
		for ( int i = 0; i < e; i++ )
		{
			res *= n;
		}
	}
	else 
	{
		for ( int i = 0; i > e; i-- )
		{
			res = res / n;
		}
	}
	
	return res;
}

///////////////////////////////////////////////////////////////////////////////
float *PolynomialRegression ( PIXC *p_coordinates, int length, int order )
{
//	using a number of coordinate pairs, output list of coefficients for polynomial of given order
//	using least squares fit

	float *p_coefficients = new float [order + 1];
	
	if ( order == 1 && length > 1 ) //linear regression
	{
		//
	}
	
	//placeholder
	
	return p_coefficients;
}


///////////////////////////////////////////////////////////////////////////////
/*
WARR Distort ( WARR wordArray )
{
	//coefficients for shifts along X and Y axes
	double Ax1, Ax2, Ax3, Bx1, Bx2, Bx3, Cx1, Cx2, Cx3;
	double Ay1, Ay2, Ay3, By1, By2, By3, Cy1, Cy2, Cy3;
	
	float Xdist, Ydist, X, Y;
	WORD val;
	
	
	Xdist = X + ( Ax1*X*X + Bx1*X + Cx1 ) * Y*Y + ( Ax2*X*X + Bx2*X + Cx2 ) * Y + ( Ax3*X*X + Bx3*X + Cx3 );
	Ydist = Y + ( Ay1*Y*Y + By1*Y + Cy1 ) * X*X + ( Ay2*Y*Y + By2*Y + Cy2 ) * X + ( Ay3*Y*Y + By3*Y + Cy3 );
	
	
	val = 0;
	
	return wordArray;
	
}
*/
float *CalculateParabolicCoefficients ( float *p_coordinates, int length )
{
	float	X1 = p_coordinates[0],
			X2 = p_coordinates[1],
			X3 = p_coordinates[2];
	
	int numOfTriples = length/3 - 1;
	/*
	
	Matrix to solve:
	
	X1^2	X1		1		Y1
	X2^2	X2		1		Y2
	X3^2	X3		1		Y3
	
	*/
	
	double *p_matrix = new double [12];
	float intermediate = 0;
	float *p_coefficients = new float [4*numOfTriples];
	//baseline; A; B; C
	
//	
	
	for ( int k = 0; k < numOfTriples; k++ )
	{	
		p_matrix[0] = X1 * X1;
		p_matrix[1] = X1;
		p_matrix[2] = 1;
		p_matrix[3] = p_coordinates[(k+1)*3 + 0];
		p_matrix[4] = X2 * X2;
		p_matrix[5] = X2;
		p_matrix[6] = 1;
		p_matrix[7] = p_coordinates[(k+1)*3 + 1];
		p_matrix[8] = X3 * X3;
		p_matrix[9] = X3;
		p_matrix[10] = 1;
		p_matrix[11] = p_coordinates[(k+1)*3 + 2];
		
		
		RowEchelonForm ( p_matrix, 3, 4 );

		p_coefficients[k*4+0] = p_coordinates[(k+1)*3 + 1];
		intermediate = p_matrix[3];
		p_coefficients[k*4+1] = intermediate;
		intermediate = p_matrix[7];
		p_coefficients[k*4+2] = intermediate;
		intermediate = p_matrix[11] - p_coordinates[(k+1)*3 + 1];
		p_coefficients[k*4+3] = intermediate;
	}
	
	delete[] p_matrix;
	p_matrix = NULL;
	
	return p_coefficients;
}


void Interpolate ( float *p_Array, int length, bool even )
{
	if ( even )
	{
		for ( int i = 1 ; i < length; i+=2 )
		{
			p_Array[i] = ( p_Array[(i+length-1)%length] + p_Array[(i+1)%length] ) / 2;
		}
	}
	else //if odd
	{
		for ( int i = 0 ; i < length; i+=2 )
		{
			p_Array[i] = ( p_Array[(i+length-1)%length] + p_Array[(i+1)%length] ) / 2;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
void OutputSpectrumAsCSVdata ( WARR wordArray )
{
	
//--- duplicate in ListPeaks
	
	int width = wordArray.width;
	int height = wordArray.height;
	int startLine = height / 2 - 16;
	int spectrumLength = width/2;
	int *R = new int [spectrumLength];
	int *G1 = new int [spectrumLength];
	int *G2 = new int [spectrumLength];
	int *B = new int [spectrumLength];
	int *AVG = new int [spectrumLength];
	int *MAX = new int [spectrumLength];
	int *colorValues = new int [4];
	int *MinMaxAvgMed = NULL;
	int *p_VertSum = new int [height];
	int *p_HorSum = new int [width];
	IARR peaks;
	
	
	for ( int i = 0; i < spectrumLength; i++ )
	{
		R[i] = 0;
		G1[i] = 0;
		G2[i] = 0;
		B[i] = 0;
		for ( int j = 0; j < 32; j+=2 )
		{
			if ( wordArray.isReflected )
			{
				R[i]  += wordArray.p_Array[(startLine + j + 0)*width + i*2 + 1];
				G1[i] += wordArray.p_Array[(startLine + j + 0)*width + i*2 + 0];
				G2[i] += wordArray.p_Array[(startLine + j + 1)*width + i*2 + 1];
				B[i]  += wordArray.p_Array[(startLine + j + 1)*width + i*2 + 0];
			}
			else
			{
				R[i]  += wordArray.p_Array[(startLine + j + 0)*width + i*2 + 0];
				G1[i] += wordArray.p_Array[(startLine + j + 0)*width + i*2 + 1];
				G2[i] += wordArray.p_Array[(startLine + j + 1)*width + i*2 + 0];
				B[i]  += wordArray.p_Array[(startLine + j + 1)*width + i*2 + 1];
			}
		}
		colorValues[0] = R[i];
		colorValues[1] = G1[i];
		colorValues[2] = G2[i];
		colorValues[3] = B[i];
		MinMaxAvgMed = FindMinMaxAvgMedValue( colorValues, 4 );
		AVG[i] = MinMaxAvgMed[2];
		MAX[i] = MinMaxAvgMed[1];
	}
	
	
//---	
	//--Actual output 
	ofstream file_output;	//"file_output" is the name of output file stream
	string filename = "Spectrum.csv";
	const char* NameOfFile = filename.c_str();
	
	file_output.open ( NameOfFile, ios::out );
	
	file_output << "X,R,G1,G2,B,Avg,Max\n";

	for ( int i = 0; i < spectrumLength; i++ )
	{
		file_output << i << ',' << R[i] << ',' << G1[i] << ',' << G2[i] << ',' << B[i] << ',' << AVG[i] << ',' << MAX[i];

		if ( i < spectrumLength-1 )
		{
			file_output << "\n";
		}
	}
	
	file_output.close();

	
///----------------------	
	filename = "VerticalSum.csv";
	NameOfFile = filename.c_str();
	
	file_output.open ( NameOfFile, ios::out );
	
	file_output << "Y,Sum\n";

	int sum;
	for ( int j = 0; j < height; j++ )
	{
		sum = 0;
		for ( int i = 0; i < width; i++ )
		{
			sum += wordArray.p_Array[j*width + i];
			if ( i < width - 1 )
			{
				sum += wordArray.p_Array[j*width + i + 1];
			}
			else
			{
				sum += wordArray.p_Array[j*width + i];
			}
			
			if ( j < height - 1 )
			{
				sum += wordArray.p_Array[(j+1)*width + i];
			}
			else
			{
				sum += wordArray.p_Array[j*width + i];
			}
			
			if ( i < width - 1 && j < height - 1)
			{
				sum += wordArray.p_Array[(j+1)*width + i + 1];
			}
			else
			{
				sum += wordArray.p_Array[j*width + i];
			}
		}
		sum = sum / 4;
		p_VertSum[j] = sum;
		
		file_output << j << ',' << sum;

		if ( j < height-1 )
		{
			file_output << "\n";
		}
	}
	
	file_output.close();
	
	
	///----------------------	
	filename = "HorizontalSum.csv";
	NameOfFile = filename.c_str();
	
	file_output.open ( NameOfFile, ios::out );
	
	file_output << "X,Sum\n";

	for ( int j = 0; j < width; j++ )
	{
		sum = 0;
		for ( int i = 0; i < height; i++ )
		{  
			sum += wordArray.p_Array[i*width + j];
			
			if ( i < height - 1 )
			{
				sum += wordArray.p_Array[(i+1)*width + j];
			}
			else
			{
				sum += wordArray.p_Array[i*width + j];
			}
			
			if ( j < width - 1 )
			{
				sum += wordArray.p_Array[i*width + j + 1];
			}
			else
			{
				sum += wordArray.p_Array[i*width + j];
			}
			
			if ( j < width - 1 && i < height - 1)
			{
				sum += wordArray.p_Array[(i+1)*width + j + 1];
			}
			else
			{
				sum += wordArray.p_Array[i*width + j];
			}
		}
		sum = sum / 4;
		p_HorSum[j] = sum;
		
		file_output << j << ',' << sum;
	
		if ( j < width-1 )
		{
			file_output << "\n";
		}
	}
	
	file_output.close();
	
//----------	

//	now there are two arrays with sums in memory
	
//	using array for vertical sums, get strips with dots -- as individual arrays(?)
//		-- maybe just ranges will suffice

	int *p_VS_mmam = FindMinMaxAvgMedValue ( p_VertSum, height );
	int numRanges = 0;
	int index = 0;
	int *p_Ranges = NULL;
	int threshold;
	float multiplier = 1.2;
	
	cout << "\nStatistics for Vertical sums:\n";
	cout << "\nMinimum value = " << p_VS_mmam[0];
	cout << "\nMaximum value = " << p_VS_mmam[1];
	cout << "\nAverage value = " << p_VS_mmam[2];
	
	cout << "\nAverage value times " << multiplier << " is used as threshold.\n";
	
	threshold = p_VS_mmam[2] * multiplier;
	for ( int i = 0; i < height-1; i++ )
	{
		if ( p_VertSum[i+1] >= threshold && p_VertSum[i] < threshold )
		{
			numRanges++;
		}
	}
	
	p_Ranges = new int [numRanges*2];

	
	for ( int i = 0; i < height-1; i++ )
	{ //assuming that start is with low values
		if ( p_VertSum[i+1] >= threshold && p_VertSum[i] < threshold )
		{
			p_Ranges[index] = i;
			index++;
		}
		
		if ( p_VertSum[i+1] < threshold && p_VertSum[i] >= threshold )
		{
			p_Ranges[index] = i;
			index++;
		}
	}
	
	cout << "\nFound ranges: " << numRanges;

	for ( int i = 0; i < numRanges; i++ )
	{
		cout << "\n[" << i+1 << "]\t[ " << p_Ranges[i*2] << "\t- " << p_Ranges[i*2+1] << " ]";
	}
	
	//remake ranges so they have width of power of 2
	//change values in Ranges array from RangeStart-RangeEnd to RangeCenter-RangeWidth
	//make all ranges the same width, equal to the biggest one
	int rangeWidth, rangeCenter;
	int powerOfTwo;
	for ( int i = 0; i < numRanges; i++ )
	{
		rangeWidth = p_Ranges[i*2+1] - p_Ranges[i*2];
		rangeCenter = (p_Ranges[i*2+1] + p_Ranges[i*2]) / 2;
		if ( rangeCenter%2 )
		{
			rangeCenter++;
		}
		
		powerOfTwo = 1;
		while ( rangeWidth >= powerOfTwo )
		{
			powerOfTwo *= 2;
		}
		
		p_Ranges[i*2+0] = rangeCenter;
		p_Ranges[i*2+1] = powerOfTwo;
	}
	
	rangeWidth = p_Ranges[1];
	for ( int i = 0; i < numRanges; i++ )
	{
		if ( rangeWidth < p_Ranges[i*2+1] )
		{
			rangeWidth = p_Ranges[i*2+1];
		}
	}
	
	for ( int i = 0; i < numRanges; i++ )
	{
		p_Ranges[i*2+1] = rangeWidth;
	}
	
	cout << "\nAdjusted ranges:\n#\tCenter:\tWidth:\n";

	for ( int i = 0; i < numRanges; i++ )
	{
		cout << "\n[" << i+1 << "]\t" << p_Ranges[i*2] << "\t" << p_Ranges[i*2+1];
	}
	
	

//	find peaks on each strip
	//temporary -- output spectra per strip
	int **pp_stripSpectra = new int * [numRanges];
	for ( int r = 0; r < numRanges; r++ )
	{
		pp_stripSpectra[r] = new int [width];
		
		for ( int i = 0; i < width; i++ )
		{
			sum = 0;
			if ( i == width-1 )
			{
				for ( int j = p_Ranges[r*2] - p_Ranges[r*2+1]/2; j < p_Ranges[r*2] + p_Ranges[r*2+1]/2; j++ )
				{
					sum += wordArray.p_Array[j*width+i];
				}			
			}
			else
			{
				for ( int j = p_Ranges[r*2] - p_Ranges[r*2+1]/2; j < p_Ranges[r*2] + p_Ranges[r*2+1]/2; j++ )
				{
					sum += wordArray.p_Array[j*width+i] + wordArray.p_Array[j*width+i+1];				
				}
				sum = sum/2;
			}
			pp_stripSpectra[r][i] = sum;
		}
	}
	
	filename = "StripsSpectra.csv";
	NameOfFile = filename.c_str();	
	file_output.open ( NameOfFile, ios::out );
	for ( int i = 0; i < numRanges; i++ )
	{
		file_output << "Strip" << i+1 << ",";
	}
	for ( int i = 0; i < width; i++ )
	{
		file_output << "\n";
		for ( int r = 0; r < numRanges; r++ )
		{
			file_output << pp_stripSpectra[r][i];
			if ( r < numRanges-1 )
			{
				file_output << ",";
			}
		}
	}
	
	file_output.close();
	
	
	//save peaks and their widths for each strip
	
	int **pp_stripPeaks = new int * [numRanges];
	int stripPeakCount = 0, sPCounter = 0;
	multiplier = 1.2;
	//counting the number of peaks to prepare an array
	for ( int r = 0; r < numRanges; r++ )
	{
		//find average for strip to use as threshold
		int *p_S_mmam = FindMinMaxAvgMedValue( pp_stripSpectra[r], width );
		threshold = p_S_mmam[2] * multiplier;
		sPCounter = 0;
		for ( int i = 0; i < width-1; i++ )
		{
			if ( pp_stripSpectra[r][i+1] >= threshold && pp_stripSpectra[r][i] < threshold )
			{
				sPCounter++;
			}
		}
		if ( sPCounter > stripPeakCount )
		{
			stripPeakCount = sPCounter;
		}
	//	pp_stripPeaks[r] = new int [width];
	}
	
	cout << "\nMax count of peaks found is " << stripPeakCount;
	//now as we have upper bound on peak number, make arrays and find values to put into them
	//arrays would have "PeakStart"-"PeakWidth" pairs in them
	for ( int r = 0; r < numRanges; r++ )
	{
		//find average for strip to use as threshold
		int *p_S_mmam = FindMinMaxAvgMedValue( pp_stripSpectra[r], width );
		threshold = p_S_mmam[2] * multiplier;
		pp_stripPeaks[r] = new int [stripPeakCount*2]; //each peak is described by two data values
		for ( int i = 0; i < stripPeakCount*2; i++ )
		{
			pp_stripPeaks[r][i] = 0; //conditioning the array
		}
		
		sPCounter = 0;
		for ( int i = 0; i < width-1; i++ )
		{
			if ( pp_stripSpectra[r][i+1] >= threshold && pp_stripSpectra[r][i] < threshold )
			{ //peak start -- ensure that the value is even
				if ( i%2 == 1 )
				{
					pp_stripPeaks[r][sPCounter*2+0] = i-1;
				}
				else
				{
					pp_stripPeaks[r][sPCounter*2+0] = i;
				}
			}
			
			if ( pp_stripSpectra[r][i+1] < threshold && pp_stripSpectra[r][i] >= threshold )
			{ //peak end -- ensure that the value is even
				//second value would be peak width, so, calculate it such that width is even
				if ( i%2 == 1 )
				{
					pp_stripPeaks[r][sPCounter*2+1] = i+1 - pp_stripPeaks[r][sPCounter*2+0];
				}
				else
				{
					pp_stripPeaks[r][sPCounter*2+1] = i + 2 - pp_stripPeaks[r][sPCounter*2+0];
				}
				
				sPCounter++;
			}
		}
	}
	
//	present them so that extraneous peaks can be removed

	string s_yes = "y";
	string s_switch = ""; //switch = "d"(delete) "u"(unite with previous)
	int row = -1, column = -1;
	bool emptyRow = false;
	cout << "\n\nFound peaks starting at following positions:\n";
	
	while ( s_yes == "y" || s_yes == "yes" )
	{
		//showing list of strip peaks
		for ( int i = 0; i < numRanges; i++ )
		{
			cout << "\tStrip" << i+1;
		}
		cout << "\n";
		
		for ( int i = 0; i < stripPeakCount*2; i += 2 )
		{	
			cout << "[" << i/2 + 1 << "]\t";
			for ( int j = 0; j < numRanges; j++ )
			{
				cout << pp_stripPeaks[j][i] << /*"\t[" << pp_stripPeaks[j][i+1] << "]"*/ "\t";
			}
		//	if ( numRanges < 5 )
		//	{
				cout << "\n";
		//	} 
		}
		
		//editing
		
		cout << "\nEdit the list?(y/n) ";
		cin >> s_yes;
		if ( s_yes == "y" || s_yes == "yes" )
		{
			row = -1;
			column = -1;
			s_switch = "";
			while ( column < 0 || column >= numRanges )
			{
				cout << "\nEnter the strip number (1-" << numRanges << "):";
				cin >> column;
				column--;
			}
			while ( row < 0 || column >= stripPeakCount )
			{
				cout << "\nEnter the peak number (1-" << stripPeakCount << "):";
				cin >> row;
				row--;
			}
			while ( s_switch != "d" && s_switch != "u" )
			{
				cout << "\nDelete or Unite with previous (d/u)?: ";
				cin >> s_switch;
				if ( s_switch == "u" && row > 0 )
				{
					//code where the width of previous peak is extended
					//pp_stripPeaks[column][(row-1)*2 + 1] --modified value
					
					//width of united peak
					pp_stripPeaks[column][(row-1)*2 + 1] = pp_stripPeaks[column][(row)*2] //start of peak to be united
														+ pp_stripPeaks[column][(row)*2 + 1] //plus its width
														- pp_stripPeaks[column][(row-1)*2];	//minus start of previous peak
				}
				
				for ( int i = row; i < stripPeakCount-1; i++ )
				{
					pp_stripPeaks[column][i*2] = pp_stripPeaks[column][(i+1)*2];
					pp_stripPeaks[column][i*2+1] = pp_stripPeaks[column][(i+1)*2+1];
				}
				pp_stripPeaks[column][(stripPeakCount-1)*2] = 0;
				pp_stripPeaks[column][(stripPeakCount-1)*2+1] = 0;
			}

			//add some code to remove lowest full zeros row -- just decrement stripPeakCount
			emptyRow = true;
			for ( int i = 0; i < numRanges; i++ )
			{
				if ( pp_stripPeaks[i][(stripPeakCount-1)*2] != 0 )
				{
					emptyRow = false;
					break;
				}
			}
			
			if ( emptyRow )
			{
				stripPeakCount--;
			}
		}
		
	}
	//now, as we have peak areas sorted out, it is time to find peak centers
	
	//creating an array  of pixel coordinates
	PIXC *p_peakCenterCoordinates = new PIXC [numRanges*stripPeakCount];
	
	for ( int j = 0; j < stripPeakCount; j++ )
	{
		for ( int i = 0; i < numRanges; i++ )
		{
		//	PIXC FindPeakCenterOfMass2D ( WARR wordArray, int peakStartX, int peakStartY, int peakWidth, int peakHeight )
			p_peakCenterCoordinates[j*numRanges+i] = FindPeakCenterOfMass2D (	wordArray, //WARR wordArray
																				pp_stripPeaks[i][j*2], //int peakStartX, 
																				p_Ranges[i*2] - p_Ranges[i*2+1]/2,//int peakStartY, 
																				pp_stripPeaks[i][j*2+1], //int peakWidth, 
																				p_Ranges[i*2+1]); 		//int peakHeight );

		}
	}
	
	//show found coordinate pairs
	cout << "\nCenters of peaks found as Centers of Mass:\n\nX\tY\n";
	for ( int i = 0; i < numRanges*stripPeakCount; i++ )
	{
		cout << p_peakCenterCoordinates[i].x << "\t" << p_peakCenterCoordinates[i].y << "\n";
	}
	
	filename = "PeakCenters.csv";
	NameOfFile = filename.c_str();	
	file_output.open ( NameOfFile, ios::out );
	
	file_output << "X,Y";
	
	for ( int i = 0; i < numRanges*stripPeakCount; i++ )
	{
		file_output << "\n" << p_peakCenterCoordinates[i].x << "," << p_peakCenterCoordinates[i].y;
		
	}
	
	file_output.close();
	
//	fit curves to these dots and get list of coefficients of 2-d transformation/deformation




//	with these coefficients the image could be transformed so tht all dots are in rectangular grid
	
	//delimit the rages on output image
	for ( int j = 0; j < height; j++ )
	{
		for ( int r = 0; r < numRanges; r++ )
		{
			if ( j == ( p_Ranges[r*2] - p_Ranges[r*2+1]/2) || j == ( p_Ranges[r*2] + p_Ranges[r*2+1]/2) )
			{
				for ( int i = 0; i < width; i++ )
				{
					wordArray.p_Array[j*width+i] = 0x0fff;
				}
			}
			if ( j > ( p_Ranges[r*2] - p_Ranges[r*2+1]/2) && j < ( p_Ranges[r*2] + p_Ranges[r*2+1]/2) )
			{
				for ( int i = 0; i < width; i++ )
				{
					for ( int s = 0; s < stripPeakCount; s++ )
					{
						if ( i == pp_stripPeaks[r][s*2]-1 || i == pp_stripPeaks[r][s*2] + pp_stripPeaks[r][s*2+1] )
						{
							wordArray.p_Array[j*width+i] = 0x0fff;
						}
					}
				}
			}
		}
	}
	
	
	
}



///////////////////////////////////////////////////////////////////////////////
void GetStatistics( WARR wordArray )
{
	string s_yes = "", s_output = "";
	stringstream ss_output;
	
	cout << "\nName of array = " << wordArray.name;
	ss_output << "\nName of array = " << wordArray.name;
	cout << "\nSize of array = " << wordArray.size;
	ss_output << "\nSize of array = " << wordArray.size;
	cout << "\nWidth = " << wordArray.width;
	ss_output << "\nWidth = " << wordArray.width;
	cout << "\nHeight = " << wordArray.height;
	ss_output << "\nHeight = " << wordArray.height;
	
//	cout << "\n-1-";
	int size = wordArray.size;
//	cout << "\n-2-";
	float *p_farray = new float [size];
//	cout << "\n-3-";
	for ( int i = 0; i < size; i++ )
	{
		p_farray[i] = float(wordArray.p_Array[i]);
	}
//	cout << "\n-4-";
	float *p_MinMaxAvgMed = FindMinMaxAvgMedValue( p_farray, size );
//	cout << "\n-5-";
	cout << "\nMinimum value = " << p_MinMaxAvgMed[0];
	ss_output << "\nMinimum value = " << p_MinMaxAvgMed[0];
	cout << "\nMaximum value = " << p_MinMaxAvgMed[1];
	ss_output << "\nMaximum value = " << p_MinMaxAvgMed[1];
	cout << "\nAverage value = " << p_MinMaxAvgMed[2];
	ss_output << "\nAverage value = " << p_MinMaxAvgMed[2];
	cout << "\nMedian value = " << p_MinMaxAvgMed[3];
	ss_output << "\nMedian value = " << p_MinMaxAvgMed[3];
	
	int maxPowerOf2 = 1;
	int numBins = 64;
	while ( maxPowerOf2 < p_MinMaxAvgMed[1] )
	{
		maxPowerOf2 = (maxPowerOf2 << 1);
	}
	
	cout << "\nNumber of values ranged by " << numBins << " bins:\n";
	ss_output << "\nNumber of values ranged by " << numBins << " bins:\n";
	int *p_bins = BinnedValues ( wordArray, p_MinMaxAvgMed[1], numBins );
	for ( int i = 0; i < numBins; i++ )
	{
		cout << "\n[" << i+1 << "]\t[" << (i+0)*(maxPowerOf2/numBins) << " - " << (i+1)*(maxPowerOf2/numBins)-1 << "]:\t" << p_bins[i];
		ss_output << "\n[" << i+1 << "]\t[" << (i+0)*(maxPowerOf2/numBins) << " - " << (i+1)*(maxPowerOf2/numBins)-1 << "]:\t" << p_bins[i];
	}
	
	cout << "\nOutput this info to text file?(y/n)";
	cin >> s_yes;
	
	if ( s_yes == "y" || s_yes == "yes" )
	{
		ofstream file_output;	//"file_output" is the name of output file stream
		string filename = "Stat_for_" + wordArray.name + ".txt";
		const char* NameOfFile = filename.c_str();
		
		file_output.open ( NameOfFile, ios::out );
		
		file_output << ss_output;
		
		
	
		file_output.close();
	}
}

///////////////////////////////////////////////////////////////////////////////
int *BinnedValues ( WARR wordArray, int max, int numBins )
{
	int maxPowerOf2 = 1;
	int *p_bins = new int [numBins];
	int numberInBin = 0;
	WORD lowerBinBound = 0, upperBinBound = 1;
	
	while ( maxPowerOf2 < max )
	{
		maxPowerOf2 = (maxPowerOf2 << 1);
	}
	
//	cout << "\nmax power of 2 is " << maxPowerOf2;
	
	for ( int i = 0; i < numBins; i++ )
	{
		numberInBin = 0;
		lowerBinBound = (i+0) * (maxPowerOf2/numBins);
		upperBinBound = (i+1) * (maxPowerOf2/numBins);
	//	cout << "\n" << lowerBinBound << "\t" << upperBinBound; 
		for ( DWORD n = 0; n < wordArray.size; n++ )
		{
			if ( wordArray.p_Array[n] >= lowerBinBound && wordArray.p_Array[n] < upperBinBound )
			{
				numberInBin++;
			}
		}
		p_bins[i] = numberInBin;
	//	cout << "\n[" << lowerBinBound << " - " << upperBinBound-1 << "]:\t" << numberInBin;
	}
	/*
	cout << "\nSize of array is " << wordArray.size;
	cout << "\nChecking total number of binned values: ";
	
	int sum = 0;
	for ( int i = 0; i < numBins; i++ )
	{
		sum += p_bins[i];
	}
	
	cout << sum << "\n";
*/	
	return p_bins;
}


///////////////////////////////////////////////////////////////////////////////
IARR FindPeaks ( int *data, int length )
{
	cout << "\nFinding spectral peaks\n";
	
	int count = 0, bufferSize = length;
	int *buffer = new int [bufferSize]; //for storing peak values right away
	IARR peaks;
	
//	finding peaks via threshold method
	int threshold = 200;
	int peakStart, peakEnd, peakMiddle;
	bool insidePeak = false;
	string s_yes = "";
	while ( s_yes != "y" )
	{
		count = 0;
		cout << "\nEnter threshold value: ";
		cin >> threshold;
		for ( int i = 0; i < length; i++ )
		{
			if ( data[i] > threshold && insidePeak == false )
			{
				insidePeak = true;
				peakStart = i;
			}
			else if ( data[i] > threshold && insidePeak == true )
			{
				//nothing, inside peak, moving to the next data point
			}
			else if ( data[i] < threshold && insidePeak == false )
			{
				//nothing, outside peak, moving to the next data point
			}
			else if ( data[i] < threshold && insidePeak == true )
			{
				insidePeak = false;
				peakEnd = i-1; //as we are outside of peak already, ending datapoint would be the previous one
				peakMiddle = ( peakStart + peakEnd ) / 2;
				buffer[count] = peakMiddle;
				count++;
				if ( count <= 16 )
				{
					cout << "\nPeak #" << count << " at " << i;
				}
			}
		}
		cout << "\nIs peak list acceptable? (y/n): ";
		cin >> s_yes;
	}
	
	peaks.size = count;
	peaks.name = "List of spectral peaks";
	peaks.p_Array = new int [count];
	
	for ( int i = 0; i < count; i++ )
	{
		peaks.p_Array[i] = buffer[i];
	}
	
	delete[] buffer;
	buffer = NULL;
	
	return peaks;
}
/*
IARR FindPeaks ( float *data, int length )
{
	cout << "\nFinding spectral peaks\n";
	
	int count = 0, bufferSize = length;
	float *buffer = new float [bufferSize]; //for storing peak values right away
	IARR peaks;

//	finding peaks via threshold method
	int threshold = 200;
	int peakStart, peakMiddle, peakWidth;
	float *peakValues = NULL;
	bool insidePeak = false;
	string s_yes = "";
	while ( s_yes != "y" )
	{
		count = 0;
		cout << "\nEnter threshold value: ";
		cin >> threshold;
		for ( int i = 0; i < length; i++ )
		{
			if ( data[i] > threshold && insidePeak == false )
			{
				insidePeak = true;
				peakStart = i;
			}
			else if ( data[i] > threshold && insidePeak == true )
			{
				//nothing, inside peak, moving to the next data point
			}
			else if ( data[i] < threshold && insidePeak == false )
			{
				//nothing, outside peak, moving to the next data point
			}
			else if ( data[i] < threshold && insidePeak == true )
			{
				insidePeak = false;
				peakWidth = i - peakStart;
				peakValues = new float [peakWidth];
				for ( int j = 0; j < peakWidth; j++ )
				{
					peakValues[j] = data[peakStart+j];
				}
			//	int FindPeakDerivative ( float *p_array, int length )
				peakMiddle = FindPeakDerivative ( peakValues, peakWidth ) + peakStart;
				delete[] peakValues;
				peakValues = NULL;
			//	peakEnd = i-1; //as we are outside of peak already, ending datapoint would be the previous one
			//	peakMiddle = ( peakStart + peakEnd ) / 2;
				buffer[count] = peakMiddle;
				count++;
				if ( count <= 16 )
				{
					cout << "\nPeak #" << count << " at " << peakMiddle;
				}
			}
		}
		cout << "\nIs peak list acceptable? (y/n): ";
		cin >> s_yes;
	}
	
	peaks.size = count;
	peaks.name = "List of spectral peaks";
	peaks.p_Array = new int [count];
	
	for ( int i = 0; i < count; i++ )
	{
		peaks.p_Array[i] = buffer[i];
	}
	
	delete[] buffer;
	buffer = NULL;
	
	return peaks;
}
*/
FARR FindPeaks ( float *data, int length )
{
	cout << "\nFinding spectral peaks\n";
	
	int count = 0, bufferSize = length;
	float *buffer = new float [bufferSize]; //for storing peak values right away
	FARR peaks;

//	finding peaks via threshold method
	int threshold = 200;
	int peakStart, peakWidth;
	float peakCOM;
	float *peakValues = NULL;
	bool insidePeak = false;
	string s_yes = "";
	while ( s_yes != "y" )
	{
		count = 0;
		cout << "\nEnter threshold value: ";
		cin >> threshold;
		for ( int i = 0; i < length; i++ )
		{
			if ( data[i] > threshold && insidePeak == false )
			{
				insidePeak = true;
				peakStart = i;
			}
			else if ( data[i] > threshold && insidePeak == true )
			{
				//nothing, inside peak, moving to the next data point
			}
			else if ( data[i] < threshold && insidePeak == false )
			{
				//nothing, outside peak, moving to the next data point
			}
			else if ( data[i] < threshold && insidePeak == true )
			{
				insidePeak = false;
				peakWidth = i - peakStart;
				peakValues = new float [peakWidth];
				for ( int j = 0; j < peakWidth; j++ )
				{
					peakValues[j] = data[peakStart+j];
				}
			
				peakCOM = FindPeakCenterOfMass ( peakValues, peakWidth ) + peakStart;
				delete[] peakValues;
				peakValues = NULL;
			
				buffer[count] = peakCOM;
				count++;
				if ( count <= 16 )
				{
					cout << "\nPeak #" << count << " at " << peakCOM;
				}
			}
		}
		cout << "\nIs peak list acceptable? (y/n): ";
		cin >> s_yes;
	}
	
	peaks.size = count;
	peaks.name = "List of spectral peaks";
	peaks.p_Array = new float [count];
	
	for ( int i = 0; i < count; i++ )
	{
		peaks.p_Array[i] = buffer[i];
	}
	
	delete[] buffer;
	buffer = NULL;
	
	return peaks;
}

int FindPeakDerivative ( float *p_array, int length )
{
	//looks at
	int peakIndex = 0;
	float d1, d2; //derivatives
	
	if ( length == 1 ) //only one point -- no derivatives can be extracted
	{
		peakIndex = 0;
	}
	else if ( length == 2 ) //only two points, one derivative could only point at which value is bigger -- that would be a peak
	{
		d1 = p_array[1] - p_array[0];
		if ( d1 > 0 )
		{
			peakIndex = 1;
		}
		else // d1 <= 0
		{
			peakIndex = 0;
		}
	}
	else //3 or more points, can try to find peak
	{
		for ( int i = 1; i < length-1; i++ )
		{
			d1 = p_array[i] - p_array[i-1];
			d2 = p_array[i+1] - p_array[i];
			if ( d1 > 0 && d2 <= 0 )
			{
				//peak found
				peakIndex = i;
				break; //no further traversal is needed
			}
		}
	}
	
	return peakIndex;
}

float FindPeakCenterOfMass ( float *p_array, int length )
{
	float partialPeakIndex = 0;
	
	float sumValues = 0, weightedSum = 0;
	
	for ( int i = 0; i < length; i++ )
	{
		sumValues += p_array[i];
		weightedSum += p_array[i] * float(i);
	}
	
	partialPeakIndex = weightedSum / sumValues;
	
	return partialPeakIndex;
}


///////////////////////////////////////////////////////////////////////////////
PIXC FindPeakCenterOfMass2D ( WARR wordArray, int peakStartX, int peakStartY, int peakWidth, int peakHeight )
{
	PIXC com;
	com.x = 0;
	com.y = 0;
	
	int width = wordArray.width;
//	int height = wordArray.height;
	
	float *p_rowsSums = new float [peakHeight];
	float *p_columnsSums = new float [peakWidth];
	//
	for ( int j = 0; j < peakHeight; j++ )
	{
		p_rowsSums[j] = 0;
		for ( int i = 0; i < peakWidth; i++ )
		{
			p_rowsSums[j] += wordArray.p_Array[(peakStartY+j)*width + peakStartX + i];
		}
	}
	
	for ( int j = 0; j < peakWidth; j++ )
	{
		p_columnsSums[j] = 0;
		for ( int i = 0; i < peakHeight; i++ )
		{
			p_columnsSums[j] += wordArray.p_Array[(peakStartY+i)*width + peakStartX + j];
		}
	}
	
	com.x = FindPeakCenterOfMass ( p_columnsSums, peakWidth ) + peakStartX;
	com.y = FindPeakCenterOfMass ( p_rowsSums, peakHeight ) + peakStartY;
	
	return com;
}

///////////////////////////////////////////////////////////////////////////////
//function reads in the file, byte by byte
CARR ReadFileToCharArray ( string fileName )
{
	ifstream fp_in; //declaring input stream
	
	CARR charArray;
	
	BYTE *p_CharArray = NULL;
	char *byte = new char[1];
	int size;
	const char* NameOfInputFile = fileName.c_str();
	
	fp_in.open ( NameOfInputFile, ios::in | ios:: binary | ios::ate );
	
	while ( fp_in.is_open() != true )
	{
		if ( fileName == "" )
		{
			cout << "\nPlease enter the file name:\n";
		}
		else if ( fileName == "exit" )
		{
			break;
		}
		else
		{
			cout << "\nThere is no file with such name. Try another one.\n";
		}
		cin >> fileName;
		const char* NameOfInputFile = fileName.c_str();
		fp_in.open ( NameOfInputFile, ios::in | ios:: binary | ios::ate );
	}
	size = fp_in.tellg();		//requesting the pointer position from the start of file -- getting the size
	cout << "\nSize of file on disc is: " << size << " bytes.\n";
	
	p_CharArray = new BYTE [ size ];
	
	fp_in.seekg( 0, ios::beg );
	
	for ( int i = 0; i < size; i++ )
	{
		fp_in.read ( byte, 1 );
		p_CharArray[i] = *reinterpret_cast<BYTE*> (byte);
	}
	
	
	charArray.name = fileName;
	charArray.size = size;
	charArray.p_Array = p_CharArray;
	
	
	return charArray;
}


///////////////////////////////////////////////////////////////////////////////
//
int *CharArrayToDIB( CARR charArray, int height, int width, int bpp )
{
	int *p_DibCanvas = NULL;
	int *p_DIBHeader = NULL;
	
	int paletteSize;
	int hRes = 10000;
	int widthCoeff = 1;
	
	while ( bpp != 1 && bpp != 4 && bpp != 8 )
	{
		cout	<< "\nEnter desired bit depth of resulting image(1, 4 or 8)"
				<< "\n(bit, nybble or byte per pixel):";
		cin >> bpp;
	}
	
	paletteSize = (0x1 << bpp);
	widthCoeff = 8 / bpp;
	
	
	if ( height <= 0 )
	{
		cout << "\nEnter desired height:";
		cin >> height;
	}
	
	if ( width == 0 )
	{
		cout << "\nEnter desired width:";
		cin >> width;
	}
	else if ( width < 0 )
	{
		cout << "\nWidth is calculated according to height and bpp: ";
		width = (charArray.size * 8) / (height * bpp);
		cout << width << " px.\n";
	}
	
	
	cout << "\nThe image will be " << width << "px wide and " << height << "px high.\n";
	
//	cout << "\n--1--\n";
	p_DIBHeader = DibHeader ( width, -height, bpp, hRes );
//	cout << "\n--2--\n";
	p_DibCanvas = DibCanvasArray ( p_DIBHeader, 0 );
	
	//for 8-bit
//	cout << "\n--3--\n";
	int concatBytes; 
	
	
//	cout << "\n--4--\n";
	for ( int j = 0; j < height; j++ )
	{
	/*	if ( j%77 == 0 )
		{
			cout << "|" << j/77 << "|";
		}*/
		for ( int i = 0; i < width/widthCoeff; i+=4 )
		{	//packing bytes into int
			concatBytes = 	( charArray.p_Array[j * width/widthCoeff + i + 0] ) +
							( charArray.p_Array[j * width/widthCoeff + i + 1] << 8 ) +
							( charArray.p_Array[j * width/widthCoeff + i + 2] << 16 ) +
							( charArray.p_Array[j * width/widthCoeff + i + 3] << 24);
			p_DibCanvas[10 + paletteSize + j*width/(4*widthCoeff) + i/4] = concatBytes;
		}
	}
	
//	cout << "\n--5--\n";
	
	
	return p_DibCanvas;
}

///////////////////////////////////////////////////////////////////////////////
//
int *WordArrayToDIB( WARR wordArray, int bpp )
{
	int height = wordArray.height; 
	int width = wordArray.width; 
	int *p_DibCanvas = NULL;
	int *p_DIBHeader = NULL;
	
	int paletteSize = 0;
	int hRes = 10000;
	int widthCoeff = 1;
	
	while ( bpp != 1 && bpp != 4 && bpp != 8 && bpp != 16 )
	{
		cout	<< "\nEnter desired bit depth of resulting image(1, 4, 8 or 16)"
				<< "\n(bit, nybble, byte or word per pixel):";
		cin >> bpp;
	}
	
	if ( bpp < 16 )
	{
		paletteSize = (0x1 << bpp);
	}
	
	widthCoeff = 16 / bpp;
	
	
	if ( height <= 0 )
	{
		cout << "\nEnter desired height:";
		cin >> height;
	}
	
	if ( width == 0 )
	{
		cout << "\nEnter desired width:";
		cin >> width;
	}
	else if ( width < 0 )
	{
		cout << "\nWidth is calculated according to height and bpp: ";
		width = (wordArray.size * 16) / (height * bpp);
		cout << width << " px.\n";
	}
	
	
	cout << "\nThe image will be " << width << "px wide and " << height << "px high.\n";
	
//	cout << "\n--1--\n";
	p_DIBHeader = DibHeader ( width, -height, bpp, hRes );
//	cout << "\n--2--\n";
	p_DibCanvas = DibCanvasArray ( p_DIBHeader, 0 );
	
	//for 16-bit
//	cout << "\n--3--\n";
	int concatWords; 
	
	
//	cout << "\n--4--\n";
	for ( int j = 0; j < height; j++ )
	{
	/*	if ( j%77 == 0 )
		{
			cout << "|" << j/77 << "|";
		}*/
		for ( int i = 0; i < width/widthCoeff; i+=2 )
		{	//packing bytes into int
			concatWords = 	( wordArray.p_Array[j * width/widthCoeff + i + 0] ) +
							( wordArray.p_Array[j * width/widthCoeff + i + 1] << 16 );
			p_DibCanvas[10 + paletteSize + j*width/(2*widthCoeff) + i/2] = concatWords;
		}
	}
	
//	cout << "\n--5--\n";
	
	
	return p_DibCanvas;
}


///////////////////////////////////////////////////////////////////////////////
// shrinks 12-bit data into 8 bit, by making values logarythmic
int *WordArrayToDIB8( WARR wordArray )
{
	int height = wordArray.height; 
	int width = wordArray.width;
	
	int *p_DibCanvas = NULL;
	int *p_DIBHeader = NULL;
	
	int paletteSize = 256;
	int hRes = 10000;
	
	int bpp = 8;
	int size = wordArray.size;
	
	WORD lowestValue = 0x0fff; //starting with the biggest one
	
	
	if ( height <= 0 )
	{
		cout << "\nEnter desired height:";
		cin >> height;
	}
	
	if ( width == 0 )
	{
		cout << "\nEnter desired width:";
		cin >> width;
	}
	else if ( width < 0 )
	{
		cout << "\nWidth is calculated according to height and bpp: ";
		width = size / height ;
		cout << width << " px.\n";
	}
	
	for ( int i = 0; i < size; i++ )
	{
		if ( wordArray.p_Array[i] < lowestValue )
		{
			lowestValue = wordArray.p_Array[i];
		}
	}
	
	
	cout << "\nThe image will be " << width << "px wide and " << height << "px high.\n";
	
//	cout << "\n--1--\n";
	p_DIBHeader = DibHeader ( width, -height, bpp, hRes );
//	cout << "\n--2--\n";
	p_DibCanvas = DibCanvasArray ( p_DIBHeader, 0 );
	
	//for 8-bit
//	cout << "\n--3--\n";
	int concatBytes; 
	BYTE b0, b1, b2, b3;
	
//	cout << "\n--4--\n";
	for ( int j = 0; j < height; j++ )
	{
	
		for ( int i = 0; i < width; i+=4 )
		{	//packing bytes into int
			b0 = Shrink12To8Bit ( wordArray.p_Array[j * width + i + 0] - lowestValue );
			b1 = Shrink12To8Bit ( wordArray.p_Array[j * width + i + 1] - lowestValue );
			b2 = Shrink12To8Bit ( wordArray.p_Array[j * width + i + 2] - lowestValue );
			b3 = Shrink12To8Bit ( wordArray.p_Array[j * width + i + 3] - lowestValue );
		
		
			concatBytes = 	b0 + ( b1 << 8 ) + ( b2 << 16 ) + ( b3 << 24);
			p_DibCanvas[10 + paletteSize + j*width/4 + i/4] = concatBytes;
		}
	}
	
//	cout << "\n--5--\n";
	
	
	return p_DibCanvas;
}
///////////////////////////////////////////////////////////////////////////////
//
BYTE Shrink12To8Bit ( WORD val )
{
	BYTE shrinkedValue = 0;
	
	val = val & 0x0fff; //to ensure that 12-bit value is used
	
	if ( val > 0x07ff ) 
	{
		shrinkedValue = 0b11100000 + ((val & 0x07ff) >> 6 );
	}
	else if ( val > 0x03ff ) 
	{
		shrinkedValue = 0b11000000 + ((val & 0x03ff) >> 5 );
	}
	else if ( val > 0x01ff ) 
	{
		shrinkedValue = 0b10100000 + ((val & 0x01ff) >> 4 );
	}
	else if ( val > 0x00ff ) 
	{
		shrinkedValue = 0b10000000 + ((val & 0x00ff) >> 3 );
	}
	else if ( val > 0x007f ) 
	{
		shrinkedValue = 0b01100000 + ((val & 0x007f) >> 2 );
	}
	else if ( val > 0x003f ) 
	{
		shrinkedValue = 0b01000000 + ((val & 0x003f) >> 1 );
	}
	else 
	{
		shrinkedValue = val;
	}
	
	return shrinkedValue;
}


///////////////////////////////////////////////////////////////////////////////
//Takes in 8-bit grayscale DIB and outputs 24-bit fullcolor
int *Dib8toColorDib32 ( int *p_DIb8Canvas, bool isReflected )
{
	//////////////////////////////////
/**
*	p_DIBHeader[0] = dibHeaderSize;
*	p_DIBHeader[1] = width;
*	p_DIBHeader[2] = height;
*	p_DIBHeader[3] = bppColPlan;
*	p_DIBHeader[4] = compress;
*	p_DIBHeader[5] = bitmapSize;
*	p_DIBHeader[6] = hRes;
*	p_DIBHeader[7] = vRes;
*	p_DIBHeader[8] = numColors;
*	p_DIBHeader[9] = impColors;
**/
//	cout << "\n--1--\n";
	//get height, width
	int *p_DibCanvas = NULL;
	int *p_DIBHeader = NULL;
	
	int bitmapSize = p_DIb8Canvas[5] - IMAGE_HEIGHT; //for the images with bpp <= 8, one column is added when calculating bitmapSize
	
	BYTE *p_byteArray = new BYTE [ bitmapSize ]; 
	int dWord;
	
	int width = p_DIb8Canvas[1];
	int height = p_DIb8Canvas[2];
	int bpp = 32;
	int hRes = 10000;
	BYTE 	redPixel, 
			greenPixel, 
			bluePixel; 
			
	int colorPixel = 0;
	
//	cout << "\n--2--\n";
//	cout << "BitmapSize is " << bitmapSize << " bytes.\n";
	
//	cout << "\n--3--\n";
	for ( int i = 0; i < bitmapSize/4; i++ )
	{
		dWord = p_DIb8Canvas[266 + i]; //getting bytes by 4
		p_byteArray[i*4+0] = ( dWord >> 0 ) & 0x000000ff;
		p_byteArray[i*4+1] = ( dWord >> 8 ) & 0x000000ff;
		p_byteArray[i*4+2] = ( dWord >> 16 ) & 0x000000ff;
		p_byteArray[i*4+3] = ( dWord >> 24 ) & 0x000000ff;
	}
		
//	cout << "\n--4--\n";
	p_DIBHeader = DibHeader ( width/2, height/2, bpp, hRes );
//	cout << "\n--5--\n";
	p_DibCanvas = DibCanvasArray ( p_DIBHeader, 0 );
	
//	cout << "\n--6--\n";
	for ( int j = 0; j < -height; j+=2 )
	{
		for ( int i = 0; i < width; i+=2 )
		{
			if ( isReflected )
			{
				redPixel = p_byteArray[(j+0)*width+(i+1)];
				greenPixel = ( p_byteArray[(j+0)*width+(i+0)] + p_byteArray[(j+1)*width+(i+1)] ) / 2;
				bluePixel = p_byteArray[(j+1)*width+(i+0)];
			}
			else
			{
				redPixel = p_byteArray[(j+0)*width+(i+0)];
				greenPixel = ( p_byteArray[(j+0)*width+(i+1)] + p_byteArray[(j+1)*width+(i+0)] ) / 2;
				bluePixel = p_byteArray[(j+1)*width+(i+1)];
			}
			colorPixel = 0;
			colorPixel = ( redPixel << 16 ) + ( greenPixel << 8 ) + ( bluePixel);
			p_DibCanvas[10 + (j/2)*width/2 + i/2] = colorPixel;
		}
	}
//	cout << "\n--7--\n";
//	int shift, nibble, val;
	/*
	cout << "\nShowing start of canvas:\n";
	for ( int i = 0; i < 32; i++ )
	{
		val = p_DibCanvas[i];
		cout << "[";
		for ( int count = 8; count > 0; count-- )
		{
			shift = ( count - 1 ) * 4;
			nibble = ( val >> shift ) & 0xf;
			cout << hex << nibble;//showing int nibble by nibble, starting with the most significant one
		} 
		cout << dec << "]\n";
	}
	*/
//	cout << "\n--8--\n";
	return p_DibCanvas;
}

///////////////////////////////////////////////////////////////////////////////
WARR ArrangeBytesToWords ( CARR charArray )
{
//	cout << "\n---1---\n";
	WARR wordArray;	
	bool reflectRows = true;
//	cout << "\n---2---\n";
	int size = charArray.size;	

//	cout << "\n---3---\n";
	WORD *p_wordArray = new WORD [ size * 8 / 12 ];
	
//	cout << "\n---4---\n";
	WORD b0, b1, b2, b3, b4, b5; 
	WORD w0 = 0, w1 = 0, w2 = 0, w3 = 0;			//12 bits go into each word
	WORD mask = 0x0fff; 
	
	
	int index = 0;
//maybe reflect rows? - do this with switch	
	int rowIndex = 0;
//  4 pixels at a time, IMAGE_WIDTH/4 (930) times per row
//	starting at the end of row, moving backwards to zeroeth pixel, then starting at the end of next row
//	cout << "\n---5---\n";
	for ( int i = 0; i < size; i+=6 )
	{
		b0 = charArray.p_Array[i + 0];
		b1 = charArray.p_Array[i + 1];
		b2 = charArray.p_Array[i + 2];
		b3 = charArray.p_Array[i + 3];
		b4 = charArray.p_Array[i + 4];
		b5 = charArray.p_Array[i + 5];
	/**
	*	Bit packing of sensor data
	*
	*	byte|0       |1       |2       |3       |4       |5       |
	*	pix	|xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx|
	*	0___|3210____|BA987654|________|________|________|________|
	*	1___|____BA98|________|________|76543210|________|________|
	*	2___|________|________|BA987654|________|________|3210____|
	*	3___|________|________|________|________|76543210|____BA98|
	**/
		w0 = ((b1 << 4) + (b0 >> 4)) & mask;
		w1 = ((b0 << 8) + (b3)) & mask;
		w2 = ((b2 << 4) + (b5 >> 4)) & mask;
		w3 = ((b5 << 8) + (b4)) & mask;
		
		index = i * 2 / 3;
		rowIndex = (index / IMAGE_WIDTH) + 1; //start with row 1
		
		if ( reflectRows )
		{	
			p_wordArray[rowIndex*IMAGE_WIDTH - index%IMAGE_WIDTH - 1] = w0;
			p_wordArray[rowIndex*IMAGE_WIDTH - index%IMAGE_WIDTH - 2] = w1;
			p_wordArray[rowIndex*IMAGE_WIDTH - index%IMAGE_WIDTH - 3] = w2;
			p_wordArray[rowIndex*IMAGE_WIDTH - index%IMAGE_WIDTH - 4] = w3;
			

		}
		else
		{
			p_wordArray[index + 0] = w0;
			p_wordArray[index + 1] = w1;
			p_wordArray[index + 2] = w2;
			p_wordArray[index + 3] = w3;
		}
	}
//	cout << "\n---6---\n";
	wordArray.isReflected = reflectRows;
	wordArray.size = size * 8 / 12;
	wordArray.width = IMAGE_WIDTH;
	wordArray.height = IMAGE_HEIGHT;
	wordArray.name = charArray.name;
	wordArray.p_Array = p_wordArray;	
	
//	cout << "\n---7---\n";
	return wordArray;
}

///////////////////////////////////////////////////////////////////////////////
// the array is modified in a way that "0" values are replaced by interpolated values
void RemoveBadPixels ( WARR wordArray )
{
	int size = wordArray.size;
	int height = IMAGE_HEIGHT;
	int width = size / height;
	WORD upperPixel, lowerPixel;
	
	int deadPixelCount = 0;
	int hotPixelCount = 0;
	
	//Import list of hot pixels from txt file, and null the values, making them appear dead
	ifstream fp_in;		//"fp_in" is the name of input file stream
	string s_inputFile = "HotPixels.txt";
	string s_string;
	stringstream ss_value;
	const char* NameOfInputFile = s_inputFile.c_str();
	fp_in.open ( NameOfInputFile, ios::in );	//feeding the file contents to stream fp_in
	int * p_hotPixelsList = NULL;
	
	
	if ( fp_in.is_open() == false )
	{
		cout << "No HotPixels.txt file, proceeding without removing hot pixels.\n";
		fp_in.close();
	}
	else
	{
	//	int index = 0;
	//	int ceil = 100;
		cout << "Counting elements:\n";
		while ( getline ( fp_in, s_string ) )
		{
			hotPixelCount++;
		}		
		fp_in.close();
		
		p_hotPixelsList = new int [hotPixelCount];
		
		fp_in.open ( NameOfInputFile, ios::in );
		for ( int i = 0; i < hotPixelCount; i++ )
		{
			getline ( fp_in, s_string );//getting a line from file
			ss_value.clear();				//clearing stream for accepting new input
			ss_value << s_string;			//feeding ssline stream with string s_string
			
			ss_value >> p_hotPixelsList[i];
		}
		fp_in.close();

		//Punching holes in places of hot pixels
		for ( int i = 0; i < hotPixelCount; i++ )
		{
			wordArray.p_Array[ p_hotPixelsList[i] ] = 0;
		}
	}
	
	
	
	//--end of hot pixels handler
	
	
	for ( int i = 0; i < size; i++ )
	{
		if ( wordArray.p_Array[i] == 0 )
		{
			deadPixelCount++;
		}
	}
	
	cout << "\nThere were " << deadPixelCount << " dead pixels found.\n";
	deadPixelCount = 0;
	
	for ( int j = 2; j < height-2; j++ )
	{
		for ( int i = 0; i < width; i++ )
		{
			if ( wordArray.p_Array[j*width+i] == 0 )
			{
				upperPixel = wordArray.p_Array[(j-2)*width+i];
				lowerPixel = wordArray.p_Array[(j+2)*width+i];
				
				//following is to cover cases when there are two dead pixels (of one color) one above other
				if ( upperPixel == 0 )
				{
					upperPixel = wordArray.p_Array[(j-4)*width+i];
				}
				if ( lowerPixel == 0 )
				{
					lowerPixel = wordArray.p_Array[(j+4)*width+i];
				}
				
				wordArray.p_Array[j*width+i] = (upperPixel + lowerPixel)/2;
				deadPixelCount++;
			}
		}
	}
	
	cout << "\n" << deadPixelCount << " dead pixels were removed.\n";
}

///////////////////////////////////////////////////////////////////////////////
//
void FindHotPixels ( WARR wordArray, WORD threshold )
{
//	PIXC *p_hotPixelsList = NULL;
	
	int size = wordArray.size;
//	int height = IMAGE_HEIGHT;
//	int width = size / height;
	int hotPixelCount = 0;
	int index = 0;
	int ceil = 100;
	
//	cout << "Size is: " << size << " words\n";
	
	if ( threshold == 0 )
	{
		cout << "Please enter the threshold value for hot pixels: ";
		cin >> threshold;
	}


	
	for ( int i = 0; i < size; i++ )
	{
		if ( wordArray.p_Array[i] >= threshold )
		{
			hotPixelCount++;
		}
	}
	
	cout << "\nThere are " << hotPixelCount << " pixels with value bigger than " << threshold << "\n";
	
	int *p_hotPixelsList = new int [hotPixelCount];

	
	if ( hotPixelCount < ceil )
	{
		ceil = hotPixelCount;
	}
	

	for ( int i = 0; i < size; i++ )
	{
		if ( wordArray.p_Array[i] >= threshold )
		{
			p_hotPixelsList[index] = i;
			index++;
		}
	}
	
	cout << "\nFirst hundred:\n";
	for ( int i = 0; i < ceil; i++ )
	{
		cout << "\n" << i+1 << "\t X: " << p_hotPixelsList[i] << ";";
	}
	
	//TO DO: export to text file
	string filename = "HotPixels.txt";
	const char* NameOfFile = filename.c_str();
	
	
	ofstream file_output;	//"file_output" is the name of output file stream
	
	file_output.open ( NameOfFile, ios::out );
	
	for ( int i = 0; i < hotPixelCount; i++ )
	{
		file_output << p_hotPixelsList[i];
		file_output << "\n";
	}
	
	file_output.close();
	
	
	
}


///////////////////////////////////////////////////////////////////////////////
//the fuction reads in cr2 file and makes Dib array in memory
int *ReadCR2toDIB( string fileName )
{
	int CR2_HEIGHT = IMAGE_HEIGHT; //sensor rows
	int CR2_SIZE = 15467760; //file size in bytes
	
	
	ifstream fp_in;	
	
	char *fourBytes; // chunk of the file to be read from file

	string inputFile = fileName + ".bmp", yes, head;

	const char* NameOfInputFile = inputFile.c_str();//the s_fileName should be already with extension
	WORD 		bpp = 8;
						
	int size;
	int *p_DibCanvas = NULL;
	int *p_DIBHeader = NULL;
	
	int height = CR2_HEIGHT;
	int width = CR2_SIZE / CR2_HEIGHT ;
	int hRes = 10000;
	int paletteSize = 0;
	if ( bpp < 16 )
	{
		paletteSize = (0x1 << bpp);
	}
	
	
	fp_in.open ( NameOfInputFile, ios::in | ios:: binary | ios::ate );
	while ( fp_in.is_open() != true )
	{
		cout << "\nThere is no file with such name. Try another one.\n";
		cin >> inputFile;
		const char* NameOfInputFile = inputFile.c_str();
		fp_in.open ( NameOfInputFile, ios::in | ios:: binary | ios::ate );
		if ( inputFile == "exit" )
		{
			break;
		}
	}
	size = fp_in.tellg();		//requesting the pointer position from the start of file -- getting the size
	cout << "\nSize of file on disc is: " << size << " bytes.\n";

	if ( size != CR2_SIZE )
	{
		cout << "ReadCR2toDIB Error: cr2 file of wrong size!\n";
	}

	
	if ( size == CR2_SIZE )
	{
		cout << "\nThis is valid Canon a480/a490/a495 framebuffer data file ( chdk .cr2 raw ).\n";
		fourBytes = new char [4];
		//creating header
		cout << "ReadCR2toDIB: 1 -- creating header\n";
		
		p_DIBHeader = DibHeader ( width, -height, bpp, hRes ); 
		cout << "ReadCR2toDIB: 2 -- creating canvas array\n";
		//creating canvas
		p_DibCanvas = DibCanvasArray ( p_DIBHeader, 0 );
		
		cout << "ReadCR2toDIB: 3 -- placing reading head to the start of file\n";
		
		fp_in.seekg( 0, ios::beg );
		
		int leftoverBytes;
		if ( width%4 == 0 )
		{
			leftoverBytes = 0;
		}
		else
		{
			leftoverBytes = 4 - width%4;
		}
		
		cout << "ReadCR2toDIB: 4 -- reading the file\n";
		
		for ( int j = 0; j < height; j++ )
		{
		
			fp_in.seekg( (j*(width+leftoverBytes)), ios::beg );
			
			for ( int i = 0; i < width/4; i++ )
			{					
				fp_in.read ( fourBytes, 4 );
				p_DibCanvas[10 + paletteSize + j*width/4 + i] = *reinterpret_cast<int*> (fourBytes);
			}
			
		}
		cout << "\nReadCR2toDIB: \nFile contents loaded into memory.\n";
		
	}
	else 
	{
		cout << "\nThis is NOT valid Canon a480/a490/a495 framebuffer data file.\n";
	}
	fp_in.close();
	
	return p_DibCanvas;
}

///////////////////////////////////////////////////////////////////////////////
//This function creates new WORD array and dumps all pixel data from bmp16 file without its BMP and DIB headers
WARR ReadBMP16FileToWordArray ( string fileName )
{
	WARR dataFromImage;
//	dataFromImage.size = IMAGE_HEIGHT * IMAGE_WIDTH;
//	dataFromImage.p_Array = new WORD [ IMAGE_HEIGHT * IMAGE_WIDTH ];
	dataFromImage.isReflected = true;
	//---
	ifstream fp_in;		
	char *twoBytes, *fourBytes; // chunk of the file to be read from file
	const char* NameOfInputFile = fileName.c_str();//the s_fileName should be already with extension
	WORD 	mn,	bpp;	// mn should be = 0x4d42; // "BM"
	int image_size, image_width, image_height; 
	
	
	fp_in.open ( NameOfInputFile, ios::in | ios:: binary | ios::ate );
	while ( fp_in.is_open() != true )
	{
		cout << "\nThere is no file with such name. Try another one.\n";
		cin >> fileName;
		const char* NameOfInputFile = fileName.c_str();
		fp_in.open ( NameOfInputFile, ios::in | ios:: binary | ios::ate );
		if ( fileName == "exit" )
		{
			break;
		}
	}
	
	int size = fp_in.tellg();		//requesting the pointer position from the start of file -- getting the size
	cout << "\nSize of file on disc is: " << size << " bytes.\n";
//	int dibCanvasSize = ( size - 14 ) / 4; //the size in ints
	
	//---Reading from start of file	
	fp_in.seekg( 0, ios::beg );
	twoBytes = new char [2];
	fourBytes = new char [4];
	fp_in.read ( twoBytes, 2 );
	mn = *reinterpret_cast<short int*> (twoBytes);
	
	cout << "\n";
/*	int temporaryValue;
	for ( int i = 0; i < 32; i++ )
	{
		fp_in.read ( fourBytes, 4 );
		temporaryValue = *reinterpret_cast<int*> (fourBytes);
		cout << hex << "[" << i << "]:\t[";
		for ( int j = 0; j < 8; j++ )
		{
			cout << ((temporaryValue >> ((7-j)*4))&0xf);
		}
		cout << dec << "]\n";
	}
	
	*/
	fp_in.seekg( 18, ios::beg );
	
	fp_in.read ( fourBytes, 4 );
	image_width = *reinterpret_cast<int*> (fourBytes);
	
	fp_in.read ( fourBytes, 4 );
	image_height = *reinterpret_cast<int*> (fourBytes);
	if (image_height < 0)
	{
		image_height = 0 - image_height;
	}
	image_size = image_width * image_height;
	dataFromImage.size = image_size;
	dataFromImage.p_Array = new WORD [ image_size ];
	dataFromImage.width = image_width;
	dataFromImage.height = image_height;
	dataFromImage.name = fileName;
	
//	cout << "\nfilesize = " << filesize << "\n";
	cout << "\nwidth = " << image_width << "\n";
	cout << "\nheight = " << image_height << "\n";
	cout << "\nsize = " << image_size << "\n\n";
	
	

	if ( mn == 0x4d42 )
	{
		cout << "\nThis is valid BMP file.\n";
		
		
		fp_in.seekg( 28, ios::beg );
	//	twoBytes = new char [2];
		fp_in.read ( twoBytes, 2 );
		bpp = *reinterpret_cast<short int*> (twoBytes);
		cout << "\nbpp = " << bpp << "\n";
		
		if ( bpp == 16 )
		{
			cout << "This is 16 bpp image.\n";
			cout << "Writing contents of the file into WordArray.\n";
		
			fp_in.seekg( 54, ios::beg );
			for ( DWORD i = 0; i < dataFromImage.size; i++ )
			{
				fp_in.read ( twoBytes, 2 );
				dataFromImage.p_Array[i] = *reinterpret_cast<WORD*> (twoBytes);
			}
			cout << "\nFile contents loaded into memory.\n";
		}
		else
		{
			cout << "\nNot a 16 bpp image, returning empty canvas.\n";
		}
	}
	else 
	{
		cout << "\nThis is NOT valid bmp file.\n";
	}
	fp_in.close();
	
	//---
	return dataFromImage;
}



///////////////////////////////////////////////////////////////////////////////
//Getting 32 bit dib canvas and writing bmp with bit depth set as bpp parameter to disc
void BMPWrite ( int *p_DibCanvas, int targetBpp, string s_fileName )
{
//--this part must insure that function works only with valid bpp values
	while ( targetBpp == 0 && targetBpp != 1 && targetBpp != 4 && targetBpp != 8 && targetBpp != 16 && targetBpp != 24 && targetBpp != 32)	
	{
		cout << "\nTarget bit depth " << targetBpp << " is not supported.\n";
		cout << "\nSet the target bit depth (must be 1, 4, 8, 16, 24 or 32 bits per pixel:\n";
		cout << "targetBpp=";
		cin >> targetBpp;
	}
	
	if ( targetBpp == 1 || targetBpp == 4 || targetBpp == 8 || targetBpp == 16 || targetBpp == 24 || targetBpp == 32 )
	{
		ofstream fp_out;	//"fp_out" is the name of output file stream
		ifstream fp_in;		//"fp_in" is the name of input file stream
			
		bool Error = false;	//value for checking for error on multiple points of function execution
		int paletteSize = 0;//this value is non-zero for bpp <= 8; the value is the number of colors
		int width, 			//pulled from p_DibCanvas[1]
			height, 		//pulled from p_DibCanvas[2]
			bitmapSize, 	//pulled from p_DibCanvas[5]
			leftoverBytes, 	//calculated for ensuring row lengths to be whole number of DWords
			resolution, 	//pulled from p_DibCanvas[6]
			headerSize;		//pulled from p_DibCanvas[0]
		int bpp = ( p_DibCanvas[3] >> 16 );

	//---check if p_DibCanvas is valid
		if ( p_DibCanvas[0] == 0 )
		{
			cout << "\nThe DIB canvas is empty.\nThrowing error\n";
			Error = true;
		}
	//----BMP header parameters	
	//---
		
		int *p_DIBHeaderNew = NULL;
		int size, offset, zero = 0;
		short int head = 0x4d42;	//ascii codes for "BM"
		
		const char* NameOfFile = NULL;

	//---Check if canvas is of correct bit depth	
		if ( targetBpp != bpp )
		{
			cout << "\nBMPWRite ERROR: The DIB canvas is wrong bit depth. BMPWrite attempting to write " << bpp << " dibCanvas as " << targetBpp << " bmp image.\nThrowing error\n";
			Error = true;
		}
		
		if ( Error != true )
		{
			width = p_DibCanvas[1];
			height = p_DibCanvas[2];
			if ( height < 0 )
			{
				height = -height;
			}
			resolution = p_DibCanvas[6];
			if ( targetBpp == 32 )//--Calculating how many leftover bytes are to be added to each row of 24bpp image
			{
				leftoverBytes = 0;
				bitmapSize = height * ( width*4 + leftoverBytes );//for each row there is a padding so that it is a whole number of DWords
			}
			else if ( targetBpp == 24 )//--Calculating how many leftover bytes are to be added to each row of 24bpp image
			{
				if ( (width*3)%4 == 0 )
				{
					leftoverBytes = 0;
				}
				else
				{
					leftoverBytes = 4 - (width*3)%4;
				}
				bitmapSize = height * ( width*3 + leftoverBytes );//for each row there is a padding so that it is a whole number of DWords
			}
			else if ( targetBpp == 16 )//--Calculating how many leftover bytes are to be added to each row of 16bpp image
			{
				if ( (width*2)%4 == 0 )
				{
					leftoverBytes = 0;
				}
				else
				{
					leftoverBytes = 4 - (width*2)%4;
				}
			
				bitmapSize = height * ( width*2 + leftoverBytes );//for each row there is a padding so that it is a whole number of DWords
			}
			else if ( targetBpp == 8 || targetBpp == 4 || targetBpp == 1 )//--Calculating how many leftover bytes are to be added to each row of image
			{
				if ( (width*targetBpp/8)%4 == 0 )
				{
					leftoverBytes = 0;
				}
				else
				{
					leftoverBytes = 4 - (width*targetBpp/8)%4;
				}
				
				paletteSize = ( 1 << targetBpp );
				
				bitmapSize = height * ( width*targetBpp/8 + leftoverBytes );//for each row there is a padding so that it is a whole number of DWords
			}
			else	//redundant check
			{
				cout << "\nBMPWRite ERROR: The targetBpp is wrong value.\nThrowing error\n";
				Error = true;
			}
			
			cout << "\nthere are " << leftoverBytes << " leftover bytes in each row.\n";
			
			p_DIBHeaderNew = DibHeader ( width, height, targetBpp, resolution );//making header
			headerSize = p_DIBHeaderNew[0];
			


			offset = p_DibCanvas[0] + paletteSize*4 + 14; //paletteSize is in int(4-byte) values
			size = offset + bitmapSize;
			
	//---if name is not set as parameter to function, it will prompt user to enter it	
			if ( s_fileName == "" )
			{
				cout << "\nEnter the name of new file (without .bmp extension): ";
				cin >> s_fileName;
				s_fileName += ".bmp";
			}
			
			
			cout << "Writing new bitmap file to disc...\n";
			
			NameOfFile = s_fileName.c_str();
		}

		if ( Error == true )
		{
			cout << "Due to errors encountered, file will not be created.\n";
		}
		else
		{
		//reinterpret_cast is writing LSB first, and MSB last
		//---- Writing BMP Header	
			cout << "\nWriting BMP Header\n";
			fp_out.open ( NameOfFile, ios::out | ios:: binary );	
			fp_out.write ( reinterpret_cast<char *> (& head ), 2 );
			fp_out.write ( reinterpret_cast<char *> (& size ), 4 );
			fp_out.write ( reinterpret_cast<char *> (& zero ), 4 );
			fp_out.write ( reinterpret_cast<char *> (& offset ), 4 );
		//----Inserting	original values for width and height into header
			
		//---- Writing DIB Canvas, with header and optional palette, and with data
			cout << "\nWriting DIB Canvas, with header";
			//--DibHeader
			for ( int i = 0; i < headerSize/4; i++ )
			{
	//			fp_out.write ( reinterpret_cast<char *> ( & p_DIBHeaderNew[i] ), 4 );
				fp_out.write ( reinterpret_cast<char *> ( & p_DibCanvas[i] ), 4 );
			}
			//--Palette, if exists
			if ( paletteSize != 0 )
			{
				cout << ", palette";
				for ( int i = 0; i < paletteSize; i++ )
				{
					fp_out.write ( reinterpret_cast<char *> ( & p_DibCanvas[headerSize/4 + i] ), 4 );
				}
			}
			//--Bitmap
			cout << " and bitmap.\n";
			if ( targetBpp == 32 || targetBpp == 24 )
			{
				for ( int j = 0; j < height; j++ )
				{
					if ( targetBpp == 32 )
					{
						for ( int i = 0; i < width; i++ )
						{
							fp_out.write ( reinterpret_cast<char *> ( & p_DibCanvas[headerSize/4 + j*width + i] ), 4 );
						}
					//	cout << '.';
					//for bpp=32 Each pixel is 4 bytes, so no leftover bytes in this case, and no need to check for them	
					}
					else if ( targetBpp == 24  )
					{
						for ( int i = 0; i < width; i++ )
						{
							fp_out.write ( reinterpret_cast<char *> ( & p_DibCanvas[headerSize/4 + j*width + i] ), 3 );
						}
					//	cout << '.';
						if ( leftoverBytes != 0 )
						{
							for ( int bytes = 0; bytes < leftoverBytes; bytes++ )
							{
								fp_out.write ( reinterpret_cast<char *> ( & zero ), 1 ) ;
					//			cout << '-';
							}
							
						}
					}
				}
			}
			else if ( targetBpp == 16 || targetBpp == 8 || targetBpp == 4 || targetBpp == 1 )	
			{	//
				for ( int i = 0; i < (bitmapSize/4); i++ )
				{
					fp_out.write ( reinterpret_cast<char *> ( & p_DibCanvas[headerSize/4 + paletteSize + i] ), 4 );
				}
		
			}
			
			else 
			{
				cout << "Unsupported target bit depth. Throwing error\n";
				Error = true;
			}
			
			cout << "\n";
			fp_out.close();
			cout << "\nFile written.\n";
		}
		
		cout << "\nCheck the file size\n";
	//---checking if file is written, and telling its size in bytes	
		fp_in.open( NameOfFile, ios::in | ios::binary | ios::ate );  //opening file with pointer positioned At The End of file
		if (fp_in.is_open() == true )
		{
			int fileSize = fp_in.tellg();		//looking up the pointer position: as it is at the end, position=file size
			string s_yes = "n";
			cout << "\nFile size is " << fileSize << " bytes.\n";
			fp_in.close();
			/*
			cout << "Show file info?(y/n): ";
			cin >> s_yes;
			if ( s_yes == "y" || s_yes == "Y" || s_yes == "yes" || s_yes == "Yes" )
			{
				BMPInfo( NameOfFile );
			}
			*/
		}
		else
		{
			cout << "\nSomething went wrong, and there is no such file on disc.\n";
		}
	}
	else
	{
		cout << "\nTarget bit depth " << targetBpp << " is not supported.\nNothing is written to disc.\n";
	}
}


///////////////////////////////////////////////////////////////////////////////
//This function makes BITMAPINFOHEADER (40 byte) DIB header for bmp file, given all parameters
int *DibHeader ( int width, int height, int bpp, int Resolution )
{
	bool Error = false;
	//----DIB header parameters (BITMAPINFOHEADER)
	int	dibHeaderSize = 40, compress = 0, bitmapSize, hRes = Resolution, vRes = Resolution, numColors = 0, impColors = 0; /*, width, height*/
	int colPlan = 1, bppColPlan; // bpp,
	int pixelCount;
	int leftoverBytes = 0;
	int startFromTop = 1;
	if ( height < 0 )
	{
		startFromTop = -1;
		height = -height;
	}
	cout << "Height is " << height << "\n";
//-----Palette parameters
//	cout << "Enter desired image bit depth\nmust be 1, 4, 8, 16, 24 or 32: ";
//	cin >> bpp;
	
	if ( bpp != 1 && bpp != 4 && bpp != 8 && bpp != 16 && bpp != 24 && bpp != 32 )
	{
		cout << "Wrong bit depth!!!\nThrowing Error\n";
		Error = true;
	}
	
	if ( bpp == 1 || bpp == 4 || bpp == 8 )
	{
		numColors = 1;
		impColors = numColors = ( numColors << bpp );	//shifting instead of multipliyng by 2 bpp times
	}
	
	bppColPlan = ( bpp << 16 ) | colPlan;
	
	cout << "DibHeader(int,int,int,int): bpp: " << bpp << "\nbppColplan: " << hex << bppColPlan << dec << "\n";
	

	//Check for valid entry
	if ( ((width*bpp)%32) != 0 )
	{
		cout << "The width of image in bytes is not divisible by 4.\n";//"Need adding padding to ends of rows.\n";
		cout << "Extra " << (32 - ((width*bpp)%32)) << " bits needed to be added to row ends.\n";
		int addedLength;
		if ( bpp == 1 || bpp == 4 || bpp == 8 )
		{
			if ( (width)%4 == 0 )
			{
				leftoverBytes = 0;
			}
			else
			{
				leftoverBytes = 4 - (width)%4;
			}
			addedLength = (8 - ((width*bpp)%8)) / bpp;
			if ( bpp == 1 || bpp == 4 )
			{
				cout << "Of these, " << addedLength << " bits will be visible on image as " << addedLength/bpp << " black pixels on the right end of each row.\n";
			}
		}
		else if ( bpp == 16 )
		{
			if ( (width*2)%4 == 0 )
			{
				leftoverBytes = 0;
			}
			else
			{
				leftoverBytes = 4 - (width*2)%4;
			}
		}
		else if ( bpp == 24 )
		{
		//---adding leftover bytes routine
			
			if ( (width*3)%4 == 0 )
			{
				leftoverBytes = 0;
			}
			else
			{
				leftoverBytes = 4 - (width*3)%4;
			}
			
			
			
		}
//		cout << "Extending width by " << addedLength << " pixels\n";
//		width += addedLength;
	}
	

	
	pixelCount = width*height;
	
	
	cout << "Total number of pixels: " << pixelCount << "\n";
	
	if ( bpp == 1 || bpp == 4 || bpp == 8 )
	{
		//!!!
		bitmapSize = height * ( ((width+1)*bpp/8) + leftoverBytes );//for writing 1bpp, 4bpp and 8bpp bmp files
	}
	else if ( bpp == 16 )
	{
		bitmapSize = height * ( width*2 + leftoverBytes );//for writing 16bpp bmp files
	}
	else if ( bpp == 24 )
	{
		bitmapSize = height * ( width*3 + leftoverBytes );//for writing 24bpp bmp files
	}
	else
	{
		bitmapSize = ( pixelCount * bpp ) / 8;
	}
	
	cout << "It will take up " << bitmapSize << " bytes\n";
//---Actual header array creation
	int *p_DIBHeader = new int [ dibHeaderSize/4 ]; //Making new array
	for ( int i = 0; i < dibHeaderSize/4; i++ )
	{
		p_DIBHeader[i] = 0;	//array conditioning
	}
//---Importing header values into new array	
	if ( Error != true )
	{
		p_DIBHeader[0] = dibHeaderSize;
		p_DIBHeader[1] = width;
		p_DIBHeader[2] = height * startFromTop;
		p_DIBHeader[3] = bppColPlan;
		p_DIBHeader[4] = compress;
		p_DIBHeader[5] = bitmapSize;
		p_DIBHeader[6] = hRes;
		p_DIBHeader[7] = vRes;
		p_DIBHeader[8] = numColors;
		p_DIBHeader[9] = impColors;
	}
//---returning pointer	
	return p_DIBHeader;
}



///////////////////////////////////////////////////////////////////////////////
//This function makes an array which is a representation of the DIB object, just without BMP header.
//Can be used as canvas, which then could be straightforvardly written to disk as BMP file

int *DibCanvasArray ( int *p_DIBHeader, int noPal )
{
//--noPal = 0 -- default, with manual palette choice
//--noPal = 1 -- making canvas with empty palette	
	
//---Setting error flag
	bool Error = false;
//---Checking if header is not empty	
	if ( p_DIBHeader[0] == 0 )
	{
		cout << "\nEmpty DIB header\nThrowing Error\n";
		Error = true;
	}

//////////////////////////////////
/*
	p_DIBHeader[0] = dibHeaderSize;
	p_DIBHeader[1] = width;
	p_DIBHeader[2] = height;
	p_DIBHeader[3] = bppColPlan;
	p_DIBHeader[4] = compress;
	p_DIBHeader[5] = bitmapSize;
	p_DIBHeader[6] = hRes;
	p_DIBHeader[7] = vRes;
	p_DIBHeader[8] = numColors;
	p_DIBHeader[9] = impColors;
*/
//////////////////////////////////	
//---Extracting relevant parameters from the header
	int dibHeaderSize = p_DIBHeader[0]/4; //size of header in int values (4-byte)
	int width = p_DIBHeader[1];
	int height = p_DIBHeader[2];
	int bpp = ((p_DIBHeader[3]) >> 16); //recovering bits per pixel value
	int *p_Palette = NULL;
	int *p_DibCanvas = NULL;
//	WORD paletteChoice;
	
	if ( height < 0 )
	{
		height = 0 - height;
	}

//---Setting up palette		
	int paletteSize = 0;  // paletteSize is used to calculate offset to start of bitmap data
	if ( bpp == 1 || bpp == 4 || bpp == 8 )
	{
		paletteSize = 1;
		paletteSize = ( paletteSize << bpp );	//shifting instead of multipliyng by 2 bpp times
	}
	
//////////////////////////////////
	if ( noPal == 0 )
	{
		p_Palette = PaletteChoice( bpp, 1 ); //made this a function to reduce clutter
	}
	else if ( noPal == 1 )
	{
		p_Palette = EmptyPalette( bpp ); //just zeros, so that palette could be added at other time in bpp reducer function
	}
//////////////////////////////////

//---The sizes are all in ints (4-byte values)
	int dibCanvasSize = width*height*bpp/32 + dibHeaderSize + paletteSize; //size of pixel array + size of DIB header + size of pallete
	
	cout << dec << "The size of DIB Canvas is: " << dibCanvasSize << " int(4-byte) values.\n";
	cout << "This is " << dibCanvasSize*4 << " bytes\n";
	
	
	if ( Error != true )
	{
	p_DibCanvas = new int [ dibCanvasSize ]; //instatiating array of apprpriate size
	int offset = dibHeaderSize + paletteSize;
	//--inserting header into start of the array
		cout << "\nInserting header into start of the array ... \n";
		for ( int i = 0; i < dibHeaderSize; i++ )
		{
			p_DibCanvas[i] = p_DIBHeader[i];
//			cout << hex << p_DibCanvas[i] << "\n"; 
		}
		cout << dec << " done.\n";
	//--adding pallete, if such is present
		if ( paletteSize != 0 )
		{	
			cout << "\nAdding palette at position " << dibHeaderSize << " ... ";
			for ( int i = 0; i < paletteSize; i++ )
			{
				p_DibCanvas[dibHeaderSize + i] = p_Palette[i];
			}
			cout << " done.\n";
		}
	//--cleaning the actual bitmap, which is the rest of array
		cout << "\nFilling Canvas with zeros at position " << offset << " ... ";
		for ( int i = offset; i < dibCanvasSize; i++ )
		{
			p_DibCanvas[i] = 0;
		}
		cout << " done.\n";
	
	
	}
	else
	{
		cout << "\nError has been encountered.\nReturning malformed canvas array.\n";
		p_DibCanvas = new int [ 1 ];
		p_DibCanvas[0] = 0;
	}
	
	return p_DibCanvas;
}


///////////////////////////////////////////////////////////////////////////////
//This function reads in the start of bmp file, and show the parameters of image
void BMPInfo ( string fileName )  //obtaining file name from outside of function
{
	ifstream fp_in;	
	
	char *twoBytes, *fourBytes; // chunk of the file to be read from file

	string inputFile, yes;
//	cout << "Enter name of existing bmp file:\n";
//	cin >> inputFile;
	inputFile = fileName; //
//	inputFile += ".bmp";
	const char* NameOfInputFile = inputFile.c_str();
	short int mn, bpp, imPlan;// = 0x4d42; // "BM"
	int val, size, width, height, numColors, impColors, bitmapSize, hRes, vRes;
	
	int dibHeaderSize, offset, fileSize;
	
	fp_in.open ( NameOfInputFile, ios::in | ios:: binary | ios::ate );
	while ( fp_in.is_open() != true )
	{
		cout << "\nThere is no file with such name. Try another one.\n";
		cin >> inputFile;
		inputFile += ".bmp";
		const char* NameOfInputFile = inputFile.c_str();
		fp_in.open ( NameOfInputFile, ios::in | ios:: binary | ios::ate );
		if ( inputFile == "exit" )
		{
			break;
		}
	}
	size = fp_in.tellg();		//requesting the pointer position from the start of file -- getting the size
	cout << "\nSize of file on disc is: " << size << " bytes.\n";

//---Reading from start of file	
	fp_in.seekg( 0, ios::beg );
	twoBytes = new char [2];
	fp_in.read ( twoBytes, 2 );
	mn = *reinterpret_cast<short int*> (twoBytes);
	if ( mn == 0x4d42 )
	{
		cout << "\nThis is valid BMP file.\nShowing it's parameters:\n";
		fourBytes = new char [4];

		fp_in.read ( fourBytes, 4 );
		fileSize= *reinterpret_cast<int*> (fourBytes);
		cout << "\nFile size is: " << fileSize << " bytes.\n";

		fp_in.read ( fourBytes, 4 );
		fp_in.read ( fourBytes, 4 );
		offset = *reinterpret_cast<int*> (fourBytes);
		cout << "Offset is: " << offset << " bytes.\n";
		
		
	//--- reading DIB header	
		cout << "\nInterpreting data from the DIB header:\n";
		fp_in.read ( fourBytes, 4 );
		dibHeaderSize = *reinterpret_cast<int*> (fourBytes);
		cout << "\nDIB header size is: " << dibHeaderSize << " bytes.\n";
		
		fp_in.read ( fourBytes, 4 );
		width = *reinterpret_cast<int*> (fourBytes);
		cout << "Image Width is: " << width << " pixels.\n";
		
		fp_in.read ( fourBytes, 4 );
		height = *reinterpret_cast<int*> (fourBytes);
		cout << "Image Height is: " << height << " pixels.\n";
		
		fp_in.read ( twoBytes, 2 );	
		imPlan = *reinterpret_cast<short int*> (twoBytes);
		cout << "N of image planes: " << imPlan << ".\n";
		
		fp_in.read ( twoBytes, 2 );	//reusing
		bpp = *reinterpret_cast<short int*> (twoBytes);
		cout << "Color depth is: " << bpp << " bits per pixel.\n";
		
		fp_in.read ( fourBytes, 4 );
		val = *reinterpret_cast<int*> (fourBytes);
		cout << "Compression method is: " << val << ".\n";
		
		fp_in.read ( fourBytes, 4 );
		bitmapSize = *reinterpret_cast<int*> (fourBytes);
		cout << "Bitmap size is: " << bitmapSize << " bytes.\n";
		
		fp_in.read ( fourBytes, 4 );
		hRes = *reinterpret_cast<int*> (fourBytes);
		cout << "Horizontal resolution is: " << hRes << " pixels per meter.\n";
		
		fp_in.read ( fourBytes, 4 );
		vRes = *reinterpret_cast<int*> (fourBytes);
		cout << "Vertical resolution is: " << vRes << " pixels per meter.\n";
		
		fp_in.read ( fourBytes, 4 );
		numColors = *reinterpret_cast<int*> (fourBytes);
		cout << "Number of colors in table is: " << numColors << ".\n";
		
		fp_in.read ( fourBytes, 4 );
		impColors = *reinterpret_cast<int*> (fourBytes);
		cout << "Number of important colors is:" << impColors << ".\n";
		
		cout << "\nDo you want to see whole header in hex view?(y/n):";
		cin >> yes;
		if ( yes == "y" )
		{
			fp_in.seekg( 14, ios::beg );
			
			int nibble, shift;
			for ( int i = 0; i < dibHeaderSize/4; i++ )
			{
				fp_in.read ( fourBytes, 4 );
				val = *reinterpret_cast<int*> (fourBytes);
				cout << dec << "[ " << i << " ]:\t";
				for ( int count = 8; count > 0; count-- )
				{
					shift = ( count - 1 ) * 4;
					nibble = ( val >> shift ) & 0xf;
					cout << hex << nibble;//showing int nibble by nibble, starting with the most significant one
				}
				cout << dec << "\n";
			}
		}
		if ( bpp == 1 || bpp == 4 || bpp == 8 )
		{
			cout << "\nDo you want to see the color table in hex view?(y/n):";
			cin >> yes;
			if ( yes == "y" )
			{
				fp_in.seekg( 14 + dibHeaderSize, ios::beg );
				
				int nibble, shift, counting = 0;
				for ( int i = 0; i < (offset - dibHeaderSize - 14)/4; i++ )
				{
					counting++;
					fp_in.read ( fourBytes, 4 );
					val = *reinterpret_cast<int*> (fourBytes);
					cout << dec << "[" << i << "]:\t";
					for ( int count = 8; count > 0; count-- )
					{
						shift = ( count - 1 ) * 4;
						nibble = ( val >> shift ) & 0xf;
						cout << hex << nibble;//showing int nibble by nibble, starting with the most significant one
					}
					cout << dec << "\n";
					if ( counting%64 == 0 )
					{
					//	cout << "\nPress Enter to see next 64 values in palette.\n";
						cout << "Enter (s) to stop showing palette.\nAnything else to continue:";
						string stop = "a";
						cin >> stop;
						if ( stop == "s" )
						{
							break;
						}
						cin.get();
					}
				}
			}
		}
		
		

		
		
		
	}
	else 
	{
		cout << "\nThis is NOT valid bmp file.\n";
	}
	fp_in.close();
	
	
}


///////////////////////////////////////////////////////////////////////////////
//menu for choosing palettes
int *PaletteChoice( int bpp, int choice )
{
	int *p_Palette = NULL;
//	int choice;
	
	if ( bpp != 1 && bpp != 4 && bpp != 8 && bpp != 16 && bpp != 24 && bpp != 32 )
	{
		cout << "\n*PalleteChoice(int): Wrong bit depth!!!\np_Palette = NULL\n";
	}
	
	if ( bpp == 1 )
	{
		p_Palette = new int [ 2 ];
		if ( choice == 0 && choice != 1 && choice != 2 )
		{
			cout << "Choose a palette:\n Black and White - 1\tManualPalette - 2\n";
			cin >> choice;
		}
		
		if ( choice == 1 )
		{
			p_Palette = GrayPaletteGenerator( bpp );
		}
		else if ( choice == 2 )
		{
			p_Palette = ManualPalette( bpp );
		}
	}
	
	if ( bpp == 4 )
	{
		p_Palette = new int [ 16 ];
		if ( choice == 0 && choice != 1 && choice != 2 && choice != 3 && choice != 4 )
		{	//default option
			cout << "Choose a palette:\n Grey - 1\tRGGB - 2\tRGGBmod - 3\tgRGBPalette - 4\tManualPalette - 5\n";
			cin >> choice;
		}
		if ( choice == 1 )
		{	
			p_Palette = GrayPaletteGenerator( bpp );
		}
		else if ( choice == 2 )
		{
			p_Palette = RGGBPalette ();
		}
		else if ( choice == 3 )
		{
			p_Palette = RGGBmodPalette ();
		}
		else if ( choice == 4 )
		{
			p_Palette = gRGBPalette ();
		}
		else if ( choice == 5 )
		{
			p_Palette = ManualPalette( bpp );
		}
	}
	else if ( bpp == 8 )
	{
		p_Palette = new int [ 256 ];
		if ( choice == 0 && choice != 1 && choice != 2 && choice != 3 && choice != 4 )
		{	//default option
			cout << "Choose a palette:\n Grey - 1\tRRRGGGBB - 2\tCustom - 3\tgRRGGGBB - 4\n";
			cin >> choice;
		}
		if ( choice == 1 )
		{
			p_Palette = GrayPaletteGenerator( bpp );
		}
		else if ( choice == 2 )
		{
			p_Palette = RRRGGGBBPalette();
		}
		else if ( choice == 3 )
		{
			p_Palette = RRRGGGBBPalette();//?????
		}
		else if ( choice == 4 )
		{
			p_Palette = gRRGGGBBPalette();
		}
	}
	
	return p_Palette;
}


///////////////////////////////////////////////////////////////////////////////

int *EmptyPalette( int bpp )
{
	
	bool Error = false;
	int paletteSize = 0;
	if ( bpp == 1 || bpp == 4 || bpp == 8 )
	{
		paletteSize = 1;
		paletteSize = ( paletteSize << bpp );
	
	}
	else
	{
		cout << "Wrong bit depth!\nThrowing error\nSetting the size of palette to 1\n";
		paletteSize = 1;
		Error = true;
	}
	
	int *p_Palette = new int [ paletteSize ];
	
	if ( Error != true )
	{
		for ( int i = 0; i < paletteSize; i++ )
		{
			p_Palette[i] = 0;	//array conditioning
		}
	}
	
	return p_Palette;
}

///////////////////////////////////////////////////////////////////////////////
//This function makes grayscale palette for bid depth given

int *GrayPaletteGenerator ( int bpp )
{
	bool Error = false;
	int paletteSize = 0;
	if ( bpp == 1 || bpp == 4 || bpp == 8 )
	{
		paletteSize = 1;
		paletteSize = ( paletteSize << bpp );
	
	}
	else
	{
		cout << "Wrong bit depth!\nThrowing error\nSetting the size of palette to 1\n";
		paletteSize = 1;
		Error = true;
	}
	
	int *p_Palette = new int [ paletteSize ];
	if ( Error != true )
	{
		for ( int i = 0; i < paletteSize; i++ )
		{
			p_Palette[i] = 0;	//array conditioning
		}
	
	
	//---making greyscale palettes of various bit depths
		int step = 0xffffff / ( paletteSize - 1 );
		cout << "\npalette size = " << paletteSize << "\n";
		cout << "Step is:\t";
		cout << hex << step;
		cout << "\n";
//		int paletteEntry;
		for ( int i = 0; i < paletteSize; i++ )
		{
//			paletteEntry = i * step;
			p_Palette[i] = i * step;
			if ( bpp == 4 )
			{
				cout << "Entry # " << i << "\t:\t" << p_Palette[i] << "\n"; 
			}
//			fp_out.write ( reinterpret_cast<char *> ( & paletteEntry ), 4 );
		}
		cout << "\nPalette had been generated\n";
	}
	
	return p_Palette;
}

///////////////////////////////////////////////////////////////////////////////

int *ManualPalette( int bpp )
{
	
	int *p_Palette = NULL;
	int color, numColors;	//entries for palette
	string standardColors;
	for ( int i = 0; i < bpp; i++ )
	{
		numColors = ( 0x00000001 << bpp );
	}
	
	p_Palette = new int [ numColors ];
	
	cout << "\nThis is manual palette setup for " << bpp << "-bit bitmaps.\n";
	cout << "There are " << numColors << " colors to define.\n";
	cout << "\nDo you want to choose from standard colors?(y/n): ";
	cin >> standardColors;
	if ( standardColors == "y" )
	{
		ListColors();
		for ( int i = 0; i < numColors; i++ )
		{
			cout << "Color #" << i+1 << ":\t";
			color = ColorLUT();
			p_Palette[i] = color;
		}
	}
	else
	{
		cout << "Choose the colour for bits set to 0 (6 hex digits)\n\t\tRRGGBB\n";
		for ( int i = 0; i < numColors; i++ )
		{
			cout << "Color #" << i+1 << ":\t\t\t";
			cin >> hex >> color;
			p_Palette[i] = color;
		}
	}
	cout << "\nPalette is ready.\n";
	
	return p_Palette;
}

///////////////////////////////////////////////////////////////////////////////

int *CustomPalette( int bpp, int *p_PopularColors )
{
	int *p_Palette = NULL;
	int numColors;	//entries for palette
	string standardColors;
	for ( int i = 0; i < bpp; i++ )
	{
		numColors = ( 0x00000001 << bpp );
	}
	
	p_Palette = new int [ numColors ];
	
	for ( int i = 0; i < numColors; i++ )
	{
		p_Palette[i] = p_PopularColors[2*i]; //entries in p_PopularColors are pairs   color - occurence
	}
	
	return p_Palette;
}


///////////////////////////////////////////////////////////////////////////////
int *RRRGGGBBPalette ()
{
	int *p_Palette = new int [ 256 ];
	
	int blueIncrement = 0x00000054,
		greenIncrement = 0x00002400,
		redIncrement = 0x00240000;
	int blueMultiplier, greenMultiplier, redMultiplier;
	
	for ( int i = 0; i < 256; i++ )
	{
		blueMultiplier = i & 0x03;
		greenMultiplier = ( ( i & 0x1c ) >> 2 );
		redMultiplier = ( ( i & 0xe0 ) >> 5 );
		
		p_Palette[i] = (redIncrement*redMultiplier)|(greenIncrement*greenMultiplier)|(blueIncrement*blueMultiplier);
	}
	
	return p_Palette;
}

///////////////////////////////////////////////////////////////////////////////

int *ZZRRBBGGPalette ()
{
	int *p_Palette = new int [ 256 ];
	
	int blueIncrement = 0x00000055,
		greenIncrement = 0x00005500,
		redIncrement = 0x00550000;
	int blueMultiplier, greenMultiplier, redMultiplier;
	
	for ( int i = 0; i < 64; i++ )
	{
		blueMultiplier = i & 0x03;
		greenMultiplier = ( ( i & 0x0c ) >> 2 );
		redMultiplier = ( ( i & 0x30 ) >> 4 );
		
		p_Palette[i] = (redIncrement*redMultiplier)|(greenIncrement*greenMultiplier)|(blueIncrement*blueMultiplier);
	}
	
	return p_Palette;
}

///////////////////////////////////////////////////////////////////////////////

int *gRRGGGBBPalette()
{
	int *p_Palette = new int [ 256 ];
	
	int blueIncrement = 0x00000055,
		greenIncrement = 0x00002400,
		redIncrement = 0x00550000,
		greyIncrement = 0x00020202;
		
	int blueMultiplier, greenMultiplier, redMultiplier;
	
	for ( int i = 0; i < 128; i++ )
	{
		blueMultiplier = i & 0x03;
		greenMultiplier = ( ( i & 0x1c ) >> 2 );
		redMultiplier = ( ( i & 0x60 ) >> 5 );
		
		p_Palette[i] = (redIncrement*redMultiplier)|(greenIncrement*greenMultiplier)|(blueIncrement*blueMultiplier);
	}
	for ( int i = 0; i < 128; i++ )
	{
		
		p_Palette[128 + i] = (greyIncrement*i);
	}
	return p_Palette;
}

///////////////////////////////////////////////////////////////////////////////

int *RGGBPalette ()
{
	int *p_Palette = new int [ 16 ];
	
	int blueIncrement = 0x000000ff,
		greenIncrement = 0x00005500,
		redIncrement = 0x00ff0000;
	int blueMultiplier, greenMultiplier, redMultiplier;
	
	for ( int i = 0; i < 16; i++ )
	{
		blueMultiplier = i & 0x1;
		greenMultiplier = ( ( i & 0x6 ) >> 1 );
		redMultiplier = ( ( i & 0x8 ) >> 3 );
		
		p_Palette[i] = (redIncrement*redMultiplier)|(greenIncrement*greenMultiplier)|(blueIncrement*blueMultiplier);
	}
	
	return p_Palette;
}

///////////////////////////////////////////////////////////////////////////////

int *RGGBmodPalette () // two pink shades replaced by greys
{
	int *p_Palette = new int [ 16 ];
	
	int blueIncrement = 0x000000ff,
		greenIncrement = 0x00005500,
		redIncrement = 0x00ff0000,
		greyIncrement = 0x00555555;
	int blueMultiplier, greenMultiplier, redMultiplier, greyMultiplier;
	
	for ( int i = 0; i < 16; i++ )
	{
		blueMultiplier = i & 0x1;
		greenMultiplier = ( ( i & 0x6 ) >> 1 );
		redMultiplier = ( ( i & 0x8 ) >> 3 );
		
		p_Palette[i] = (redIncrement*redMultiplier)|(greenIncrement*greenMultiplier)|(blueIncrement*blueMultiplier);
//-- replacing pink shades with grays:
		if ( i == 11 || i == 13 )
		{
			greyMultiplier = ( ( i & 0x6 ) >> 1 );
			p_Palette[i] = greyIncrement*greyMultiplier;
		}
	}
	
	return p_Palette;
}

///////////////////////////////////////////////////////////////////////////////

int *gRGBPalette ()
{
	int *p_Palette = new int [ 16 ];
	
	int blueIncrement = 0x000000ff,
		greenIncrement = 0x0000ff00,
		redIncrement = 0x00ff0000,
		greyIncrement = 0x001c1c1c;
	int blueMultiplier, greenMultiplier, redMultiplier;
	
	for ( int i = 0; i < 8; i++ )
	{
		blueMultiplier = i & 0x1;
		greenMultiplier = ( ( i & 0x2 ) >> 1 );
		redMultiplier = ( ( i & 0x4 ) >> 2 );
		
		p_Palette[i] = (redIncrement*redMultiplier)|(greenIncrement*greenMultiplier)|(blueIncrement*blueMultiplier);
	}
	for ( int i = 0; i < 8; i++ )
	{
		p_Palette[8 + i] = (greyIncrement*(i+1));//this is to make 8 grey shades without duplicating black and white
	}
	
	return p_Palette;
}

///////////////////////////////////////////////////////////////////////////////
//

void ListColors ()
{
	cout << "\nList of standard colors.\n\nBasic colors:\n black - 0; blue - 1; green - 4; red - 8; white - f\n";
	cout << "\nAdditional colors:\n sky_blue - 3;\t\tlight_blue - 5;\t\tcyan - 7;\n";
	cout << " dark_green - 2;\tbright_green - 6;\n";
	cout << " dark_orange - a;\torange - c;\t\tyellow - e;\n";
	cout << " magenta - 9;\t\tpink - b;\t\tlight_pink - d\n";
	cout << " dark_grey - b_mod;\tlight_grey - d_mod;\n";
}

///////////////////////////////////////////////////////////////////////////////

int ColorLUT()
{
	int color = 0x00000000;
	string name;
	
	cin >> name;
	if ( name == "0" || name == "black" || name == "BLK" || name == "000" || name == "000000" || name == "00" )
	{
		color = 0x00000000;
	}
	else if ( name == "1" || name == "blue" || name == "BLU" || name == "00f" || name == "0000ff" || name == "03" )
	{
		color = 0x000000ff;
	}
	else if ( name == "2" ||  name == "dark_green" || name == "DGR" || name == "050" || name == "005500" || name == "04" )
	{
		color = 0x00005500;
	}
	else if ( name == "3" || name == "sky_blue" || name == "SBL" || name == "05f" || name == "0055ff" || name == "07" )
	{
		color = 0x000055ff;
	}
	else if ( name == "4" ||  name == "green" || name == "GRN" || name == "0a0" || name == "00aa00" || name == "08" )
	{
		color = 0x0000aa00;
	}
	else if ( name == "5" || name == "light_blue" || name == "LBL" || name == "0af" || name == "00aaff" || name == "0b" )
	{
		color = 0x0000aaff;
	}
	else if ( name == "6" || name == "bright_green" || name == "BGR" || name == "0f0" || name == "00ff00" || name == "0c" )
	{
		color = 0x0000ff00;
	}
	else if ( name == "7" || name == "cyan" || name == "CYN" || name == "0ff" || name == "00ffff" || name == "0f" )
	{
		color = 0x0000ffff;
	}
	else if ( name == "8" || name == "red" || name == "RED" || name == "f00" || name == "ff0000" || name == "30" )
	{
		color = 0x00ff0000;
	}
	else if ( name == "9" || name == "magenta" || name == "MGN" || name == "f0f" || name == "ff00ff" || name == "33" )
	{
		color = 0x00ff00ff;
	}
	else if ( name == "a" || name == "dark_orange" || name == "DOR" || name == "f50" || name == "ff5500" || name == "34" )
	{
		color = 0x00ff5500;
	}
	else if ( name == "b" || name == "pink" || name == "PNK" || name == "f5f" || name == "ff55ff" || name == "37" )
	{
		color = 0x00ff55ff;
	}
	else if ( name == "c" || name == "orange" || name == "ORN" || name == "fa0" || name == "ffaa00" || name == "38" )
	{
		color = 0x00ffaa00;
	}
	else if ( name == "d" || name == "ligtht_pink" || name == "LPK" || name == "faf" || name == "ffaaff" || name == "3b" )
	{
		color = 0x00ffaaff;
	}
	else if ( name == "e" || name == "yellow" || name == "YLW" || name == "ff0" || name == "ffff00" || name == "3c" )
	{
		color = 0x00ffff00;
	}
	else if ( name == "f" || name == "white" || name == "WHT" || name == "f" || name == "fff" || name == "ffffff" || name == "3f" )
	{
		color = 0x00ffffff;
	}
	else if ( name == "b_mod" || name == "dark_grey" || name == "555" || name == "555555" || name == "15" )
	{
		color = 0x00555555;
	}
	else if ( name == "d_mod" || name == "light_grey" || name == "aaa" || name == "aaaaaa" || name == "2a" )
	{
		color = 0x00aaaaaa;
	}
	else
	{
		cout << "\nThere is no " << name << " color in the table, please enter its hex value:\n";
		cout << "\t  RRGGBB\nValue:\t0x";
		cin >> hex >> color;
		while ( (color&0xff000000) != 0 )
		{
			cout << "Value is out of range. Please enter another one.\n";
			cout << "\t  RRGGBB\nValue:\t0x";
			cin >> hex >> color;
		}
		cout << dec << "\n";
	}
	
	cout << "\nColor value (hex) is: 0x";
	ShowIntHex ( color, 6 );
	cout << dec << "\n";
	
	return color;
}


void ShowIntHex ( int value, int n )	//n = number of nibbles shown, min = 1, max = 8
{
	if ( n > 1 && n <=8 )
	{
		int nibble, shift;
		for ( int count = n; count > 0; count-- )
		{
			shift = ( count - 1 ) * 4;
			nibble = ( value >> shift ) & 0xf;
			cout << hex << nibble;//showing int nibble by nibble, starting with the most significant one
		}
	}
	else
	{
		cout << "Wrong number of nibbles!!!\n";
	}
}

///////////////////////////////////////////////////////////////////////////////
int FindMedianValue( int *p_values, int numValues )
{
	int medianValue = 0;
	int smallestValue = p_values[0];
	int index;
	
//	int *p_sortedValues = new int [numValues];
	
	//sort
	for ( int j = 0; j < numValues; j++ )
	{
		for ( int i = j; i < numValues; i++ )
		{
			if ( p_values[i] < smallestValue )
			{
				smallestValue = p_values[i];
				index = i;
			}
		}
		p_values[index] = p_values[j]; //copying value at j to the place of smallest value found
		p_values[j] = smallestValue; //while replacing value at j with that smallest value
	}

	
	if ( numValues%2 == 1 ) //odd number of values
	{
		medianValue = p_values[numValues/2];
	}
	else //even number of values
	{//taking mean of center values
		medianValue = ( p_values[numValues/2] + p_values[(numValues/2)-1] ) / 2;
	}
		
	return medianValue;
}

float FindMedianValue( float *p_values, int numValues )
{
//	cout << "\n---1---";
	float medianValue = 0;
	float smallestValue = p_values[0];
	int index;
//	cout << "\n---2---";
//	int *p_sortedValues = new int [numValues];
	
	//sort -- should do more efficient sort(?)
	for ( int j = 0; j < numValues; j++ )
	{
		for ( int i = j; i < numValues; i++ )
		{
			if ( p_values[i] < smallestValue )
			{
				smallestValue = p_values[i];
				index = i;
			}
		}
		p_values[index] = p_values[j]; //copying value at j to the place of smallest value found
		p_values[j] = smallestValue; //while replacing value at j with that smallest value
	}
//	cout << "\n---3---";
	
	if ( numValues%2 == 1 ) //odd number of values
	{
		medianValue = p_values[numValues/2];
	}
	else //even number of values
	{//taking mean of center values
		medianValue = ( p_values[numValues/2] + p_values[(numValues/2)-1] ) / 2;
	}
		
//	cout << "\n---4---";
	return medianValue;
}

//
int *FindMinMaxAvgMedValue ( int *p_Values, int num )
{
	int *p_result = new int [4];
	int max = p_Values[0];
	int min = p_Values[0];
	int avg = p_Values[0];
	int med = 0;
	
	for ( int i = 1; i < num; i++ )
	{
		if ( p_Values[i] > max )
		{
			max = p_Values[i];
		}
		if ( p_Values[i] < min )
		{
			min = p_Values[i];
		}
		avg += p_Values[i];
	}
	
	avg = avg / num;
	
	if ( num <= 100 )
	{
		med = FindMedianValue( p_Values, num );
	}
	else
	{
		cout << "\nMedian is not computed for arrays bigger than 100 elements";
	}
	
	p_result[0] = min;
	p_result[1] = max;
	p_result[2] = avg;
	p_result[3] = med;
	
	return p_result;
}

float *FindMinMaxAvgMedValue ( float *p_Values, int num )
{
//	cout << "\n--1--";
	float *p_result = new float [4];
	float max = p_Values[0];
	float min = p_Values[0];
	float avg = p_Values[0];
	float med = 0;
//	cout << "\n--2--";
	for ( int i = 1; i < num; i++ )
	{
		if ( p_Values[i] > max )
		{
			max = p_Values[i];
		}
		if ( p_Values[i] < min )
		{
			min = p_Values[i];
		}
		avg += p_Values[i];
	}
//	cout << "\n--3--";
	avg = avg / num;
//	cout << "\n--4--";
	
	if ( num <= 100 )
	{
		med = FindMedianValue( p_Values, num );
	}
	else
	{
		cout << "\nMedian is not computed for arrays bigger than 100 elements";
	}
//	cout << "\n--5--";
	p_result[0] = min;
	p_result[1] = max;
	p_result[2] = avg;
	p_result[3] = med;
//	cout << "\n--6--";
	return p_result;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
//matrix ops
void RowEchelonForm ( double* p_data, int rows, int columns )
{
	NormalizeRows ( p_data, rows, columns );
		
	for ( int i = 0; i < rows-1; i++ )
	{
		SubtractRowDown ( p_data, rows, columns, i );
		NormalizeRows ( p_data, rows, columns );
	}
	
	for ( int i = rows-1; i >= 0; i-- )
	{
		SubtractRowUp (p_data, rows, columns, i );
	}
	
	for ( int i = 0; i < rows*columns; i++ )
	{
		if ( p_data[i] == -0.0 )
		{
			p_data[i] = 0;
		}
	}
}

void NormalizeRows ( double* p_data, int rows, int columns )
{
	double leadingValue = 0;
	int count;
	
	for ( int j = 0; j < rows; j++ )
	{
		count = 0;
		leadingValue = 0;
		while ( leadingValue == 0 )
		{			
			leadingValue = p_data[j*columns + count];
			count++;
		}
		for ( int i = 0; i < columns; i++ )
		{
			p_data[j*columns+i] = p_data[j*columns+i] / leadingValue;
		}
	}
	
}

void SubtractRowDown ( double* p_data, int rows, int columns, int rowToSubtract )
{
	for ( int j = rowToSubtract + 1; j < rows; j++ )
	{
		for ( int i = 0; i < columns; i++ )
		{
			p_data[j*columns+i] = p_data[j*columns+i] - p_data[rowToSubtract*columns+i];
		}
	}
}

void SubtractRowUp ( double* p_data, int rows, int columns, int rowToSubtract )
{
	//find first non-zero value index
	int leadingValueIndex = 0;
	double coefficient = 1;
	
	for ( int i = 0; i < columns; i++ )
	{
		if ( p_data[rowToSubtract*columns+i] != 0 )
		{
			leadingValueIndex = i;
			break;
		}
	}
	
	//subtracting rows such that on leadingValueIndex result become zero
	for ( int j = rowToSubtract-1; j >= 0; j -- )
	{
		coefficient = p_data[j*columns+leadingValueIndex] / p_data[rowToSubtract*columns+leadingValueIndex];
		for ( int i = 0; i < columns; i++ )
		{
			p_data[j*columns+i] = p_data[j*columns+i] - coefficient * p_data[rowToSubtract*columns+i];
		}
	}
	
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////
