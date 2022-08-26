//raw echelon form for matrix
/*
	The matrix: 
	[	9	3	1	|	3	]
	[	81	9	1	|	1	]
	[	256	16	1	|	3	]
	
	*/
#include <iostream>
#include <iomanip>

using namespace std;

void ShowMatrix ( double* p_data, int rows, int columns );

void RowEchelonForm ( double* p_data, int rows, int columns );
void NormalizeRows ( double* p_data, int rows, int columns );
void SubtractRowDown ( double* p_data, int rows, int columns, int rowToSubtract );
void SubtractRowUp ( double* p_data, int rows, int columns, int rowToSubtract );

int main()
{
	cout << "This is a test for solving linear systems.\n";
	int rows = 3;
	int columns = 4;
	
	double *p_data = new double [rows*columns];
	p_data[0] = 9;
	p_data[1] = 3;
	p_data[2] = 1;
	p_data[3] = 3;
	
	p_data[4] = 81;
	p_data[5] = 9;
	p_data[6] = 1;
	p_data[7] = 1;
	
	p_data[8] = 256;
	p_data[9] = 16;
	p_data[10] = 1;
	p_data[11] = 3;
	
	
	
	cout << "\nInitial matrix:\n";

	ShowMatrix ( p_data, rows, columns );

	
	RowEchelonForm ( p_data, rows, columns );
	
	cout << "\nMatrix in Row Echelon Form:\n";
	ShowMatrix ( p_data, rows, columns );
	
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

void ShowMatrix ( double* p_data, int rows, int columns )
{
	cout << char(218);
	for ( int i = 0; i < columns+2; i++ )
	{
		cout << "\t";
	}
	cout << char(191);
	
	for ( int j = 0; j < rows; j++ )
	{
		cout << "\n" << char(179)<< "\t";
		for ( int i = 0; i < columns; i++ )
		{
			cout << fixed << setprecision(4) << p_data[j*columns + i] << "\t";
			if ( i == columns - 2 )
			{
				cout << "|\t";
			}
		}
		cout << char(179);
		if ( j < rows-1 )
		{
			cout << "\n" << char(179);
			for ( int i = 0; i < columns+2; i++ )
			{
				cout << "\t";
			}
			cout << char(179);
		}
	}
	
	cout << "\n" << char(192);
	for ( int i = 0; i < columns+2; i++ )
	{
		cout << "\t";
	}
	cout << char(217) << "\n";
}
