#include <string>
#include <vector>
#include <map>

using namespace std;

class ParameterReader
{
public:
	ParameterReader() 
	{
		string filename = "/home/kim/dso/src/para.txt";
		ifstream fin( filename.c_str() );
		if ( !fin )
		{
			printf( "parameter file does not exist. \n" );
			return;
		}
		while( !fin.eof() )
		{
			string str;
			getline( fin, str );
			if ( str[ 0 ] == '#' )
			{
				continue;
			}
			int pos = str.find( "=" );
			if ( pos == -1 )
				continue;
			string key = str.substr( 0, pos );
			string value = str.substr( pos + 1, str.length() );
			data[ key ] = value;
			cout<< key <<":="<<value <<endl;;

			if ( !fin.good() )
				break;
		}
	}
	string getData( string key )
    {
        map<string, string>::iterator iter = data.find(key);
        if (iter == data.end())
        {
            cerr<<"Parameter name "<< key <<" not found!"<<endl;
            return string("NOT_FOUND");
        }
        return iter->second;
    }
public:
    map<string, string> data;
};
