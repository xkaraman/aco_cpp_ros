// reading a text file
#include <iostream>
#include <fstream>
#include <sstream> 
using namespace std;

int main () {
  string line;
  
  ifstream myfile ("acotest.txt");
  if (myfile.is_open())
  {
    while ( getline (myfile,line) )
    {
      
	int line_no = 1;
	while (line_no != 10 && getline(myfile,line)) {  
    ++line_no;
	 }                                                        //diavazoume tin grammi 5 apo to txt arxeio pou exei ta dedomena  https://stackoverflow.com/questions/26288145/how-to-read-a-specific-line-from-file-using-fstream-c

	if (line_no == 10) {
    // sLine contains the fifth line in the file.
	cout<< line << '\n';
	} 
	
	    stringstream ss;     
	  
	    /* Storing the whole string into string stream */
	    ss << line; 
	  
	    /* Running loop till the end of the stream */
	    string temp; 
	    int found; 
	    while (!ss.eof()) { 
	  
		/* extracting word by word from stream */
		ss >> temp; 
	  
		/* Checking the given word is integer or not */
		if (stringstream(temp) >> found) 
		    cout << temp << " "; 
	  
		/* To save from space at the end of string */
		temp = ""; 
	    } 

	
    }

    myfile.close();
  }

  else cout << "Unable to open file"; 

  return 0;
}


