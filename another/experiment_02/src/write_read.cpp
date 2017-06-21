
//read and write to a file
#include <sstream>
#include <fstream>
#include <cstdio>

using namespace std;
const int N = 540;
struct SannerData {
	int number;
	double data1;
	double data2;
	double data3;
};

void main() {
	SannerData *s[N];
	std::ifstream inputFile("D:\\r_04.dat");
	//open a file to write
	std::ofstream outfile;
	outfile.open("D:\\r_04_smooth.dat");

	std::string line;
	double sum;
	int i = 0;
	while (getline(inputFile, line)) {
		if (!line.length() || line[0] == '#')
			continue;
			s[i] = new SannerData;
			std::istringstream iss(line);
			iss >>s[i]->number>> s[i]->data1>>s[i]->data2>>s[i]->data3;
			sum = s[i]->data1 + s[i]->data2 + s[i]->data3;
			outfile << s[i]->number <<" "<< sum<<endl;
			i++;
	}

	system("pause");
}