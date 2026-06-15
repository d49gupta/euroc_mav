#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <map>

using namespace std;

class csvReader 
{
public:
    csvReader(string filePath) : filePath(filePath) {}
    vector<string> split(const string &str, char delimiter);
    void readCSV();
    void printRow(map<string, float> row);
    void printHeaders(map<string, float> row);
    vector<map<string, float>> data;

private:
    string filePath;
};