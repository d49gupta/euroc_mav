#include "csv_reader.hpp"

// Function to split a string by a delimiter
vector<string> csvReader::split(const string &str, char delimiter) 
{
    vector<string> result;
    stringstream ss(str);
    string token;
    while (getline(ss, token, delimiter)) {
        result.push_back(token);
    }
    return result;
}

// Function to read CSV and store the data
void csvReader::readCSV() {
    ifstream file(this->filePath);
    vector<map<string, float>> csv_vector;

    if (!file.is_open()) {
        cerr << "Failed to open file: " << this->filePath << endl;
        return;
    }

    string line;
    bool first_row = true;
    vector<string> headers;

    // Read the file line by line
    while (getline(file, line)) {
        vector<string> row = split(line, ',');

        if (first_row) {
            headers = row;
            first_row = false;
        } 
        else {
            map<string, float> data_row; 
            for (size_t i = 0; i < row.size(); ++i) {
                data_row[headers[i]] = stof(row[i]);
            }
            this->data.push_back(data_row);
        }
    }

    file.close();
}

// Function to print a row of data
void csvReader::printRow(map<string, float> row) 
{
    for (const auto &pair : row) {
        cout << pair.first << ": " << pair.second << " | ";
    }
    cout<<endl;
}

// Function to print headers
void csvReader::printHeaders(map<string, float> row) 
{
    for (const auto &pair : row) {
        cout << pair.first << " | ";
    }
    cout<<endl;
}