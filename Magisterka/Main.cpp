#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

//using namespace std;

class CSVRow
{
public:
	std::string const& operator[](std::size_t index) const
	{
		return m_data[index];
	}
	std::size_t size() const
	{
		return m_data.size();
	}
	void readNextRow(std::istream& str)
	{
		std::string line;
		std::getline(str, line);

		std::stringstream lineStream(line);
		std::string cell;

		m_data.clear();
		while (std::getline(lineStream, cell, ';'))
		{
			m_data.push_back(cell);
		}
		// This checks for a trailing comma with no data after it.
		if (!lineStream && cell.empty())
		{
			// If there was a trailing comma then add an empty element.
			m_data.push_back("");
		}
	}
private:
	std::vector<std::string> m_data;
};

std::istream& operator >> (std::istream& str, CSVRow& data)
{
	data.readNextRow(str);
	return str;
}

class CSVData {
public:
	CSVRow const& operator[](std::size_t index) const
	{
		return m_data[index];
	}
	std::size_t size() const
	{
		return m_data.size();
	}
	void readCSVData(std::ifstream& file) {
		CSVRow* row = new CSVRow();
		while (file >> *row) {
			m_data.push_back(*row);
			row = new CSVRow();
		}
	}
private:
	std::vector<CSVRow> m_data;
};

int main()
{
	std::ifstream       file("csv/testData.csv");

	/*CSVRow              row;
	while (file >> row)
	{
		std::cout << "Row size: " << row.size() << std::endl;
		std::cout << "1st Element(" << row[0] << ")\n";
	}*/

	CSVData data;
	data.readCSVData(file);

	std::cout << "Read " << data.size() << " rows.\n1st row, 1st cell: " << data[0][0] << "\nlast row, 1st cell: " << data[data.size() - 1][0] << std::endl;
	std::system("pause");
	return 0;
}