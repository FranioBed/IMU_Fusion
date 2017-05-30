#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#define M_PI 3.14159265359
#define HIGH_PASS 0.98
#define LOW_PASS 0.02
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
	void readXFirstRows(std::ifstream& file, int count) {
		CSVRow* row = new CSVRow();
		for (int i = 0; i < count; i++) {
			file >> *row;
			m_data.push_back(*row);
			row = new CSVRow();
		}
	}
private:
	std::vector<CSVRow> m_data;
};

void ComplementaryFilter(float accData[3], float gyrData[3], float dt, float *pitch, float *roll)
{
	float pitchAcc, rollAcc;

	// Integrate the gyroscope data -> int(angularSpeed) = angle
	*pitch += ((float)gyrData[0]) * dt; // Angle around the X-axis
	*roll -= ((float)gyrData[1]) * dt;    // Angle around the Y-axis

																  // Compensate for drift with accelerometer data if !bullshit
																  // Sensitivity = -2 to 2 G at 16Bit -> 2G = 32768 && 0.5G = 8192
	int forceMagnitudeApprox = abs(accData[0]) + abs(accData[1]) + abs(accData[2]);
	if (true)//(forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
	{
		// Turning around the X axis results in a vector on the Y-axis
		pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
		*pitch = *pitch /** 0.98*/ + pitchAcc * 0.02;

		// Turning around the Y axis results in a vector on the X-axis
		rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
		*roll = *roll /** 0.98*/ + rollAcc * 0.02;
	}
}

//TODO: refactor name
void CountThetaAcc(float accData[3], float *thetaAcc) {
	thetaAcc[0] = LOW_PASS * atan2f(accData[2], accData[1]) * 180 / M_PI;
	thetaAcc[1] = LOW_PASS * atan2f(accData[0], accData[2]) * 180 / M_PI;
	thetaAcc[2] = LOW_PASS * atan2f(accData[1], accData[0]) * 180 / M_PI;
}

void ComplementaryFilter(float accData[3], float gyrData[3], float *theta, float dt) {
	float* thetaAcc = new float[3];
	CountThetaAcc(accData, thetaAcc);
	for (int i = 0; i < 3; i++) {
		theta[i] = HIGH_PASS * (theta[i] + gyrData[i] * dt) + thetaAcc[i];
	}
	//delete thetaAcc;
}

int main()
{
	std::ifstream file("csv/Pendulum_Test03_Trial1_Segment_1.csv");

	/*CSVRow              row;
	while (file >> row)
	{
		std::cout << "Row size: " << row.size() << std::endl;
		std::cout << "1st Element(" << row[0] << ")\n";
	}*/

	//ignore 2 first lines. Crude.
	std::string ignore;
	std::getline(file, ignore);
	std::getline(file, ignore);

	float *pitch = new float();
	float *roll = new float();
	//accelerometer column nums: 5, 6, 7
	//gyro colum nums: 8,9,10
	CSVData data;
	CSVRow curRow, prevRow;
	data.readCSVData(file);
	//data.readXFirstRows(file, 1000);
	curRow = data[0];
	float accData[3] = { atof(curRow[5].c_str()), atof(curRow[6].c_str()), atof(curRow[7].c_str()) };
	float *theta = new float[3]; 
	CountThetaAcc(accData, theta);

	for (size_t i = 1; i < data.size(); i++)
	{
		*pitch = 0;
		*roll = 0;
		prevRow = data[i-1];
		curRow = data[i];
		float accData[3] = { atof(curRow[5].c_str()), atof(curRow[6].c_str()), atof(curRow[7].c_str()) };
		float gyroData[3] = { atof(curRow[8].c_str()), atof(curRow[9].c_str()), atof(curRow[10].c_str()) };
		float dt = atof(curRow[0].c_str()) - atof(prevRow[0].c_str());
		ComplementaryFilter(accData, gyroData, theta, dt);
		//ComplementaryFilter(accData, gyroData, dt, pitch, roll);
		std::cout << "time: " << curRow[0] << "\tTheta x: " << theta[0] << " \tTheta y: " << theta[1] << " \tTheta z: " << theta[2] << std::endl;
	}
	std::system("pause");
	return 0;
}