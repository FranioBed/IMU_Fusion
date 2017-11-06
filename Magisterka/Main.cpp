#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <gmtl/gmtl.h>
#include <gmtl/Matrix.h>
#include <gmtl/Vec.h>

#define G 9.81f
#define M_PI 3.14159265359
#define HIGH_PASS 0.98
#define LOW_PASS 0.02
#define K 1000.0f
#define CSVFILENAME "csv/PrioData_idle1.csv"
#define RESFILEPATH1 "csv/Result/Sensor1.csv"
#define RESFILEPATH2 "csv/Result/Sensor2.csv"
#define RESFILEPATH3 "csv/Result/Sensor3.csv"
#define RESFILEPATH4 "csv/Result/Sensor4.csv"
#define RESFILEPATH5 "csv/Result/Sensor5.csv"
#define SENSORS_COUNT 5

//using namespace std;

float magneticFieldVector[3] = {
	19662.1, 1712.6, 45279.2
};

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

class Sensor {
public:
	//need data for 3 steps to estimate acceleration
	//float accPrev[3], gyroPrev[3], magPrev[3];
	//float acc[3], gyro[3], mag[3], prioQuat[4];
	//float accNext[3], gyroNext[3], magNext[3];

	gmtl::Vec3f accPrev, gyroPrev, magPrev;
	gmtl::Vec3f acc, gyro, mag;// , prioQuat[4];
	gmtl::Vec3f accNext, gyroNext, magNext;
	Sensor *child;
	gmtl::Vec3f offset, childOffset, parentJointLinAcc, thetaAcc;
	gmtl::EulerAngleXYZf theta;
	gmtl::Quatf estOrientation, prevOrientation, prioOrientation, simpleOrientation;

	Sensor() {};

	void SetData(float quat[4], float acc[3], float gyro[3], float mag[3]) {
		//this or that? xyzw/wxyz
		//prioOrientation.set(quat[3], quat[2], quat[1], quat[0]);
		prioOrientation.set(quat[0], quat[1], quat[2], quat[3]);

		accPrev = this->acc;
		gyroPrev = this->gyro;
		magPrev = this->mag;

		this->acc = accNext;
		this->gyro = gyroNext;
		this->mag = magNext;

		accNext.set(acc);
		gyroNext.set(gyro);
		magNext.set(mag);

		//accNext[0] = -accNext[0];
		//accNext[1] = -accNext[1];
		//accNext[2] = -accNext[2];

		//float tmp = accNext[0];
		//accNext[0] = accNext[2];
		//accNext[2] = tmp;

		//for (int i = 0; i < 3; i++) {
		//	accPrev[i] = this->acc[i];
		//	gyroPrev[i] = this->gyro[i];
		//	magPrev[i] = this->mag[i];

		//	this->acc[i] = accNext[i];
		//	this->gyro[i] = gyroNext[i];
		//	this->mag[i] = magNext[i];

		//	accNext[i] = acc[i];
		//	gyroNext[i] = gyro[i];
		//	magNext[i] = mag[i];

		//	prioQuat[i] = quat[i];
		//}
		////crude...
		//prioQuat[3] = quat[3];
	}

	void init() {
		parentJointLinAcc.set(0, 0, 0);			//for first sensor the parent reference linear acceleration vector is zero, por the next ones it will be overwritten
		estOrientation = VecObs(accNext, magNext);
		CalculateThetaAcc();
		theta.set(thetaAcc[0], thetaAcc[1], thetaAcc[2]);
	}

	//"Vector observation" algorithm taken from Young's paper, aka QUEST
	gmtl::Quatf VecObs(gmtl::Vec3f acc, gmtl::Vec3f mag) {
		gmtl::Vec3f v1, v2, v3;

		gmtl::normalize(acc);
		v3 = acc;

		//check if correct: m - v3(v3 (dot) m)
		gmtl::normalize(mag);
		float dotp = gmtl::dot(v3, mag);
		v1 = mag - v3*dotp;
		gmtl::normalize(v1);

		gmtl::cross(v2, v3, v1);
		
		//matches the paper pseudocode. check if correct, or skip transposition.
		gmtl::Matrix33f r;
		r.set(v1[0], v2[0], v3[0],
			v1[1], v2[1], v3[1],
			v1[2], v2[2], v3[2]);
		gmtl::transpose(r);

		return gmtl::make<gmtl::Quatf>(r);
	}

	void ComplementaryFilter(float dt, float ddt) {
		prevOrientation = estOrientation;
		gmtl::Quatf omega, q, qv0, conj;
		omega = gmtl::makePure(gyro);
		//q = omega * 0.5f * prevOrientation;
		//estOrientation = prevOrientation + q * dt;

		gmtl::Vec3f estLAcc, lt, lr, alpha, sensOffset;
		alpha = (gyroNext - gyroPrev) / ddt;
		conj = gmtl::makeConj(prevOrientation);
		sensOffset = gmtl::makeVec(prevOrientation * gmtl::makePure(offset) * conj);
		lt = gmtl::makeCross(alpha, sensOffset);
		lr = gmtl::dot(gyro, sensOffset)*gyro - sensOffset * gmtl::lengthSquared(gyro);
		estLAcc = gmtl::makeVec(conj * gmtl::makePure(parentJointLinAcc) * prevOrientation) + lt + lr;

		qv0 = VecObs(acc - estLAcc, mag);

		//if (gmtl::dot(qv0, q) < 0) {
		//	qv0 = -qv0;
		//}

		prevOrientation = prevOrientation + (qv0 - prevOrientation) * (float)(1/(float)K);
		gmtl::normalize(prevOrientation);
		q = omega * 0.5f * prevOrientation;
		estOrientation = prevOrientation + q * dt;
		gmtl::normalize(estOrientation);

		if (child != NULL) {
			sensOffset = gmtl::makeVec(prevOrientation * gmtl::makePure(child->offset) * conj);
			lt = gmtl::makeCross(alpha, sensOffset);
			lr = gmtl::dot(gyro, sensOffset)*gyro - sensOffset * gmtl::lengthSquared(gyro);
			gmtl::Vec3f part = lt + lr;
			child->parentJointLinAcc = parentJointLinAcc + gmtl::makeVec(prevOrientation * gmtl::makePure(part) * conj);
		}
	}

	void CalculateThetaAcc() {
		thetaAcc[0] = LOW_PASS * atan2f(acc[2], acc[1]);// *180 / M_PI;
		thetaAcc[1] = LOW_PASS * atan2f(acc[0], acc[2]);// *180 / M_PI;
		thetaAcc[2] = LOW_PASS * atan2f(acc[1], acc[0]);// *180 / M_PI;
	}

	void SimpleComplementary(float dt) {
		CalculateThetaAcc();
		for (int i = 0; i < 3; i++) {
			theta[i] = HIGH_PASS * (theta[i] + gyro[i] * dt) + thetaAcc[i];
		}

		//calculate quaternion from euler angles
		simpleOrientation = gmtl::make<gmtl::Quatf>(theta);
		gmtl::normalize(simpleOrientation);
	}
};

//Old, deprecated, don't use, delete
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
}

// Calculates rotation matrix given euler angles.
gmtl::Matrix33f eulerAnglesToRotationMatrix(float theta[3])
{
	// Calculate rotation about x axis
	gmtl::Matrix33f R_x;
	R_x.set(1, 0, 0,
		0, cos(theta[0]), -sin(theta[0]),
		0, sin(theta[0]), cos(theta[0]));

	// Calculate rotation about y axis
	gmtl::Matrix33f R_y;
	R_y.set(
		cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1])
		);

	// Calculate rotation about z axis
	gmtl::Matrix33f R_z;
	R_z.set(
		cos(theta[2]), -sin(theta[2]), 0,
		sin(theta[2]), cos(theta[2]), 0,
		0, 0, 1);


	// Combined rotation matrix
	gmtl::Matrix33f R;
	R = R_z * R_y * R_x;
	return R;

}

void quatToEulerianAngle(const gmtl::Quatf& q, gmtl::EulerAngleXYZf& euler)
{
	float ysqr = q[1] * q[1];

	// roll (x-axis rotation)
	float t0 = +2.0 * (q[3] * q[0] + q[1] * q[2]);
	float t1 = +1.0 - 2.0 * (q[0] * q[0] + ysqr);
	float r = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q[3] * q[1] - q[2] * q[0]);
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	float p = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q[3] * q[2] + q[0] * q[1]);
	double t4 = +1.0 - 2.0 * (ysqr + q[2] * q[2]);
	float y = std::atan2(t3, t4);
	euler.set(r, p, y);
}

//Get row data and store it in proper arrays.
//Row composition: one int with sensor number (for clarity's sake), 4 floats with quaternion data, 3 floats with gyroscope data, 3 floats with accelerometer data, 3 floats with magnetometer data
void parseRowData(CSVRow row, float *quat,  float *gyro, float *acc,float *mag) {
	quat[0] = atof(row[1].c_str());
	quat[1] = atof(row[2].c_str());
	quat[2] = atof(row[3].c_str());
	quat[3] = atof(row[4].c_str());
	gyro[0] = atof(row[5].c_str());
	gyro[1] = atof(row[6].c_str());
	gyro[2] = atof(row[7].c_str());
	acc[0] = atof(row[8].c_str());
	acc[1] = atof(row[9].c_str());
	acc[2] = atof(row[10].c_str());
	mag[0] = atof(row[11].c_str());
	mag[1] = atof(row[12].c_str());
	mag[2] = atof(row[13].c_str());
}

int main()
{
#pragma region Init

	std::ifstream file(CSVFILENAME);
	std::fstream resfile1, resfile2, resfile3, resfile4, resfile5;
	resfile1.open(RESFILEPATH1, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
	resfile2.open(RESFILEPATH2, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
	resfile3.open(RESFILEPATH3, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
	resfile4.open(RESFILEPATH4, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
	resfile5.open(RESFILEPATH5, std::ios_base::in | std::ios_base::out | std::ios_base::trunc);
	//std::ifstream file("csv/Pendulum_Test03_Trial1_Segment_1.csv");
	//std::ifstream file2("csv/Pendulum_Test03_Trial1_Segment_2.csv");

	/*CSVRow              row;
	while (file >> row)
	{
		std::cout << "Row size: " << row.size() << std::endl;
		std::cout << "1st Element(" << row[0] << ")\n";
	}*/

	//ignore 2 first lines. Crude.
	//std::string ignore;
	//std::getline(file, ignore);
	//std::getline(file, ignore);
	//std::getline(file2, ignore);
	//std::getline(file2, ignore);

	/*float *pitch = new float();
	float *roll = new float();*/

	//Data comp: one row with timestamp, SENSORS_COUNT (5) rows with sensor data.
	
	CSVData data, data2;
	CSVRow curRow, prevRow;
	Sensor s1, s2, s3, s4, s5;

	s1.child = &s2;
	s2.child = &s3;
	s3.child = &s4;
	s4.child = &s5;

	s1.offset.set(0, 0, 0);
	s2.offset.set(0, 0, -0.13f);
	s3.offset.set(0, 0, -0.18f);
	s4.offset.set(0, 0, -0.04f);
	s5.offset.set(0, 0, -0.1f);		//or (0,0.1f,0)

	s1.childOffset.set(0, 0, -0.25f);
	s2.childOffset.set(0, 0, -0.25f);
	s3.childOffset.set(0, 0, 0);
	s4.childOffset.set(0, 0, 0);

	data.readCSVData(file);
	//data2.readCSVData(file2);
	//data.readXFirstRows(file, 1000);
	//data2.readXFirstRows(file2, 1000);

	const int arrayOffset = SENSORS_COUNT + 1;
	float prevTimestamp;
	float timestamp = atof(data[0][0].c_str());
	float nextTimestamp = atof(data[arrayOffset][0].c_str());
	//curRow = data[1];
	//float accData[3] = { atof(curRow[5].c_str()), atof(curRow[6].c_str()), atof(curRow[7].c_str()) };
	//float gyroData[3] = { atof(curRow[8].c_str()), atof(curRow[9].c_str()), atof(curRow[10].c_str()) };
	//float magData[3] = { atof(curRow[11].c_str()), atof(curRow[12].c_str()), atof(curRow[13].c_str()) };
	//curRow = data[2];
	//float accData2[3] = { atof(curRow[5].c_str()), atof(curRow[6].c_str()), atof(curRow[7].c_str()) };
	//float gyroData2[3] = { atof(curRow[8].c_str()), atof(curRow[9].c_str()), atof(curRow[10].c_str()) };
	//float magData2[3] = { atof(curRow[11].c_str()), atof(curRow[12].c_str()), atof(curRow[13].c_str()) };

	float quatData[4], gyroData[3], accData[3], magData[3];
	curRow = data[1];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s1.SetData(quatData, accData, gyroData, magData);

	curRow = data[2];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s2.SetData(quatData, accData, gyroData, magData);
	//float gyroData2[3] = { atof(curRow[1].c_str()), atof(curRow[2].c_str()), atof(curRow[3].c_str()) };
	//float accData2[3] = { atof(curRow[4].c_str()), atof(curRow[5].c_str()), atof(curRow[6].c_str()) };
	//float magData2[3] = { atof(curRow[7].c_str()), atof(curRow[8].c_str()), atof(curRow[9].c_str()) };
	
	curRow = data[3];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s3.SetData(quatData, accData, gyroData, magData);

	curRow = data[4];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s4.SetData(quatData, accData, gyroData, magData);

	curRow = data[5];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s5.SetData(quatData, accData, gyroData, magData);
	//repeat for each sensor

	curRow = data[arrayOffset + 1];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s1.SetData(quatData, accData, gyroData, magData);

	curRow = data[arrayOffset + 2];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s2.SetData(quatData, accData, gyroData, magData);

	curRow = data[arrayOffset + 3];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s3.SetData(quatData, accData, gyroData, magData);

	curRow = data[arrayOffset + 4];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s4.SetData(quatData, accData, gyroData, magData);

	curRow = data[arrayOffset + 5];
	parseRowData(curRow, quatData, gyroData, accData, magData);
	s5.SetData(quatData, accData, gyroData, magData);

	//for first, simpler complementary filter
	/*float *theta = new float[3];
	float *theta2 = new float[3];*/

	//CountThetaAcc(accData, theta);
	//CountThetaAcc(accData2, theta2);

	s1.init();
	s2.init();
	s3.init();
	s4.init();
	s5.init();
#pragma endregion

	for (size_t i = arrayOffset*2; i < data.size(); i+=arrayOffset)
	{
#pragma region StepDataCollection
		/**pitch = 0;
		*roll = 0;*/
		//get data
		prevTimestamp = timestamp;
		timestamp = nextTimestamp;
		nextTimestamp = atof(data[i][0].c_str());
		float dt = nextTimestamp - timestamp;
		float ddt = nextTimestamp - prevTimestamp;
		//prevRow = data[i-1];
		curRow = data[i + 1];
		//float accData[3] = { atof(curRow[5].c_str()), atof(curRow[6].c_str()), atof(curRow[7].c_str()) };
		//float gyroData[3] = { atof(curRow[8].c_str()), atof(curRow[9].c_str()), atof(curRow[10].c_str()) };
		//float magData[3] = { atof(curRow[11].c_str()), atof(curRow[12].c_str()), atof(curRow[13].c_str()) };
		//curRow = data2[i];
		//float accData2[3] = { atof(curRow[5].c_str()), atof(curRow[6].c_str()), atof(curRow[7].c_str()) };
		//float gyroData2[3] = { atof(curRow[8].c_str()), atof(curRow[9].c_str()), atof(curRow[10].c_str()) };
		//float magData2[3] = { atof(curRow[11].c_str()), atof(curRow[12].c_str()), atof(curRow[13].c_str()) };

		//float gyroData[3] = { atof(curRow[1].c_str()), atof(curRow[2].c_str()), atof(curRow[3].c_str()) };
		//float accData[3] = { atof(curRow[4].c_str()), atof(curRow[5].c_str()), atof(curRow[6].c_str()) };
		//float magData[3] = { atof(curRow[7].c_str()), atof(curRow[8].c_str()), atof(curRow[9].c_str()) };
		//curRow = data[i+2];
		//float gyroData2[3] = { atof(curRow[1].c_str()), atof(curRow[2].c_str()), atof(curRow[3].c_str()) };
		//float accData2[3] = { atof(curRow[4].c_str()), atof(curRow[5].c_str()), atof(curRow[6].c_str()) };
		//float magData2[3] = { atof(curRow[7].c_str()), atof(curRow[8].c_str()), atof(curRow[9].c_str()) };
		parseRowData(curRow, quatData, gyroData, accData, magData);
		s1.SetData(quatData, accData, gyroData, magData);

		curRow = data[i + 2];
		parseRowData(curRow, quatData, gyroData, accData, magData);
		s2.SetData(quatData, accData, gyroData, magData);

		curRow = data[i + 3];
		parseRowData(curRow, quatData, gyroData, accData, magData);
		s3.SetData(quatData, accData, gyroData, magData);

		curRow = data[i + 4];
		parseRowData(curRow, quatData, gyroData, accData, magData);
		s4.SetData(quatData, accData, gyroData, magData);

		curRow = data[i + 5];
		parseRowData(curRow, quatData, gyroData, accData, magData);
		s5.SetData(quatData, accData, gyroData, magData);
		//s1.SetData(accData, gyroData, magData);
		//s2.SetData(accData2, gyroData2, magData2);  
#pragma endregion


#pragma region Calculations
		s1.ComplementaryFilter(dt, ddt);
		s2.ComplementaryFilter(dt, ddt);
		s3.ComplementaryFilter(dt, ddt);
		s4.ComplementaryFilter(dt, ddt);
		s5.ComplementaryFilter(dt, ddt);

		s1.SimpleComplementary(dt);
		s2.SimpleComplementary(dt);
		s3.SimpleComplementary(dt);
		s4.SimpleComplementary(dt);
		s5.SimpleComplementary(dt);

		//calculate complementary filter data
		//ComplementaryFilter(accData, gyroData, theta, dt);
		//ComplementaryFilter(accData2, gyroData2, theta2, dt);
		//ComplementaryFilter(accData, gyroData, dt, pitch, roll);

		//calculate quaternion from euler angles
		//gmtl::EulerAngleXYZf euler1;
		//euler1.set(theta[0], theta[1], theta[2]);
		//gmtl::Quatf quat1 = gmtl::make<gmtl::Quatf>(euler1);
		//gmtl::normalize(quat1);

		//calculate orientation matrix
		//gmtl::Matrix33f rotMatrix = eulerAnglesToRotationMatrix(theta);
		//gmtl::Matrix33f rotMatrix2 = eulerAnglesToRotationMatrix(theta2);

		//gmtl::Vec3f gravity, mag;
		//mag.set(magneticFieldVector[0], magneticFieldVector[1], magneticFieldVector[2]);
		//gravity.set(0.0f, 0.0f, G);

		//calculate vectors
		//gmtl::Vec3f rotatedGravity = rotMatrix * gravity;
		//gmtl::Vec3f rotatedGravity2 = rotMatrix2 * rotatedGravity;
		//gmtl::Vec3f counted, counted2;
		//gmtl::cross(counted, rotatedGravity, mag);
		//gmtl::cross(counted2, rotatedGravity2, mag);

		gmtl::Quatf print1, print2;
		gmtl::EulerAngleXYZf e1, e2;
		//print1 = gmtl::makeInvert(s1.prevOrientation);
		print1 = s1.prevOrientation;
		//print2 = s1.prioOrientation;
		print2 = s1.simpleOrientation;

		quatToEulerianAngle(print1, e1);
		//quatToEulerianAngle(print2, e2);
		e2 = s1.theta;
#pragma endregion

#pragma region Results
		//std::cout << "time: " << curRow[0] << "\tTheta x: " << theta[0] << " \tTheta y: " << theta[1] << " \tTheta z: " << theta[2] << "\tcounted: " << counted << std::endl;
		//std::cout << "time: " << "rotated gravity: " << rotatedGravity << "\trotated gravity2: " << rotatedGravity2 << std::endl;
		//std::cout << "time: " << data[i][0] << "\tEstimated quat: " << print1 << "\tPrio: " << print2 << std::endl;
		//gmtl::conj(print1);
		//std::cout << "Conjugate quaternion: " << print1;
		//gmtl::conj(print1);
		//std::cout << "\tEstimated euler angle: " << e1 << "\tPrio: " << e2 << std::endl << std::endl;

		resfile1 << data[i][0] << ";" << print1[0] << ";" << print1[1] << ";" << print1[2] << ";" << print1[3] <<
			";" << print2[0] << ";" << print2[1] << ";" << print2[2] << ";" << print2[3] <<
			";" << e1[0] << ";" << e1[1] << ";" << e1[2] << ";" << e2[0] << ";" << e2[1] << ";" << e2[2] << std::endl;

		//print1 = gmtl::makeInvert(s2.prevOrientation);
		print1 = s2.prevOrientation;
		print2 = s2.simpleOrientation;
		print2 = gmtl::makeConj(print1) * print2 *  print1;
		quatToEulerianAngle(print1, e1);
		//quatToEulerianAngle(print2, e2);
		e2 = s2.theta;
		resfile2 << data[i][0] << ";" << print1[0] << ";" << print1[1] << ";" << print1[2] << ";" << print1[3] <<
			";" << print2[0] << ";" << print2[1] << ";" << print2[2] << ";" << print2[3] <<
			";" << e1[0] << ";" << e1[1] << ";" << e1[2] << ";" << e2[0] << ";" << e2[1] << ";" << e2[2] << std::endl;

		//print1 = gmtl::makeInvert(s3.prevOrientation);
		print1 = s3.prevOrientation;
		print2 = s3.simpleOrientation;
		quatToEulerianAngle(print1, e1);
		//quatToEulerianAngle(print2, e2);
		e2 = s3.theta;
		resfile3 << data[i][0] << ";" << print1[0] << ";" << print1[1] << ";" << print1[2] << ";" << print1[3] <<
			";" << print2[0] << ";" << print2[1] << ";" << print2[2] << ";" << print2[3] <<
			";" << e1[0] << ";" << e1[1] << ";" << e1[2] << ";" << e2[0] << ";" << e2[1] << ";" << e2[2] << std::endl;

		//print1 = gmtl::makeInvert(s4.prevOrientation);
		print1 = s4.prevOrientation;
		print2 = s4.simpleOrientation;
		quatToEulerianAngle(print1, e1);
		//quatToEulerianAngle(print2, e2);
		e2 = s4.theta;
		resfile4 << data[i][0] << ";" << print1[0] << ";" << print1[1] << ";" << print1[2] << ";" << print1[3] <<
			";" << print2[0] << ";" << print2[1] << ";" << print2[2] << ";" << print2[3] <<
			";" << e1[0] << ";" << e1[1] << ";" << e1[2] << ";" << e2[0] << ";" << e2[1] << ";" << e2[2] << std::endl;

		//print1 = gmtl::makeInvert(s5.prevOrientation);
		print1 = s5.prevOrientation;
		print2 = s5.simpleOrientation;
		quatToEulerianAngle(print1, e1);
		//quatToEulerianAngle(print2, e2);
		e2 = s5.theta;
		resfile5 << data[i][0] << ";" << print1[0] << ";" << print1[1] << ";" << print1[2] << ";" << print1[3] <<
			";" << print2[0] << ";" << print2[1] << ";" << print2[2] << ";" << print2[3] <<
			";" << e1[0] << ";" << e1[1] << ";" << e1[2] << ";" << e2[0] << ";" << e2[1] << ";" << e2[2] << std::endl;
#pragma endregion

	}
	//std::system("pause");
	return 0;
}