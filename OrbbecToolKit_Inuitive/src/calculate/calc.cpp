#include "calc.h"

typedef struct _RGB_888BIT
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
}RGB_888BIT;

Calc* Calc::gInstance = nullptr;
Calc* Calc::Instance()
{
	if (nullptr == gInstance)
	{
		gInstance = new Calc();
	}
	return gInstance;
}

Calc::Calc()
{
	memset(&mPalletIntsRed, 0, sizeof(mPalletIntsRed));
	memset(&mPalletIntsGreen, 0, sizeof(mPalletIntsGreen));
	memset(&mPalletIntsBlue, 0, sizeof(mPalletIntsBlue));

	//createColorPallet();
	CreateRainbowPallet();
}
void Calc::createColorPallet() {
	int r[] = { 0,0,1,25,49,73,98,122,146,162,173,184,195,207,217,229,240,252,255,255,255,255,255,255,255,255,255,255,255,255,255,255 };
	int g[] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,14,35,57,79,101,117,133,147,161,175,190,205,219,234,248,255,255,255,255 };
	int b[] = { 0,61,96,130,165,192,220,227,210,181,151,122,93,64,35,5,0,0,0,0,0,0,0,0,0,0,0,35,98,160,223,255 };

	/*int r[] = { 90,92,92,95,99,93,98,122,146,162,173,184,195,207,217,220,230,232,235,240,245,248,250,250,250,250,255,255,255,255,255,255 };
	int g[] = { 10,10,10,12,13,14,15,10,11,12,13,15,21,24,35,57,79,101,117,133,147,161,175,190,205,219,234,248,255,255,255,255 };
	int b[] = { 50,61,96,90,50,80,80,100,150,160,151,122,93,64,59,55,54,60,80,90,100,120,130,140,150,160,180,100,98,160,100,100 };
    */

	for (int i = 0; i < 64; i++) {
		if (i < 32) {


			mPalletIntsRed[i] = (uint8_t)r[i];
			mPalletIntsGreen[i] = (uint8_t)g[i];
			mPalletIntsBlue[i] = (uint8_t)b[i];
		}
		else {
			mPalletIntsRed[i] = (uint8_t)r[63 - i];
			mPalletIntsGreen[i] = (uint8_t)g[63 - i];
			mPalletIntsBlue[i] = (uint8_t)b[63 - i];
		}
	}
}
void Calc::CreateRainbowPallet()
{
	uint8_t r, g, b;
	for (int i = 0; i < 256; i++)
	{
		if (i == 0) {
			r = 0;
			g = 0;
			b = 0;
		}
		else if (0 <= i && i <= 31)//31
		{

			r = 0;
			g = 0;
			b = (uint8_t)(128 + 4 * i);
			//qDebug() << i << b << g << r;
		}
		else if (i == 32)
		{
			r = 0;
			g = 0;
			b = 250;
		}
		else if (33 <= i && i <= 95)
		{
			r = 0;
			g = (4 + 4 * (i - 33));
			b = (250);
			//qDebug() << i << b << g << r;
		}
		else if (i == 96)
		{
			r = (uint8_t)(2);
			g = (uint8_t)255;
			b = 250;
			//qDebug() << i << b << g << r;
		}
		else if (97 <= i  && i <= 158)
		{
			r = (uint8_t)(6 + 4 * (i - 97));
			g = (uint8_t)(255);
			b = 250 - 4 * (i - 97);
			//qDebug() << i << b << g << r;
		}
		else if (i == 159)
		{
			r = (uint8_t)(250);
			g = (uint8_t)(255);
			b = 1;
		}
		else if (160 <= i  && i <= 223)
		{
			r = (uint8_t)(250);
			g = (uint8_t)(252 - 4 * (i - 160));
			b = 0;
			//qDebug() << i << b << g << r;
		}
		else if (224 <= i  && i <= 255)
		{
			r = (uint8_t)(252 - 4 * (i - 224));
			g = (uint8_t)(0);
			b = 0;
			//qDebug() << i << b << g << r;
		}

		mPalletIntsRed[i] = b;
		mPalletIntsGreen[i] = g;
		mPalletIntsBlue[i] = r;


	}
}


void Calc::getHistogram(const int16_t* src, int8_t* dst, int width, int height, bool YFlip)
{
	float depthHistogram[80000];
	int numberOfPoints = 0;

	auto histSize = sizeof(depthHistogram);
	memset(depthHistogram, 0, sizeof(depthHistogram));
	for (int h = 0; h < height; ++h)
	{
		for (int w = 0; w < width; ++w)
		{
			int32_t iIndex = h * width + w;
			int32_t depthData = src[iIndex];

			if (depthData > 10)
			{
				depthHistogram[depthData]++;
				numberOfPoints++;
			}
		}
	}

	int32_t num = sizeof(depthHistogram) / sizeof(int);
	for (int32_t nIndex = 1; nIndex < sizeof(depthHistogram) / sizeof(int); nIndex++)
	{
		depthHistogram[nIndex] += depthHistogram[nIndex - 1];
	}
	for (uint32_t nIndex = 1; nIndex < sizeof(depthHistogram) / sizeof(int); nIndex++)
	{
		depthHistogram[nIndex] = (numberOfPoints - depthHistogram[nIndex]) / numberOfPoints;
	}

	RGB_888BIT *dstRgbPtr = (RGB_888BIT*)dst;
	for (int h = 0; h < height; ++h)
	{
		int yU = (height - h - 1) * width;
		if (YFlip == false)
		{
			yU = h * width;
		}
		for (int w = 0; w < width; ++w)
		{
			uint32_t iIndex = h * width + w;
			uint32_t iIndex888 = (yU + w);
			uint32_t depthData = src[iIndex];
			uint8_t depthValue = qRound(depthHistogram[depthData] * 255.0);


			dstRgbPtr[iIndex888].r = depthValue;
			dstRgbPtr[iIndex888].g = depthValue;
			dstRgbPtr[iIndex888].b = depthValue;
		}
	}
}

int Calc::calcParamMat(const OBCameraParams& cam_par, float *mat_16)
{
	qDebug() << "OBCameraParams:";
	qDebug() << QString("l_fx:%1 l_fy:%2 l_cx:%3 l_cy:%4")
		.arg(cam_par.l_intr_p[0])
		.arg(cam_par.l_intr_p[1])
		.arg(cam_par.l_intr_p[2])
		.arg(cam_par.l_intr_p[3]);

	qDebug() << QString("r_fx:%1 r_fy:%2 r_cx:%3 r_cy:%4")
		.arg(cam_par.r_intr_p[0])
		.arg(cam_par.r_intr_p[1])
		.arg(cam_par.r_intr_p[2])
		.arg(cam_par.r_intr_p[3]);

	mMatrix4.fill(0.0f);

	QMatrix4x4 RK_Mat(cam_par.r_intr_p[0], 0.0, cam_par.r_intr_p[2], 0.0,
		0.0, cam_par.r_intr_p[1], cam_par.r_intr_p[3], 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	QMatrix4x4 LK_Mat(cam_par.l_intr_p[0], 0.0, cam_par.l_intr_p[2], 0.0,
		0.0, cam_par.l_intr_p[1], cam_par.l_intr_p[3], 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0);

	QMatrix4x4 R2L_Mat(cam_par.r2l_r[0], cam_par.r2l_r[1], cam_par.r2l_r[2], cam_par.r2l_t[0],
		cam_par.r2l_r[3], cam_par.r2l_r[4], cam_par.r2l_r[5], cam_par.r2l_t[1],
		cam_par.r2l_r[6], cam_par.r2l_r[7], cam_par.r2l_r[8], cam_par.r2l_t[2],
		0.0, 0.0, 0.0, 1.0);

	QMatrix4x4 All_Mat = RK_Mat * R2L_Mat * LK_Mat.inverted();

	float m_d2c_mat[16] = { 0 };
	m_d2c_mat[0] = All_Mat(0, 0);
	m_d2c_mat[1] = All_Mat(0, 1);
	m_d2c_mat[2] = All_Mat(0, 2);
	m_d2c_mat[3] = All_Mat(0, 3);
	m_d2c_mat[4] = All_Mat(1, 0);
	m_d2c_mat[5] = All_Mat(1, 1);
	m_d2c_mat[6] = All_Mat(1, 2);
	m_d2c_mat[7] = All_Mat(1, 3);
	m_d2c_mat[8] = All_Mat(2, 0);
	m_d2c_mat[9] = All_Mat(2, 1);
	m_d2c_mat[10] = All_Mat(2, 2);
	m_d2c_mat[11] = All_Mat(2, 3);
	m_d2c_mat[12] = All_Mat(3, 0);
	m_d2c_mat[13] = All_Mat(3, 1);
	m_d2c_mat[14] = All_Mat(3, 2);
	m_d2c_mat[15] = All_Mat(3, 3);

	memcpy(mat_16, m_d2c_mat, sizeof(m_d2c_mat));

	return 0;
}

void Calc::loadPly(const QString ply_filename)
{
	if (true == mLoadPlyFinishFlag)
	{
		mPlyFilename = ply_filename;
		std::thread depth_calc_thread(&Calc::ThreadLoadPly, this);
		depth_calc_thread.detach();
	}
}

void Calc::calcMapping(MappingData& mapping_data)
{
	if (nullptr == mapping_data.depthPtr.data8bitPtr
		|| nullptr == mapping_data.colorPtr.data8bitPtr
		|| mapping_data.depthPtr.width != mapping_data.colorPtr.width)
	{
		mMappingFinishFlag = true;
		return;
	}
	RGB_888BIT *leftRgbPtr = (RGB_888BIT*)mapping_data.depthPtr.data8bitPtr;
	RGB_888BIT *rightRgbPtr = (RGB_888BIT*)mapping_data.colorPtr.data8bitPtr;

	auto h_min = qMin(mapping_data.depthPtr.height, mapping_data.colorPtr.height);
	auto h_max = qMax(mapping_data.depthPtr.height, mapping_data.colorPtr.height);
	bool is_color_height_max = mapping_data.colorPtr.height > mapping_data.depthPtr.height;

	int img_width = mapping_data.depthPtr.width;
	int img_height = mapping_data.depthPtr.height;

	auto mapping_size = img_width * h_max * 3;
	uint8_t * mapping_tmp = new uint8_t[mapping_size]();
	RGB_888BIT *mp_ = (RGB_888BIT*)mapping_tmp;

	for (int h = 0; h < h_max; h++)
	{
		for (int w = 0; w < img_width; w++)
		{
			auto index = (h * img_width + w);
			if (h < h_min)
			{
				mp_[index].r = (leftRgbPtr[index].r * mDepthRatio + rightRgbPtr[index].r * (1 - mDepthRatio));
				mp_[index].g = (leftRgbPtr[index].g * mDepthRatio + rightRgbPtr[index].g * (1 - mDepthRatio));
				mp_[index].b = (leftRgbPtr[index].b * mDepthRatio + rightRgbPtr[index].b * (1 - mDepthRatio));
			}
			else
			{
				if (is_color_height_max)
				{
					mp_[index].r = rightRgbPtr[index].r;
					mp_[index].g = rightRgbPtr[index].g;
					mp_[index].b = rightRgbPtr[index].b;
				}
				else
				{
					mp_[index].r = leftRgbPtr[index].r;
					mp_[index].g = leftRgbPtr[index].g;
					mp_[index].b = leftRgbPtr[index].b;
				}
			}
		}
	}

	mapping_data.mappingPtr.UpdateData(mapping_tmp, mapping_data.depthPtr.width, h_max);

	if (nullptr != mapping_tmp)
	{
		delete mapping_tmp;
		mapping_tmp = nullptr;
	}
}
void Calc::setDrawModel(DepthDrawModel draw_mode)
{
	mDeptMode = draw_mode;
}

void Calc::setRainbow(int r_value, int r_step)
{
	mRainbowValue = r_value;
	mRainbowStep = r_step;
}

bool Calc::toWorldSpace(const uint16_t * depth_16bit, QVector<OBPoint>& obp_vec, const uint8_t * rgb_888bit
	, bool save_color, int width, int height, float fxy, float cx, float cy)
{
	if (nullptr == depth_16bit) {
		return false;
	}

	obp_vec.clear();

	const uint8_t* rgb_p = nullptr;
	if (save_color)
	{
		rgb_p = (nullptr != rgb_888bit) ? rgb_888bit : wt_.color_data_ptr;
	}

	const RGB_888BIT *color_rgb_ptr = reinterpret_cast<const RGB_888BIT*>(rgb_p);

	for (int h = 0; h < height; ++h)
	{
		for (int w = 0; w < width; ++w)
		{
			uint32_t iIndex = h * width + w;
			int depth_value = depth_16bit[iIndex];

			if (depth_value > 10)
			{
				OBPoint tmp_p;

				tmp_p.Pos.z = depth_value;
				tmp_p.Pos.x = (w*1.0 - cx) / fxy * depth_value;
				tmp_p.Pos.y = (h*1.0 - cy) / fxy * depth_value;

				if (true == save_color && nullptr != rgb_p)
				{
					uint32_t rgb_index = iIndex * 3;
					tmp_p.Color.r = color_rgb_ptr[iIndex].r;
					tmp_p.Color.g = color_rgb_ptr[iIndex].g;
					tmp_p.Color.b = color_rgb_ptr[iIndex].b;
				}

				obp_vec.push_back(tmp_p);
			}
		}
	}

	return true;
}

bool Calc::writePly(const char * ply_name, const float* obp_vec, int size, bool isPly,int w,int h)
{
	ofstream ply_stream;
	ply_stream.open(ply_name, ofstream::out);
	if (!ply_stream)
	{
		qDebug() << __FILE__ << __FUNCTION__ << __LINE__ << "ofstream open fail";
		return -1;
	}

	uint32_t point_count = size;
	qDebug() << "point count:" << size;
	if (isPly)
	{
		ply_stream << "ply" << std::endl;
		ply_stream << "format ascii 1.0" << std::endl;
		ply_stream << "comment made by Orbbec " << std::endl;
		ply_stream << "comment Orbbec Co.,Ltd." << std::endl;

		ply_stream << "element vertex " << w * h << std::endl;

		ply_stream << "property float32 x" << std::endl;
		ply_stream << "property float32 y" << std::endl;
		ply_stream << "property float32 z" << std::endl;
		if (false)
		{
			ply_stream << "property uint8 red" << std::endl;
			ply_stream << "property uint8 green" << std::endl;
			ply_stream << "property uint8 blue" << std::endl;
		}
		ply_stream << "end_header" << std::endl;
	}
	const float * pData = obp_vec;
	float x, y, z;
	std::string write_buf;
	char line[50] = { 0 };

	for (uint32_t i = 0; i < w * h; i++)
	{
		x = pData[0];
		y = pData[1];
		z = pData[2];
		memset(line, 0, 50);
		sprintf(line, "%.3f %.3f %.0f\n", x, y, z);
		// ply_stream << x << " " << y << " " << z;

		write_buf += line;
		pData += 6;
	}
	ply_stream.write(write_buf.c_str(), write_buf.length());
	ply_stream.close();

	return true;
}

QVector<OBPoint> Calc::calcPointCloud(WorldTransform wt)
{
	QVector<OBPoint> pc_vec;

	if (nullptr == wt.depthDataPtr || false == interiParamValid()) {
		return pc_vec;
	}
	mQmutexCalc.lock();
	wt_ = wt;
	mQmutexCalc.unlock();

	RGB_888BIT *color_rgb_ptr = reinterpret_cast<RGB_888BIT*>(wt.color_data_ptr);
	for (decltype(wt.height) h = 0; h < wt.height; ++h)
	{
		for (decltype(wt.width) w = 0; w < wt.width; ++w)
		{
			uint32_t iIndex = h * wt.width + w;
			int iDepthValue = wt.depthDataPtr[iIndex];

			if (iDepthValue > 10 && iDepthValue < mMaxDepth)
			{
				OBPoint tmpData;

				tmpData.Pos.z = iDepthValue;
				tmpData.Pos.x = (w*1.0 - cx_) / fx_ * iDepthValue;
				tmpData.Pos.y = (h*1.0 - cy_) / fx_ * iDepthValue;

				if (color_rgb_ptr != nullptr)
				{
					tmpData.Color.r = color_rgb_ptr[iIndex].r;
					tmpData.Color.g = color_rgb_ptr[iIndex].g;
					tmpData.Color.b = color_rgb_ptr[iIndex].b;
				}
				else if (wt.model == DepthDrawModel::NONE)
				{
					uchar ucValue = 0;
					tmpData.Color.r = ucValue;
					tmpData.Color.g = ucValue;
					tmpData.Color.b = ucValue;
				}

				OBPoint point_normalization = tmpData;
				point_normalization.Pos.z = tmpData.Pos.z - mCoordDiff;
				point_normalization.Pos.x /= kPcScale;
				point_normalization.Pos.y /= kPcScale;
				point_normalization.Pos.z /= kPcScale;
				point_normalization.Color.r /= 255;
				point_normalization.Color.g /= 255;
				point_normalization.Color.b /= 255;

				pc_vec.push_back(point_normalization);
			}
		}
	}

	return pc_vec;
}

std::vector<std::string> Calc::stringSplit(std::string strtem, char a)
{
	std::vector<std::string> strvec;

	std::string::size_type pos1, pos2;
	pos2 = strtem.find(a);
	pos1 = 0;
	while (std::string::npos != pos2)
	{
		strvec.push_back(strtem.substr(pos1, pos2 - pos1));

		pos1 = pos2 + 1;
		pos2 = strtem.find(a, pos1);
	}
	strvec.push_back(strtem.substr(pos1));
	return strvec;
}

double Calc::correl(std::vector<double>& A, std::vector<double>& B)
{
	double res = 0.0;
	if (A.empty() || B.empty() || A.size() != B.size())
		return res;

	int length = A.size();
	double sumA, sumB, sumAB;
	sumA = sumB = sumAB = 0.0;

	double square_sum_x = 0.0;
	double square_sum_y = 0.0;

	//求和
	sumA = std::accumulate(A.begin(), A.end(), 0.0);
	sumB = std::accumulate(B.begin(), B.end(), 0.0);
	for (int i = 0; i < length; ++i)
	{
		sumAB += A[i] * B[i];
		square_sum_x += A[i] * A[i];
		square_sum_y += B[i] * B[i];
	}

	double corr = (length * sumAB - sumA * sumB) /
		sqrt((length * square_sum_x - sumA * sumA) * (length*square_sum_y - sumB * sumB));

	return corr;
}

void Calc::ThreadLoadPly()
{
	mLoadPlyFinishFlag = false;

	// 转换，支持中文
	QByteArray ba = mPlyFilename.toLocal8Bit();
	const char* c_file_name = ba.data();

	fstream fopen;
	fopen.open(c_file_name, ios::in);
	if (false == fopen.is_open())
	{
		mLoadPlyFinishFlag = true;
		return;
	}

	QVector<OBPoint> pc_vec;
	std::string line_str;
	while (!fopen.eof())
	{
		getline(fopen, line_str, '\n');
		if (0 == line_str.compare("end_header"))
		{
			while (!fopen.eof())
			{
				OBPoint tmpData;

				getline(fopen, line_str, '\n');

				std::vector<std::string> vecStrSplit = stringSplit(line_str, ' ');
				if (6 == vecStrSplit.size())
				{
					tmpData.Pos.x = atof(vecStrSplit[0].c_str()) / kPcScale;
					tmpData.Pos.y = atof(vecStrSplit[1].c_str()) / kPcScale;
					tmpData.Pos.z = atof(vecStrSplit[2].c_str()) / kPcScale;
					tmpData.Color.r = atof(vecStrSplit[3].c_str()) / 255;
					tmpData.Color.g = atof(vecStrSplit[4].c_str()) / 255;
					tmpData.Color.b = atof(vecStrSplit[5].c_str()) / 255;
					pc_vec.push_back(tmpData);
				}
				else if (3 == vecStrSplit.size())
				{
					tmpData.Pos.x = atof(vecStrSplit[0].c_str()) / kPcScale;
					tmpData.Pos.y = atof(vecStrSplit[1].c_str()) / kPcScale;
					tmpData.Pos.z = atof(vecStrSplit[2].c_str()) / kPcScale;
					uchar ucValue = (uchar)((uint32_t)tmpData.Pos.z >> 2);
					tmpData.Color.r = ucValue / 255;
					tmpData.Color.g = ucValue / 255;
					tmpData.Color.b = ucValue / 255;
					pc_vec.push_back(tmpData);
				}
			}
			break;
		}
	}
	mLoadPlyFinishFlag = true;
}
bool Calc::updateD2CFunc(OniData& oni_data)
{
	if (nullptr == oni_data.oData.mDataPtr) {
		return false;
	}

	uint32_t size_tmp = oni_data.mDataSizeBytes / 2 * 3 * sizeof(uint8_t);

	if (size_tmp != oni_data.oData.SizeOfD2C888bit())
	{
		oni_data.oData.ObDeleteD2C888bit();
		oni_data.oData.SetD2C888bitSize(size_tmp);

		oni_data.oData.mDataD2c888Ptr = new int8_t[size_tmp]();
	}

	if (nullptr == oni_data.oData.mDataD2c888Ptr)
	{
		oni_data.oData.SetD2C888bitSize(0);
		return false;
	}

	for (int h = 0; h < oni_data.mHeight; h++)
	{
		for (int w = 0; w < oni_data.mWidth; w++)
		{
			auto depth_index = h * oni_data.mWidth + w;
			auto rgb_index = depth_index * 3;

			uint32_t depth_value = reinterpret_cast<uint16_t*>(oni_data.oData.mDataPtr)[depth_index];

			if (depth_value > 1 && depth_value < mD2cRenderDistance)
			{
				uchar nOffsetIndex = (uchar)(depth_value * 255 / mD2cRenderDistance);
				oni_data.oData.mDataD2c888Ptr[rgb_index + 0] = mPalletIntsRed[nOffsetIndex];
				oni_data.oData.mDataD2c888Ptr[rgb_index + 1] = mPalletIntsGreen[nOffsetIndex];
				oni_data.oData.mDataD2c888Ptr[rgb_index + 2] = mPalletIntsBlue[nOffsetIndex];
			}
			else
			{
				oni_data.oData.mDataD2c888Ptr[rgb_index + 0] = 255;
				oni_data.oData.mDataD2c888Ptr[rgb_index + 1] = 255;
				oni_data.oData.mDataD2c888Ptr[rgb_index + 2] = 255;
			}
		}
	}

	if (size_tmp != oni_data.oData.SizeOfD2C888bit())
		oni_data.oData.SetD2C888bitSize(size_tmp);

	return true;
}
bool Calc::rainbowFunc(OniData& oni_data)
{
	if (nullptr == oni_data.oData.mDataPtr) {
		return false;
	}

	uint32_t size_tmp = oni_data.mDataSizeBytes / 2 * 3 * sizeof(uint8_t);

	if (size_tmp != oni_data.oData.SizeOfRainbow888bit())
	{
		oni_data.oData.ObDeleteRainbow888bit();
		oni_data.oData.SetRainbow888bitSize(size_tmp);

		oni_data.oData.mDataRainbow888Ptr = new int8_t[size_tmp]();
	}

	if (nullptr == oni_data.oData.mDataRainbow888Ptr)
	{
		oni_data.oData.SetRainbow888bitSize(0);
		return false;
	}
#if 1
	for (int h = 0; h < oni_data.mHeight; h++)
	{
		for (int w = 0; w < oni_data.mWidth; w++)
		{
			auto depth_index = h * oni_data.mWidth + w;
			auto rgb_index = depth_index * 3;
			if ((rgb_index + 3) < size_tmp)
			{
				uint16_t depth_value = reinterpret_cast<uint16_t*>(oni_data.oData.mDataPtr)[depth_index];
				if (depth_value >= mMaxDepthColorDistance || depth_value <= mMinDepthColorDistance)
				{
					oni_data.oData.mDataRainbow888Ptr[rgb_index + 0] = 0;
					oni_data.oData.mDataRainbow888Ptr[rgb_index + 1] = 0;
					oni_data.oData.mDataRainbow888Ptr[rgb_index + 2] = 0;
				}
				else
				{
					uchar nOffsetIndex = (uchar)(((depth_value - mMinDepthColorDistance) % mMaxDepthColorDistance) * 255 / (mMaxDepthColorDistance - mMinDepthColorDistance));
					//uchar nOffsetIndex = (uchar)(((depth_value - mMinDepthColorDistance) * 255) / (mMaxDepthColorDistance - mMinDepthColorDistance));
					if (nOffsetIndex < 256)
					{
						oni_data.oData.mDataRainbow888Ptr[rgb_index + 0] = mPalletIntsRed[nOffsetIndex];
						oni_data.oData.mDataRainbow888Ptr[rgb_index + 1] = mPalletIntsGreen[nOffsetIndex];
						oni_data.oData.mDataRainbow888Ptr[rgb_index + 2] = mPalletIntsBlue[nOffsetIndex];
					}
				}
			}
		}
	}
#endif
	if (size_tmp != oni_data.oData.SizeOfRainbow888bit())
		oni_data.oData.SetRainbow888bitSize(size_tmp);

	return true;
}
void Calc::setD2CDistance(int distance) {
	mD2cRenderDistance = distance;
}

double Calc::interpolate(double x, double x0, double y0, double x1, double y1) {

	if (x1 == x0) {
		return y0;
	}
	else {
		return ((x - x0)*(y1 - y0) / (x1 - x0) + y0);
	}
}
void Calc::createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue) {

	double k = 1;
	double B0 = -0.125 * k - 0.25;
	double B1 = B0 + 0.25 * k;
	double B2 = B1 + 0.25 * k;
	double B3 = B2 + 0.25 * k;

	double G0 = B1;
	double G1 = G0 + 0.25 * k;
	double G2 = G1 + 0.25 * k;
	double G3 = G2 + 0.25 * k + 0.125;

	double R0 = B2;
	double R1 = R0 + 0.25 * k;
	double R2 = R1 + 0.25 * k;
	double R3 = R2 + 0.25 * k + 0.25;

	double i = (double)indx / (double)numSteps - 0.25 * k;

	if (i >= R0 && i < R1) {
		red = interpolate(i, R0, 0, R1, 255);
	}
	else if ((i >= R1) && (i < R2)) {
		red = 255;
	}
	else if ((i >= R2) && (i < R3)) {
		red = interpolate(i, R2, 255, R3, 0);
	}
	else {
		red = 0;
	}

	if (i >= G0 && i < G1) {
		green = interpolate(i, G0, 0, G1, 255);
	}
	else if ((i >= G1) && (i < G2)) {
		green = 255;
	}
	else if ((i >= G2) && (i < G3)) {
		green = interpolate(i, G2, 255, G3, 0);
	}
	else {
		green = 0;
	}


	if (i >= B0 && i < B1) {
		blue = interpolate(i, B0, 0, B1, 255);
	}
	else if ((i >= B1) && (i < B2)) {
		blue = 255;
	}
	else if ((i >= B2) && (i < B3)) {
		blue = interpolate(i, B2, 255, B3, 0);
	}
	else {
		blue = 0;
	}

}

void Calc::setMaxDepthColorDistance(int value)
{
	mMaxDepthColorDistance = value;
}
void Calc::setMinDepthColorDistance(int value)
{
	mMinDepthColorDistance = value;
}