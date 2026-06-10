#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include <assert.h>
#include <float.h>
#include <math.h>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <thread>
#include <chrono>
#include <time.h>

using namespace std;

#include "SinglePoint/singlePoint.h"
#include "LineCloud/LineCloud.h"
#include "Stlplus/file_system.hpp"
#include "LineCluster/LineCluster.h"

int main(int argc, char* argv[])
{
	clock_t start, end;
	cout << "\tstart\n";
	start = clock();
	string imgPath = argv[1];										//影像路径
	string plyPath = argv[2];										//Mesh路径
	string EOPath = argv[3];										//外参路径
	string IOPath = argv[4];										//内参路径
	string neighbourFilePath = argv[5];								//邻域列表文件
	string outputPath = argv[6];									//输出文件路径
	int flag = stoi(argv[7]);										//1代表聚类 0不聚类
	int cur = stoi(argv[8]);										//当前影像序号，便于断点生成

	if (!stlplus::folder_exists(outputPath))
	{
		if (!stlplus::folder_create(outputPath))
		{
			std::cout << "\n\t Cannot create tempPathName directory.";
			return false;
		}
	}

	SinglePoint singleTask;
	singleTask.vmInitialDataSet(imgPath.c_str(), IOPath.c_str(), EOPath.c_str(), plyPath.c_str());
	CLineCloud lineCloud;
	lineCloud.vmInitialNeighbourImg(neighbourFilePath.c_str());
	lineCloud.vmGenLineCloud(singleTask, outputPath, flag, cur);
	end = clock();
	cout << "\tTotal time: " << double(end - start) / CLOCKS_PER_SEC << "s" << endl;
	system("pause");
	return true;
}
