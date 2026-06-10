#pragma once
#include <iostream>
#include <vector>
#include <set>
#include <cmath>
#include <thread>
#include <chrono>
#include <omp.h>
#include <opencv2/opencv.hpp>
#include "../PLY/PlyIO.h"
#include "../SinglePoint/singlePoint.h"

#include <tuple>
#include <boost/variant.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/point_xyz.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/foreach.hpp>
#include <bullet/btBulletCollisionCommon.h>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;
typedef bg::model::point<double, 2, bg::cs::cartesian> DPoint2;
typedef bg::model::point<double, 3, bg::cs::cartesian> DPoint3;
typedef bg::model::box<DPoint3> DBox;
typedef bg::model::box<DPoint3> DExpandBox;
typedef bg::model::box<DPoint2> DImageBox;
typedef bg::model::segment<DPoint3> DLine;
typedef bg::model::segment<DPoint2> DImageLine;
typedef std::tuple < DBox, DImageBox, int, int, DExpandBox, set<int>> DTree;

typedef struct
{
	IMGPOINT pt1;
	IMGPOINT pt2;
}LINE2D;

typedef struct
{
	STRUCTUREPOINT Point1;
	STRUCTUREPOINT Point2;
	IMGPOINT pt1;
	IMGPOINT pt2;
	set<int>viewindex;
}LINE3D;

class CLineCloud
{
public:
	CLineCloud();
	~CLineCloud();
	int m_LineErrorThres = 5;							//共线阈值
	double angle_ErrorThre = 15.0;						//角度阈值
	std::vector<std::vector<std::string>> timagesStrArr;
	std::vector<std::vector<LINE3D>> LineCloudArr;
	int filenum = 1;									//初始节点个数
	bgi::rtree< DTree, bgi::rstar<16, 4> > rtree;		//全局R树

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	btCollisionWorld* world =
		new btCollisionWorld(dispatcher, overlappingPairCache, collisionConfiguration);	//全局世界

	void vmInitialNeighbourImg(const char* imgNeighPath);
	void vmGenLineCloud(SinglePoint& singleTask, string outputPath, int flag,int cur);
	double vmGetObjPtResolution(Vec3f& objPt, int photoID);
	double vmConvertObjPt2Depth(Vec3f& objPt, int photoID);
	void vmTransformPointW2C(int photoID, Vec3f& camera_center, Vec3f& pt_c, Vec3f& pt_w);
	void verifyLineStructure(int photoID, STRUCTUREPOINT& singlePt1, STRUCTUREPOINT& singlePt2, STRUCTUREPOINT& singlePt_m, std::vector<LINE3D>& verifiedLines);
	double angle_between_vectors2(LINE3D vector1, LINE3D vector2);
	double angle_between_vectors(const DPoint3& vector1, const DPoint3& vector2);

	void atsLoadLineCloud(const std::string& filePath);
private:
	void genOneImageLines(SinglePoint& singleTask, std::vector<LINE3D>& Line3DArr, int photoID, string outputPath, int flag);
	void readImageLines(const char* imgLinePath, std::vector<LINE2D>& imageLineArr);
	/*void trimRtree();
	void trimRtree2();*/
	double computeLength(LINE3D line);
	double computeOverlap(LINE3D line, LINE3D maxLengthline);
	void outputVisualInformation(string outputPath);


	string OutputBCDClusterLines(string outputPath, int flag);
	string OutputForwardClusterLines(string outputPath, int flag);
	void resultBCDCluster(string outputPath, int flag);
	void resultForwardCluster(string outputPath, int flag);
	void OutputNoClusterLinesPart(string outputPath, int flag, std::vector<LINE3D> Line3DArr);
	string OutputNoClusterLines(string outputPath, int flag);

	void outputSubPly(string outputPath, SinglePoint& singleTask, std::vector<LINE3D>& Line3DArr, int flag);
	void outputPly(string outputPath, int flag);
};
extern CameraPara m_frameCamera;
extern std::vector<IMGEO> m_EOArray;
extern bgi::rtree< DTree, bgi::rstar<16, 4> > rtree;
