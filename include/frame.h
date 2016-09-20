#ifndef _CQ_FRAME_H_
#define _CQ_FRAME_H_

#include<string>

#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>


using std::string;
using Eigen::Vector3d;
using Eigen::Vector2d;
using cv::Point2d;
using cv::Point3d;
using cv::Mat;
namespace vslam{

class AbstractCamera {
public:
	AbstractCamera() {};
	AbstractCamera(int width, int height) : width_(width), height_(height) {};
	
	virtual ~AbstractCamera() {};

	virtual Vector3d Cam2World(const double &uw, const double &uh) const =0;
	virtual Vector3d Cam2World(const Vector2d &pixel) const =0;
	virtual Vector3d Cam2World(const Point2d &pixel) const =0;


	virtual Vector2d World2Cam(const double &x, const double &y, const double &z) const =0;
	virtual Vector2d World2Cam(const Vector3d &xyz) const =0;

	inline int width() const { return width_; }
	inline int height() const { return height_; }

	virtual bool GetCamParaOrDie(string filename); 

protected:
	int width_;
	int height_;
};

class PinholeCamera : public AbstractCamera {
public:
	PinholeCamera(double width, double height,
				  double fx, double fy, double cx, double cy,
				  double k1 = 0.0, double k2 = 0.0, double p1 = 0.0, double p2 = 0.0, double k3 = 0.0);
	~PinholeCamera();

	virtual bool GetCamParaOrDie(string filename);
	virtual Vector3d Cam2World(const double &uw, const double &uh) const;
	virtual Vector3d Cam2World(const Vector2d &pixel) const;
	virtual Vector3d Cam2World(const Point2d &pixel) const;


	virtual Vector2d World2Cam(const double &x, const double &y, const double &z) const;
	virtual Vector2d World2Cam(const Vector3d &xyz) const;

	inline double fx() const { return fx_; };
	inline double fy() const { return fy_; };
	inline double cx() const { return cx_; };
	inline double cy() const { return cy_; };

private:
	double fx_, fy_, cx_, cy_, d_[5];
	bool is_distorted;
	Mat K_cv_, D_cv_;
	Mat map_[2];
};

class FisheyeCamera : public AbstractCamera {
public:
	virtual bool GetCamParaOrDie(string filename);
	virtual Vector3d Cam2World(const double &uw, const double &uh) const;
	virtual Vector3d Cam2World(const Vector2d &pixel) const;
	virtual Vector3d Cam2World(const Point2d &pixel) const;


	virtual Vector2d World2Cam(const double &x, const double &y, const double &z) const;
	virtual Vector2d World2Cam(const Vector3d &xyz) const;
public:

	double kc_[4];
	double cx_, cy_;
};



class AbstractFrame {


};


}


#endif