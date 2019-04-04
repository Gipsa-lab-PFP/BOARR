#ifndef DEF_UTIL
#define DEF_UTIL

#include <ros/ros.h>
#include <string>

class Quaternion
{
	public:
	double w;
	double x;
	double y;
	double z;
	void fromYawPitchRoll(double yaw, double pitch, double roll);
	double toRoll();	
	double toPitch();  
	double toYaw();
	void toRollPitchYaw(double* roll, double*pitch, double*yaw);
    void operator=( Quaternion q );
    Quaternion(double x, double y, double z, double w);
    Quaternion();
    void rotate(Quaternion q);
};

class Cart
{
    public:
    double x;
    double y;
    double z;
    void operator=( Cart point );
    Cart();
    Cart(double x, double y, double z);
};

class Spheric
{
    public :
    double rho;
    double theta; 
    double phi;
    void operator=( Spheric point );
};

enum Point_3D_Type { CART, SPHERIC };

class Point_3D 
{
    private :
    Cart cart;
    Spheric spheric;
    void from_cart_to_spheric();
    void from_spheric_to_cart();

    public:
    void x(double a);
    void y(double a);
    void z(double a);
    void rho(double a);
    void theta(double a);
    void phi(double phi);
    Point_3D();
    Point_3D(Point_3D_Type my_type, double a, double b, double c);
    void define(Point_3D_Type my_type, double a, double b, double c);
    double x(){return this->cart.x;};
    double y(){return this->cart.y;};
    double z(){return this->cart.z;};
    double rho(){return this->spheric.rho;};
    double theta(){return this->spheric.theta;};
    double phi(){return this->spheric.phi;};
    double dist_to_point(Point_3D point);
    void operator=( Point_3D point );
    void operator=( Cart cart );
    void rotate(Quaternion q);
    void rotateInverse( Quaternion q);
};

class Twist
{
    public:
    Cart linear;
    Cart angular;
    void operator=( Twist twist );
};

class Pose
{
    public:
        Cart position;
        Quaternion quaternion;
        void operator=( Pose pose );
};

class Odometry
{
	public:
        Pose pose;
        Twist twist;
        ros::Time stamp;
};
#endif
