#include <iostream>
#include "util.h"
#include <cmath>
#include <string>

using namespace std;

void Quaternion::fromYawPitchRoll(double yaw, double pitch, double roll )
{
	double t0 = std::cos(yaw * 0.5f);
	double t1 = std::sin(yaw * 0.5f);
	double t2 = std::cos(roll * 0.5f);
	double t3 = std::sin(roll * 0.5f);
	double t4 = std::cos(pitch * 0.5f);
	double t5 = std::sin(pitch * 0.5f);

	this->w = t0 * t2 * t4 + t1 * t3 * t5;
	this->x = t0 * t3 * t4 - t1 * t2 * t5;
	this->y = t0 * t2 * t5 + t1 * t3 * t4;
	this->z = t1 * t2 * t4 - t0 * t3 * t5;
	return; 
}


double Quaternion::toRoll()
{
  return atan2(2*this->x*this->w + 2*this->y*this->z, 1 - 2*this->x*this->x - 2*this->y*this->y);
}

double Quaternion::toPitch()
{
 	double t2 = 2.0f * (this->w * this->y - this->z * this->x);
    double pitch = std::asin(t2);
	return pitch;
}

double Quaternion::toYaw()
{
   return atan2(2.0 * (this->w * this->z + this->x * this->y),
                1.0 - 2.0 * (this->y * this->y + this->z * this->z));
}

void Quaternion::toRollPitchYaw(double &roll, double &pitch, double &yaw)
{
    roll =  atan2(2*this->x*this->w + 2*this->y*this->z, 1 - 2*this->x*this->x - 2*this->y*this->y);

    double t2 = +2.0f * (this->w * this->y - this->z * this->x);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    pitch = std::asin(t2);

    yaw =  atan2(2.0 * (this->w * this->z + this->x * this->y),
            1.0 - 2.0 * (this->y * this->y + this->z * this->z));

}



void Point_3D::from_spheric_to_cart()
{
    this->cart.x = this->spheric.rho * cos(this->spheric.theta)*cos(this->spheric.phi);
    this->cart.y = this->spheric.rho * sin(this->spheric.theta)*cos(this->spheric.phi);
    this->cart.z = this->spheric.rho * sin(this->spheric.phi);
    return;
}


void Point_3D::from_cart_to_spheric()
{
    this->spheric.rho = sqrt(this->cart.x*this->cart.x + this->cart.y*this->cart.y + this->cart.z*this->cart.z);
    this->spheric.theta = atan2(this->cart.y,this->cart.x);
    this->spheric.phi = M_PI/2 - acos(this->cart.z/this->spheric.rho);
}

void Point_3D::x(double x)
{
    this->cart.x = x; 
    this->from_cart_to_spheric();
}
void Point_3D::y(double y)
{
    this->cart.y = y; 
    this->from_cart_to_spheric();
}
void Point_3D::z(double z)
{
    this->cart.z = z; 
    this->from_cart_to_spheric();
}
void Point_3D::rho(double rho)
{
    this->spheric.rho = rho; 
    this->from_spheric_to_cart();
}
void Point_3D::theta(double theta)
{
    this->spheric.theta = theta; 
    this->from_spheric_to_cart();
}
void Point_3D::phi(double phi)
{
    this->spheric.phi = phi; 
    this->from_spheric_to_cart();
}

Point_3D::Point_3D()
{
    cart.x = 0;
    cart.y = 0;
    cart.z = 0;
    spheric.rho = 0;
    spheric.theta =0 ;
    spheric.phi =0;
}

Point_3D::Point_3D(Point_3D_Type my_type, double a, double b, double c)
{
    if (my_type ==  SPHERIC)
    {
        this->spheric.rho = a;
        this->spheric.theta=b;
        this->spheric.phi = c;
        this->from_spheric_to_cart();
    }
    else if ( my_type ==  CART) 
    {
        this->cart.x=a;
        this->cart.y=b;
        this->cart.z=c;
        this->from_cart_to_spheric();
    }
    else
    {
        cout << " Point_3D unknow type lots of errors incoming" << endl;
    }
}

void Point_3D::define(Point_3D_Type my_type, double a, double b, double c)
{
    if (my_type ==  SPHERIC)
    {
        this->spheric.rho = a;
        this->spheric.theta=b;
        this->spheric.phi = c ;
        this->from_spheric_to_cart();
        return;
    }
    else if ( my_type ==  CART) 
    {
        this->cart.x =a;
        this->cart.y=b;
        this->cart.z=c;
        this->from_cart_to_spheric();
        return;
    }
    else
    {
        cout << " Point_3D unknow type lots of errors incoming" << endl;
    }
}

double Point_3D::dist_to_point(Point_3D point)
{
    double temp = ( this->x() - point.x() ) * ( this->x() - point.x() ) +  ( this->y() - point.y() ) * ( this->y() - point.y() )
        +   ( this->z() - point.z() ) * ( this->z() - point.z() ) ;
    return sqrt(temp);
}

void Point_3D::operator=( Point_3D point )
{
    this->cart = point.cart;
    this->spheric = point.spheric;
    return;
}

void Point_3D::operator=( Cart cart )
{
    this->cart = cart;
    this->from_cart_to_spheric();
    return;
}

void Point_3D::rotate( Quaternion q)
{
    double t2 =   q.w*q.x;
    double t3 =   q.w*q.y;
    double t4 =   q.w*q.z;
    double t5 =  -q.x*q.x;
    double t6 =   q.x*q.y;
    double t7 =   q.x*q.z;
    double t8 =  -q.y*q.y;
    double t9 =   q.y*q.z;
    double t10 = -q.z*q.z;
    double xnew = 2*( (t8 + t10)*this->cart.x + (t6 -  t4)*this->cart.y + (t3 + t7)*this->cart.z ) + this->cart.x;
    double ynew = 2*( (t4 +  t6)*this->cart.x + (t5 + t10)*this->cart.y + (t9 - t2)*this->cart.z ) + this->cart.y;
    double znew = 2*( (t7 -  t3)*this->cart.x + (t2 +  t9)*this->cart.y + (t5 + t8)*this->cart.z ) + this->cart.z;
    this->cart.x = xnew;
    this->cart.y = ynew;
    this->cart.z = znew;
    this->from_cart_to_spheric();
}

void Point_3D::rotateInverse(Quaternion q)
{
    double x = -q.x;
    double y = -q.y ;
    double z = -q.z;
    double t2 =   q.w*x;
    double t3 =   q.w*y;
    double t4 =   q.w*z;
    double t5 =  -x*x;
    double t6 =   x*y;
    double t7 =   x*z;
    double t8 =  -y*y;
    double t9 =   y*z;
    double t10 = -z*z;
    double xnew = 2*( (t8 + t10)*this->cart.x + (t6 -  t4)*this->cart.y + (t3 + t7)*this->cart.z ) + this->cart.x;
    double ynew = 2*( (t4 +  t6)*this->cart.x + (t5 + t10)*this->cart.y + (t9 - t2)*this->cart.z ) + this->cart.y;
    double znew = 2*( (t7 -  t3)*this->cart.x + (t2 +  t9)*this->cart.y + (t5 + t8)*this->cart.z ) + this->cart.z;
    this->cart.x = xnew;
    this->cart.y = ynew;
    this->cart.z = znew;
    this->from_cart_to_spheric();
}

void Point_3D::translate(double x, double y, double z )
{
    this->cart.x += x;
    this->cart.y += y;
    this->cart.z += z;
    this->from_cart_to_spheric();
}

void Cart::operator=( Cart cart )
{
    this->x = cart.x;
    this->y = cart.y;
    this->z = cart.z;
}

void Spheric::operator=( Spheric spheric )
{
    this->rho = spheric.rho;
    this->theta = spheric.theta;
    this->phi = spheric.phi;
}

void Quaternion::operator=( Quaternion q )
{
    this->w = q.w;
    this->x = q.x;
    this->y = q.y;
    this->z = q.z;
}


void Twist::operator=( Twist twist )
{
    this->linear  =  twist.linear;
    this->angular = twist.angular;
}


void Pose::operator=( Pose pose )
{
    this->position = pose.position;
    this->quaternion = pose.quaternion;
}


Quaternion::Quaternion(double x, double y, double z, double w)
{
    this->x = x;
    this->y = y;
    this->z = z;
    this->w = w;
}

Quaternion::Quaternion()
{
    this->x = 0;
    this->y = 0;
    this->z = 0;
    this->w = 0;
}

void Quaternion::rotate( Quaternion r)
{
    double old_x = this->x;
    double old_y = this->y;
    double old_z = this->z;
    double old_w = this->w;
    
    this->w = old_w *r.w - old_x * r.x - old_y *r.y - old_z *r.z ;
    this->x = old_x *r.w + old_w * r.x - old_z *r.y + old_y *r.z ;
    this->y = old_y *r.w + old_z * r.x + old_w *r.y - old_x *r.z ;
    this->z = old_z *r.w - old_y * r.x + old_x *r.y + old_w *r.z ;

    return;
}

void Odometry::operator=( Odometry odom )
{
    this->pose = odom.pose;
    this->twist = odom.twist;
    this->acc = odom.acc;
    this->stamp = odom.stamp;
}
