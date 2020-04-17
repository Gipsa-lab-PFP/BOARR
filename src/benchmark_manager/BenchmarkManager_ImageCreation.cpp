#include <iostream>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "BenchmarkManager.hpp"
#include "util.hpp"


void BenchmarkManager::createImg()
{
    /*****************Draw a 5k*5k summary view of the test from above ************************/
    //wait for the world callback to initialize
    if ( !_worldCallback_CALLED ) return;
    //initialize only if it hasn't been done before
    if ( _worldCallback_CALLED && _drawingBase.cols != 5000 ) 
    {
        _drawingBase = cv::Mat(5000,5000,CV_8UC3,cv::Scalar(255,255,255));
        initialDrawing(); 
    }

    // At each frame, add data to view from above
    addDataToBaseImg(); 

    // Add data to the map from above if this frame is needed
    if ( _showImg || _saveVideo || _stop )
    {
        // Add data to the specific frame
        addFrameSpecificContent(); 

        /***************** Design an output that's "nice" to look at  ************************/
        // create a &920*1080 image
        cv::Mat finalImg = cv::Mat(1080,1920,CV_8UC3,cv::Scalar(200,200,200));

        // reduce the view from above to 1080*1080 andput it on the left of the final image
        cv::Mat small_plan;
        cv::resize(_currentDrawing, small_plan, cv::Size(1080,1080), 1.0, 1.0);
        small_plan.copyTo(finalImg(cv::Rect(0,0,1080,1080)));

        // Add a view from the looking ahead camera of the drone on the top right
        // Add the target in the camera if it is inside its FOV
        addTargetToCam();
        cv::Mat tmp;
        if ( _currentImg.cols != 0 && _currentImg.rows != 0 )
        {
            double row_max = 700.0;
            if ( 840. / (float) _currentImg.cols < row_max / (float) _currentImg.rows ) 
            {
                cv::resize(_currentImg, tmp, cv::Size(0,0), 840.0/_currentImg.cols,840.0/_currentImg.cols);
                tmp.copyTo(finalImg(cv::Rect(1080,0,tmp.cols,tmp.rows)));
            }
            else
            {
                cv::resize(_currentImg, tmp, cv::Size(0,0), row_max/_currentImg.rows,row_max/_currentImg.rows);
                tmp.copyTo(finalImg(cv::Rect(1080+420-tmp.cols/2.0,0,tmp.cols,tmp.rows)));
            }
        }

        // Add a Recap Table of the BenchmarkManager
        cv::Mat tableImg = cv::Mat(1080-tmp.rows,840,CV_8UC3,cv::Scalar(200,200,200));
        drawTable(tableImg);
        tableImg.copyTo(finalImg(cv::Rect(1080, tmp.rows, tableImg.cols, tableImg.rows)));  

        if( _showImg ) 
        {
            cv::Mat tmp3;
            cv::resize(finalImg, tmp3, cv::Size(1600,900), 0,0);//reduce the image size for display comfort
            cv::imshow("current image", tmp3);
            cv::waitKey(1);
        }
        if ( _saveVideo )
        {
            outputVideo << finalImg;
        }
        if ( _saveImage && _stop )
        {
            cv::imwrite(_imageName, finalImg);
        }
    }
}

cv::Point BenchmarkManager::worldToImage(double world_x, double world_y)
{
    //map the world into the image depending on the chosen scenario
    int img_x, img_y;
    if ( !_perfectTest ) 
    {
        img_x = 2500 - 50 * world_y;
        img_y = 2500 - 50 * world_x;
    }
    else 
    {
        img_x = 2500 - 100 * world_y;
        img_y = 4500 - 100 * world_x;
    }

    cv::Point returnPoint(img_x, img_y);
    return returnPoint;
}


void BenchmarkManager::initialDrawing()
{
    // draw the goals
    if ( _goals.size() > 0 )
    {
        for ( int i = 0; i <(int) _goals.size() ; i++ )
        {
            float world_x = _goals.at(i).x;
            float world_y = _goals.at(i).y;
            draw_cross(_drawingBase, worldToImage(world_x, world_y), cv::Scalar(0,150,0), 70, 10, 1);  
        }
    }

    if ( _perfectTest ) //in case of a perfect test
    {
        // draw all the cylinders
        if ( _modelPos.size() > 0 )
        {
            for (int i = 0; i < (int)_modelPos.size() - 1 ; i++ )
            {
                float world_x = _modelPos.at(i).x;
                float world_y = _modelPos.at(i).y;
                int radius = 0.16*100;
                cv::circle(_drawingBase,  worldToImage(world_x, world_y), radius, cv::Scalar(100,100,100), -1);
            }
        }
        //draw the bounding box of the cylinder spawn rectangle 
        cv::line(_drawingBase, worldToImage(3,-20), worldToImage(3, 20.0), cv::Scalar(0,0,0), 10); 
        cv::line(_drawingBase, worldToImage(3, 20), worldToImage(43, 20), cv::Scalar(0,0,0), 10); 
        cv::line(_drawingBase, worldToImage(43, 20), worldToImage(43 ,-20), cv::Scalar(0,0,0), 10); 
        cv::line(_drawingBase, worldToImage(43,-20), worldToImage(3,-20), cv::Scalar(0,0,0), 10); 


        //draw a scale
        cv::line(_drawingBase, worldToImage(-4, 24), worldToImage(-4,  19), cv::Scalar(100,100,100), 10); 
        cv::line(_drawingBase, worldToImage(-4,  19), worldToImage(-3.7,  19), cv::Scalar(100,100,100), 10); 
        cv::line(_drawingBase, worldToImage(-4,  24), worldToImage(-3.7,  24), cv::Scalar(100,100,100), 10); 
        cv::putText(_drawingBase, "5m", worldToImage(-3.8, 23), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0,0,255), 10 );
    }
    else // for the noisy test
    {
        // draw the trees
        if ( _modelPos.size() > 0 )
        {
            for (int i = 0; i < (int)_modelPos.size() - 1 ; i++ )
            {
                float world_x = _modelPos.at(i).x;
                float world_y = _modelPos.at(i).y;
                int radius = 0.30*100;
                cv::circle(_drawingBase,  worldToImage(world_x, world_y), radius, cv::Scalar(100,100,100), -1);
            }
        }

        //draw the bounding box of the cylinder spawn rectangle 
        cv::line(_drawingBase, worldToImage(-41,-41), worldToImage(-41, 41.0), cv::Scalar(0,0,0), 10); 
        cv::line(_drawingBase, worldToImage(-41, 41), worldToImage(41, 41), cv::Scalar(0,0,0), 10); 
        cv::line(_drawingBase, worldToImage(41, 41), worldToImage(41 ,-41), cv::Scalar(0,0,0), 10); 
        cv::line(_drawingBase, worldToImage(41,-41), worldToImage(-41,-41), cv::Scalar(0,0,0), 10); 

        //draw a scale
        cv::line(_drawingBase, worldToImage(-48, 40), worldToImage(-48,  30), cv::Scalar(100,100,100), 10); 
        cv::line(_drawingBase, worldToImage(-48,  40), worldToImage(-47,  40), cv::Scalar(100,100,100), 10); 
        cv::line(_drawingBase, worldToImage(-48,  30), worldToImage(-47,  30), cv::Scalar(100,100,100), 10); 
        cv::putText(_drawingBase, "10m", worldToImage(-47, 39), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0,0,255), 10 );
    }
}

// draw a cross in a square of length side size. If orientation == 0, N->S + E->W cross; If orientation != 0, NE->SW + NW->SE cross
void BenchmarkManager::draw_cross(cv::Mat &img, cv::Point position, cv::Scalar Color, int length, int width, int orientation )
{
    if ( orientation == 0 )
    {
        cv::line(img, cv::Point(position.x - length/2.0, position.y ), cv::Point(position.x + length/2.0, position.y ), Color,width); 
        cv::line(img, cv::Point(position.x , position.y + length/2.0), cv::Point(position.x , position.y - length/2.0), Color,width); 
    }
    else
    {
        cv::line(img, cv::Point(position.x - length/2.0, position.y - length/2.0), cv::Point(position.x + length/2.0, position.y + length/2.0), Color,width); 
        cv::line(img, cv::Point(position.x - length/2.0, position.y + length/2.0), cv::Point(position.x + length/2.0, position.y - length/2.0), Color,width); 
    }
}

void BenchmarkManager::addDataToBaseImg()
{
    //draw the drone positions
    if ( _odomVector.size() > 0 )
    {
        for (int i = 0; i < _odomVector.size() ; i++ )
        {
            if ( lastDroneDrownStamp.toSec() + 1.0 <= _odomVector.at(i).stamp.toSec() ) //every 1 sec, draw "shadow drones" in order to have a feel of its speed
            {
                float world_x = _odomVector.at(i).pose.position.x;
                float world_y = _odomVector.at(i).pose.position.y;
                if( _perfectTest )
                {
                    cv::circle(_drawingBase, worldToImage(world_x, world_y), 28, cv::Scalar(100,100,100), -1 );//real size representation of the drone
                }
                else
                {
                    cv::circle(_drawingBase, worldToImage(world_x, world_y), 50, cv::Scalar(220,220,220), -1 );//not real size representation of the drone
                }
                lastDroneDrownStamp = _odomVector.at(i).stamp;
            }
        }

        for (int i = 0; i < _odomVector.size() ; i++ )// draw a point at each position
        {
                float world_x = _odomVector.at(i).pose.position.x;
                float world_y = _odomVector.at(i).pose.position.y;
                cv::circle(_drawingBase, worldToImage(world_x, world_y), 10, cv::Scalar(0,255,0), -1 );
        }
    }

    //keep only the last 5s of odometry in order to draw less points points;
    //keep a smaller buffer will accelerate the process but often the precise path wont be visible on the image, hidden behind the "shadow drones" drown every 2s
    if ( _odomVector.size() > 501 ) _odomVector.erase(_odomVector.begin(), _odomVector.end() - 500);

    //draw the collisions, redraw all the collisions all the time in order to be sure that they are above everything else (especially the shadow drones)
    //not that costly, there shouldn't be that many collisions or the tested algorithm has other issues than realtime capabilities due to the benchmark
    if ( _collisions.size() > 0 ) 
    {
        for (int i = 0; i < (int)_collisions.size() ; i++ ) 
        {
            float collision_x = _collisions.at(i).x;
            float collision_y = _collisions.at(i).y;
            draw_cross(_drawingBase,  worldToImage(collision_x, collision_y), cv::Scalar(0,0,255), 70, 10 ,1);
        }
    }
}

void BenchmarkManager::addFrameSpecificContent()
{
    //Copy _drawingBase On _currentDrawing
    _drawingBase.copyTo(_currentDrawing);

    //Draw a circle arround the current target and write "target" above it in red
    cv::circle(_currentDrawing, worldToImage(_goals.at(_currentGoalNumber).x, _goals.at(_currentGoalNumber).y), 85, cv::Scalar(0,0,255), 20);
    if( _perfectTest )
    {
        cv::putText(_currentDrawing, "Target", worldToImage(_goals.at(_currentGoalNumber).x + 1.5, _goals.at(_currentGoalNumber).y + 3), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0,0,255), 10 );
    }
    else
    {
        cv::putText(_currentDrawing, "Target", worldToImage(_goals.at(_currentGoalNumber).x + 3, _goals.at(_currentGoalNumber).y + 6), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0,0,255), 10 );
    }

    //If possible, draw a circle representing the drone on it's current position and a line to represent it's yaw
    if ( _odomVector.size() == 0 )
    {
        return;
    }
    
    float world_x = _odomVector.at(_odomVector.size()-1).pose.position.x;
    float world_y = _odomVector.at(_odomVector.size()-1).pose.position.y;
    if( _perfectTest )
    {
        cv::circle(_currentDrawing, worldToImage(world_x, world_y), 28, cv::Scalar(50,50,50), -1 );//real size representation of the drone
    }
    else
    {
        cv::circle(_currentDrawing, worldToImage(world_x, world_y), 50, cv::Scalar(150,150,150), -1 );//not real size representation of the drone
    }
    
    Point_3D x1_BF(CART, 1,0,0);
    x1_BF.rotate(_odomVector.at(_odomVector.size()-1).pose.quaternion);
    float x1_x_EF = world_x+x1_BF.x(); 
    float x1_y_EF = world_y+x1_BF.y();
    cv::line(_currentDrawing, worldToImage(world_x, world_y),worldToImage(x1_x_EF, x1_y_EF), cv::Scalar(0,0,255), 9 );

    //add the speed, on the bottom right
    double vel = std::sqrt( _odomVector.at(_odomVector.size()-1).twist.linear.x * _odomVector.at(_odomVector.size()-1).twist.linear.x
                          + _odomVector.at(_odomVector.size()-1).twist.linear.y * _odomVector.at(_odomVector.size()-1).twist.linear.y
                          + _odomVector.at(_odomVector.size()-1).twist.linear.z * _odomVector.at(_odomVector.size()-1).twist.linear.z);
    char ctmp [50];
    std::sprintf(ctmp,"%3.2f",vel); 
    std::string speedString = "|v| = ";
    speedString.append(ctmp);
    speedString.append(" m/s");
    if( _perfectTest ) 
    {
        cv::putText(_currentDrawing, speedString, worldToImage(-3.8, -10), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0,0,0), 10 );
    }
    else
    {   
        cv::putText(_currentDrawing, speedString, worldToImage(-47, -20), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(0,0,0), 10 );
    }
}

void BenchmarkManager::drawTable(cv::Mat &img)
{    /***************** Add some informations on the output Image ************************/
    // the number of collisions
    if ( _collisionNumber == 0 )
    {
        std::string collosionText = "No Collision";
        cv::putText(img, collosionText, cv::Point(100,img.rows-50),  cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,150,0), 4 );
    }
    else if ( _collisionNumber == 1 ) 
    {
        std::string collosionText = "1 Collision";
        cv::putText(img, collosionText, cv::Point(100,img.rows-50),  cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,220), 4 );
    }
    else
    {
        std::string collision_number_string = std::to_string(_collisionNumber);
        std::string collision_text = " Collisions";
        std::string full_collision_text = collision_number_string + collision_text;
        cv::putText(img, full_collision_text, cv::Point(100,img.rows-50),  cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,220), 4 );
    }
    
    //the headline of the table
    double totalDistToTravel = 1000.;
//     Point_3D initialPos(CART, _initialHoverPos.x, _initialHoverPos.y, _initialHoverPos.z);
//     Point_3D firstWaypoint(CART, _goals.at(0).x, _goals.at(0).y, _goals.at(0).z);
//     totalDistToTravel += initialPos.dist_to_point(firstWaypoint);
//     int id = 1;
//     while ( id < _goals.size() )
//     {
//             Point_3D idWaypoint(CART, _goals.at(id).x, _goals.at(id).y, _goals.at(id).z);
//             Point_3D idMinusOneWaypoint(CART, _goals.at(id-1).x, _goals.at(id-1).y, _goals.at(id-1).z);
//             totalDistToTravel += idWaypoint.dist_to_point(idMinusOneWaypoint);
//             id ++;
//     }
    if ( !_stop)
    {
        std::string completionPercentage = std::to_string(std::min((int)(100*linearDist/totalDistToTravel),99)); 
        std::string completionPercetageFullString = std::string("Test ") + completionPercentage + std::string("\% Completed");
        cv::putText(img, completionPercetageFullString, cv::Point(100,100),  cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,0), 4 );
    }
    else
    {
        if ( testEndTime.toSec() - testBeginTime.toSec() > _testMaxTime )
        {
            std::string completionPercentage = std::to_string(std::min((int)(linearDist/totalDistToTravel*100),99)); 
            std::string completionPercetageFullString = std::string("Test Terminated at ")+ completionPercentage + std::string("\%");
            cv::putText(img, completionPercetageFullString, cv::Point(100,100),  cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 4 );
        }
        else
        {
            std::string completionPercetageFullString = std::string("Test Completed");
            cv::putText(img, completionPercetageFullString, cv::Point(100,100),  cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0,0,0), 4 );
        }
    }

    cv::line(img, cv::Point(50,130), cv::Point(790,130), cv::Scalar(0,0,0), 2);
    cv::line(img, cv::Point(50,330), cv::Point(790,330), cv::Scalar(0,0,0), 2);
    cv::line(img, cv::Point(50,130), cv::Point(50,330), cv::Scalar(0,0,0), 2);
    cv::line(img, cv::Point(790,130), cv::Point(790,330), cv::Scalar(0,0,0), 2);
    cv::putText(img, cv::String("Time(s)"), cv::Point(60,190), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
    cv::putText(img, cv::String("E(Wh)"), cv::Point(210,190), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
    cv::putText(img, cv::String("LinearDist(m)"), cv::Point(335,190), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
    cv::putText(img, cv::String("TotalDist(m)"), cv::Point(590,190), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
    cv::line(img, cv::Point(50,230), cv::Point(790,230), cv::Scalar(0,0,0), 2);
    cv::line(img, cv::Point(180,130), cv::Point(180,330), cv::Scalar(0,0,0), 2);
    cv::line(img, cv::Point(320,130), cv::Point(320,330), cv::Scalar(0,0,0), 2);
    cv::line(img, cv::Point(570,130), cv::Point(570,330), cv::Scalar(0,0,0), 2);

    //write everything with a single digit. 
    char ctmp [50];
    std::sprintf(ctmp,"%3.1f",linearDist); 
    std::string tmpString(ctmp); 
    cv::putText(img, tmpString, cv::Point(335,290), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);

    std::sprintf(ctmp,"%3.1f",travelledDistance); 
    tmpString = ctmp;
    cv::putText(img, tmpString, cv::Point(590,290), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);

    std::sprintf(ctmp,"%2.3f",consumedEnergy); 
    tmpString = ctmp;
    cv::putText(img, tmpString, cv::Point(190,290), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);

    if (!_stop) std::sprintf(ctmp,"%3.1f",ros::Time::now().toSec() - testBeginTime.toSec()); 
    if (_stop && testEndTime.toSec() - testBeginTime.toSec() > _testMaxTime ) std::sprintf(ctmp,"%3.1f",_testMaxTime);
    if (_stop && testEndTime.toSec() - testBeginTime.toSec() <= _testMaxTime ) std::sprintf(ctmp,"%3.1f",testEndTime.toSec() - testBeginTime.toSec());
    tmpString = ctmp;
    cv::putText(img, tmpString, cv::Point(60,290), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,0), 2);
}

void BenchmarkManager::addTargetToCam()
{
    // Move Target In Body Frame
    uint it = 0;
    if ( _odomVector.size() == 0 ) return; 
    while ( it < _odomVector.size() - 1 && _odomVector.at(it).stamp.toSec() < _camStamp.toSec() )
    {
        it++;
    }
    Odometry imgOdom = _odomVector.at(it);

    Point_3D goal_BF( CART, _goals.at(_currentGoalNumber).x - imgOdom.pose.position.x, _goals.at(_currentGoalNumber).y - imgOdom.pose.position.y,  _goals.at(_currentGoalNumber).z - imgOdom.pose.position.z );
    goal_BF.rotateInverse(imgOdom.pose.quaternion);

    // Draw a Point in Body Frame For the Specific looking ahead Camera
    // moving the camera OR changing its characteristic will create errors in the following lines
    double camFovHor = 1.39;
    double camFovVert = camFovHor / _currentImg.cols * _currentImg.rows ;
    
    int x0 = _currentImg.cols/2;
    int y0 = _currentImg.rows/2;
    
    float delta_theta = camFovHor / (float)_currentImg.cols ;
    float delta_phi = camFovVert / (float)_currentImg.rows ;
    
    int x = _currentImg.cols - (int)( goal_BF.theta() / (float)delta_theta + x0 );
    int y = _currentImg.rows - (int)( goal_BF.phi() / (float)delta_phi + y0 ) ;
    if ( x < 0 ) x =0;
    if ( x > _currentImg.cols-1 ) x = _currentImg.cols-1;
    if ( y < 0 ) y=0;
    if ( y >_currentImg.rows-1 ) y = _currentImg.rows-1;

    cv::circle(_currentImg, cv::Point(x,y), 5, cv::Scalar(0,0,255), 7,8,0);
}
