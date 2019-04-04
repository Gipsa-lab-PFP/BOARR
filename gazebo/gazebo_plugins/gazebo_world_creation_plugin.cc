#include <ignition/math/Pose3.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/math/gzmath.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/rendering/rendering.hh"

//#include <gazebo/common/Plugin.hh>

#include <string> 
#include <iostream>
#include <random>
#include <fstream>
#include <memory>

namespace gazebo
{

    struct Point2D{
        double x;
        double y;
    };

    class HeightMap : public WorldPlugin
    {

        public: 
            event::ConnectionPtr updateConnection_;
            physics::WorldPtr parent;
            double x_min, x_max, y_min, y_max, heightMapSize;
            int treeModelNumber, heightMapModelNumber, tree_number, forestMaxNumber, forestGaussianCenterNumber, forestNumber;
            int hgt_done, forest_done, saved, takeScreen, insertCam;

            void OnUpdate(const common::UpdateInfo& _info);
            double heightMapZ(double x, double y);  
            void insert_camera();
            void insert_heightmap();
            void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
            void move_heightmap();
            bool InMinMax(double x, double min, double max);
            double RandNormal(double mean, double stddev);
            double SpawnOneTree(double x, double y, double z);
            bool CheckRestrictions(double x, double y);
            void GenerateYamlFile();

            std::vector<Point2D> safePoints;
            std::vector<double> forestGaussianSTDDEV;
            double safeMargin; 
    };

    void HeightMap::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        // all parameters
        forestMaxNumber = 10; //number of world to generate

        tree_number = 100; //number of tree to spawn in each world
        treeModelNumber = 20; //number of tree model available
        forestGaussianCenterNumber = 3;//number of gaussian center of the randomly spawned tree
        forestGaussianSTDDEV.push_back(12);
        forestGaussianSTDDEV.push_back(30);
        forestGaussianSTDDEV.push_back(60);

        heightMapModelNumber=5; //number of available heightmaps
        heightMapSize = 82; //XXX also in form of a constant in line 306 and 333
        safeMargin=5.; // No tree will be spawned in the closest safemargin to the side of the map on which there will be waypoints
        x_min = - heightMapSize/2. + safeMargin; 
        x_max = heightMapSize/2. - safeMargin; 
        y_min = - heightMapSize/2. + safeMargin;
        y_max = heightMapSize/2. - safeMargin;

        // waypoints, free of tree //XXX old way to do it
        //Point2D tmp;
        //tmp.x = x_min +1;
        //tmp.y = 0;
        //safePoints.push_back(tmp);
        //tmp.x = x_max-15;
        //tmp.y = y_max-1;
        //safePoints.push_back(tmp);
        //tmp.x = x_max-15;
        //tmp.y = y_min+1;
        //safePoints.push_back(tmp);
        //safeMargin=5.; // free dist between the free points and the closest possible tree

        //fixed initialisations
        parent =_parent;
        hgt_done = -1;
        forest_done = -1;
        saved = -1;
        forestNumber = 0;
        insertCam = -1;
        takeScreen = -1;
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&HeightMap::OnUpdate, this, _1));

    }

    void HeightMap::OnUpdate(const common::UpdateInfo& _info) 
    {
        //std::cout << "current it " << parent->GetIterations() << std::endl; 
        if ( parent->GetIterations() > 500 && hgt_done < 0 )
        {
            std::cout << forestNumber+1 << "th world genertion" << std::endl; 
            std::cout << "it count = " << parent->GetIterations() << " Generating the heightmap" << std::endl; 
            insert_heightmap();
            hgt_done = parent->GetIterations();
            std::cout << "end heightmap" << std::endl; 
        }
        else if ( parent->GetIterations() > 1000 && forest_done < 0 )
        {
            std::cout << "it count = " << parent->GetIterations() << " Generating the Yaml File" << std::endl; 
            GenerateYamlFile();
            std::cout << "it count = " << parent->GetIterations() << " Generating the forest" << std::endl; 
            for ( int i = 0; i < forestGaussianCenterNumber ; i++ )
            {
                double centeringRatio = ( 1 - 2*forestGaussianSTDDEV.at(i)/heightMapSize);
                centeringRatio = centeringRatio< 0.1 ?0.1 :(centeringRatio> 0.9 ?0.9 :centeringRatio);
                double x_center = ( double(rand())/double(RAND_MAX) - 0.5 ) * centeringRatio * heightMapSize;
                double y_center = ( double(rand())/double(RAND_MAX) - 0.5 ) * centeringRatio * heightMapSize;
                std::cout << "x_center " << x_center << std::endl; 
                std::cout << "y_center " << y_center << std::endl; 
                for ( int j = 0; j < tree_number / forestGaussianCenterNumber ; j++) 
                {   
                    double x = RandNormal(x_center, forestGaussianSTDDEV.at(i)); //XXX use std::normal_distribution instead, see http://www.cplusplus.com/reference/random/normal_distribution/
                    double y = RandNormal(y_center, forestGaussianSTDDEV.at(i)); //XXX use std::normal_distribution instead, see http://www.cplusplus.com/reference/random/normal_distribution/
                    if ( !CheckRestrictions(x,y) )
                    {
                        j--;
                        continue;
                    }
                    double z= heightMapZ(x, y);
                    SpawnOneTree(x,y,z);
                }
            }
            forest_done = parent->GetIterations();
            std::cout << "end forest creation" << std::endl; 
        }
        else if ( parent->GetIterations() > 1500 && saved < 0 )
        {
            // save the world
            std::cout << "it count = " << parent->GetIterations() << " Saving the forest" << std::endl; 
            std::string worldName = std::string("../worlds/forest") + std::to_string(forestNumber) + std::string(".world");
            parent->Save(worldName);
            saved = parent->GetIterations();
            std::cout << "end saving" << std::endl; 
        }
        else if ( parent->GetIterations() > 2000 && insertCam < 0 )
        {
            //std::cout << "begin camera creation" << std::endl; 
                //insert_camera();
            //std::cout << "end camera creation" << std::endl; 
            insertCam = parent->GetIterations();
        }
        else if ( parent->GetIterations() > 2500 && takeScreen < 0 )
        {
            std::string imgName = std::string("../worlds/forest") + std::to_string(forestNumber) + std::string(".jpg");
            std::cout << "settting up the dynamic pointer" << std::endl; 
            //std::string cameraName = "my_camera" + std::to_string(forestNumber);
            //auto camera = std::dynamic_pointer_cast<sensors::CameraSensor>(
                //sensors::SensorManager::Instance()->GetSensor(cameraName));
            //std::cout << "end settting up the dynamic pointer" << std::endl; 
            //camera->Camera()->EnableSaveFrame(true);
            //camera->Camera()->SetSaveFramePathname("/tmp/tmp");
            //camera->Camera()->SetCaptureDataOnce();
            //camera->C
            //camera->Camera()->Render();
            //camera->Camera()->SaveFrame(imgName);
            takeScreen = parent->GetIterations();
            std::cout << "end frame saved" << std::endl; 
        }
        else if ( parent->GetIterations() > 3000 )
        {
            std::cout << "resetting for next world creation" << std::endl;
            if ( forestNumber < forestMaxNumber - 1 ) 
            {
                forestNumber++;
                saved = -1;
                forest_done = -1;
                hgt_done = -1;
                insertCam = -1;
                takeScreen = -1;
                //std::cout << "clear sensors" << std::endl; 
                //sensors::SensorManager* sensor_manager = sensors::SensorManager::Instance();
                //auto sensor_manager = std::dynamic_pointer_cast<sensors::SensorManager*>(
                //sensors::SensorManager::Instance());
                //std::cout << "use fini on sensor" << std::endl; 
                //sensor_manager->RemoveSensors();
                //sensors::SensorManager::Instance()->Fini();
                //std::cout << "clear models" << std::endl; 
                //parent->Clear();
                parent->ClearModels();
                //std::cout << "reset" << std::endl; 
                parent->Reset();
                std::cout << "end resetting" << std::endl; 
            }
            else
            {
                std::cout << "all the worlds have been generated \n don't forget to use the .sh file to treat the created world files. \n Plugin stopped on a getchar()." << std::endl;
                std::getchar();
            }
        }
    }

    bool HeightMap::CheckRestrictions(double x, double y)
    {
        if ( !InMinMax(x, x_min, x_max) ) return false;
        if ( !InMinMax(y, y_min, y_max) ) return false;

        // XXX old way to restrict tree spawning around waypoints
        //for (uint i = 0; i < safePoints.size(); i++ )
        //{
            //double d2 = ( safePoints.at(i).x - x ) * ( safePoints.at(i).x - x ) + ( safePoints.at(i).y - y ) * ( safePoints.at(i).y - y );
            //if ( d2 < safeMargin*safeMargin ) return false;
        //}
        return true;
    }

    void HeightMap::move_heightmap() //XXX not available, see issue 245 on the gazebo bitbucket repo
    {
        //double z0 = heightMapZ(0,0);
        //std::cout << "zo " << z0 << std::endl; 

        //physics::ModelPtr model = parent->GetModel("heightmap")->GetLink("link")->GetCollision("collision");
        //parent->GetModel("heightmap")->SetStatic(false);
        //math::Pose thisPose = math::Pose(0,0,10,0,0,0);
        //parent->GetModel("heightmap")->SetWorldPose(thisPose);
        //parent->GetModel("heightmap")->Update();
        //parent->GetModel("heightmap")->SetStatic(true);
        //z0 = heightMapZ(0,0);
        //std::cout << "zo " << z0 << std::endl; 
    }

    double HeightMap::heightMapZ(double x, double y )
    {
        // getting the pointer to the HeightmapShape
        physics::ModelPtr model = parent->GetModel("heightmap");
        physics::CollisionPtr collision = model->GetLink("link")->GetCollision("collision");
        physics::HeightmapShapePtr heightmap = boost::dynamic_pointer_cast<physics::HeightmapShape>(collision->GetShape());

        // coordinate transform from regular Word (x,y) to the HeightmapShape (index_x,index_y) 
        math::Vector3 size = heightmap->GetSize(); 
        math::Vector2i vc = heightmap->GetVertexCount();
        int index_x = (  ( (x + size.x/2)/size.x ) * vc.x - 1 ) ;
        int index_y = (  ( (-y + size.y/2)/size.y ) * vc.y - 1 ) ;

        //  getting the height :
        double z =  heightmap->GetHeight( index_x , index_y ) ; 

        return z;
    }

    double HeightMap::SpawnOneTree(double x, double y, double z)
    {
        std::string treeString1 =  "<sdf version='1.6'>\
                                    <model name='my_mesh'>\
                                    <pose> ";
        std::string space = " ";
        std::string treeString2 = " </pose>\
                                   <static>1</static>\
                                   <link name='body'>\
                                   <visual name='visual'>\
                                   <geometry>\
                                   <mesh>\
                                   <uri>model://trees/";
        std::string treeString3 =  ".dae</uri>\
                                    <scale>1 1 1</scale>\
                                    </mesh>\
                                    </geometry>\
                                    </visual>\
                                    <collision name='collision'>\
                                    <geometry>\
                                    <mesh>\
                                    <uri>model://trees/";
        std::string treeString4 =  ".dae</uri>\
                                    <scale>1 1 1</scale>\
                                    </mesh>\
                                    </geometry>\
                                    </collision>\
                                    <self_collide>0</self_collide>\
                                    <kinematic>0</kinematic>\
                                    <gravity>1</gravity>\
                                    </link>\
                                    </model>\
                                    </sdf>";

        int chosenTreeModel = rand() % treeModelNumber;
        std::string treeString = treeString1 + std::to_string(x) + space + std::to_string(y) + space + std::to_string(z)
                + space + std::to_string(0) + space + std::to_string(0) + space + std::to_string(rand()/(double)RAND_MAX*3.14-1.57)
                + treeString2 + std::to_string(chosenTreeModel) + treeString3 + std::to_string(chosenTreeModel) + treeString4;

        sdf::SDF treeSDF;
        treeSDF.SetFromString(treeString);
        parent->InsertModelSDF(treeSDF);
    }


    void HeightMap::insert_heightmap()
    {
        // insert the heightmap : 
        std::string heightMapString1 =  "<sdf version ='1.4'>\
                                         <model name='heightmap'>\
                                         <pose frame=''>0 0 0 0 -0 0</pose>\
                                         <static>1</static>\
                                         <link name='link'>\
                                         <collision name='collision'>\
                                         <geometry>\
                                         <heightmap>\
                                         <uri>model://heightmaps/";
        std::string heightMapString2 =  ".img</uri>\
                                         <size>82 82 30</size>\
                                         <pos>0 0 0</pos>\
                                         </heightmap>\
                                         </geometry>\
                                         <max_contacts>10</max_contacts>\
                                         </collision>\
                                         <visual name='visual_heightmap'>\
                                         <geometry>\
                                         <heightmap>\
                                         <texture>\
                                         <diffuse>file://media/materials/textures/dirt_diffusespecular.png</diffuse>\
                                         <normal>file://media/materials/textures/flat_normal.png</normal>\
                                         <size>1</size>\
                                         </texture>\
                                         <texture>\
                                         <diffuse>file://media/materials/textures/grass_diffusespecular.png</diffuse>\
                                         <normal>file://media/materials/textures/flat_normal.png</normal>\
                                         <size>1</size>\
                                         </texture>\
                                         <texture>\
                                         <diffuse>file://media/materials/textures/fungus_diffusespecular.png</diffuse>\
                                         <normal>file://media/materials/textures/flat_normal.png</normal>\
                                         <size>1</size>\
                                         </texture>\
                                         <blend>\
                                         <min_height>2</min_height>\
                                         <fade_dist>5</fade_dist>\
                                         </blend>\
                                         <blend>\
                                         <min_height>12</min_height>\
                                         <fade_dist>5</fade_dist>\
                                         </blend>\
                                         <uri>file://heightmaps/map";
        std::string heightMapString3 = ".img</uri>\
                                        <size>82 82 30</size>\
                                        </heightmap>\
                                        </geometry>\
                                        </visual>\
                                        <self_collide>0</self_collide>\
                                        <kinematic>0</kinematic>\
                                        <gravity>1</gravity>\
                                        </link>\
                                        </model>\
                                        </sdf>";

        int chosenMap = rand() % heightMapModelNumber;
        std::string heightMapString = heightMapString1 + std::to_string(chosenMap) + heightMapString2 + 
            std::to_string(chosenMap) + heightMapString3;

        sdf::SDF heightmapSDF;
        heightmapSDF.SetFromString(heightMapString);
        parent->InsertModelSDF(heightmapSDF);
    }
    
    
    void HeightMap::insert_camera()
    {
        // insert the heightmap : 
        //std::string cameraString = "<sdf version='1.6'>\
                                      //<model name='camera'>\
                                        //<link name='model_for_camera/base_link'>\
                                          //<pose frame=''>0 0 10 0 -0 0</pose>\
                                          //<inertial>\
                                            //<pose frame=''>0 0 10 0 0 0</pose>\
                                            //<mass>3.811</mass>\
                                            //<inertia>\
                                              //<ixx>0.0357563</ixx>\
                                              //<ixy>0</ixy>\
                                              //<ixz>0</ixz>\
                                              //<iyy>0.0471428</iyy>\
                                              //<iyz>0</iyz>\
                                              //<izz>0.0989499</izz>\
                                            //</inertia>\
                                          //</inertial>\
                                          //<sensor name='my_camera' type='camera'>\
                                            //<always_on>1</always_on>\
                                            //<update_rate>30</update_rate>\
                                            //<camera>\
                                              //<horizontal_fov>1.27</horizontal_fov>\
                                              //<image>\
                                                //<format>L8</format>\
                                                //<width>640</width>\
                                                //<height>640</height>\
                                              //</image>\
                                              //<clip>\
                                                //<near>0.1</near>\
                                                //<far>100</far>\
                                              //</clip>\
                                            //</camera>\
                                            //<pose frame=''>0 0 0 0 -0 0</pose>\
                                          //</sensor>\
                                        //</link>\
                                      //</model>\
                                    //</sdf>";

        std::string cameraString = "<sdf version='1.6'>\
                                       <model name='camera";
        
        std::string cameraString2 = "'>\
                                          <static>true</static>\
                                          <pose>0 0 55 0 1.57079633 0</pose>\
                                          <link name='link'>\
                                            <sensor name='my_camera";
       
        std::string cameraString3 = "' type='camera'>\
                                              <camera>\
                                                <horizontal_fov>1.57079633</horizontal_fov>\
                                                <image>\
                                                  <width>2160</width>\
                                                  <height>2160</height>\
                                                </image>\
                                                <clip>\
                                                  <near>0.1</near>\
                                                  <far>1000</far>\
                                                </clip>\
                                              </camera>\
                                              <always_on>1</always_on>\
                                              <update_rate>30</update_rate>\
                                            </sensor>\
                                          </link>\
                                        </model>\
                                    </sdf>";

        std::string cameraStringFinal = cameraString + std::to_string(forestNumber) + cameraString2 + 
            std::to_string(forestNumber) + cameraString3;
        sdf::SDF cameraSDF;
        cameraSDF.SetFromString(cameraStringFinal);
        parent->InsertModelSDF(cameraSDF);
    }

    double HeightMap::RandNormal(double mean, double stddev)//Box muller method
    {
        static double n2 = 0.0;
        static int n2_cached = 0;
        if (!n2_cached)
        {
            double x, y, r;
            do
            {
                x = 2.0*rand()/RAND_MAX - 1;
                y = 2.0*rand()/RAND_MAX - 1;

                r = x*x + y*y;
            }
            while (r == 0.0 || r > 1.0);
            {
                double d = sqrt(-2.0*log(r)/r);
                double n1 = x*d;
                n2 = y*d;
                double result = n1*stddev + mean;
                n2_cached = 1;
                return result;
            }
        }
        else
        {
            n2_cached = 0;
            return n2*stddev + mean;
        }
    }

    bool HeightMap::InMinMax(double x, double min, double max)
    {
        return x<min ?false :(x>max ?false :true);
    }

    void HeightMap::GenerateYamlFile()
    {
        std::string yamlName = std::string("../worlds/forest") + std::to_string(forestNumber) + std::string(".yaml");

        // define all possible waypoits and find their z coordinates : 
        double x[16],y[16],z[16];
        x[0] = 25.0; y[0] = - 40.0;
        x[1] = 10.0; y[1] = - 40.0;
        x[2] = -10.0; y[2] = - 40.0;
        x[3] = -25.0; y[3] = - 40.0;
        x[4] = -40.0; y[4] = -25.0;
        x[5] = -40.0; y[5] = -10.0;
        x[6] = -40.0; y[6] = 10.0;
        x[7] = -40.0; y[7] = 25.0;
        x[8] = -25.0; y[8] = 40.0;
        x[9] = -10.0; y[9] = 40.0;
        x[10] = 10.0; y[10] = 40.0;
        x[11] = 25.0; y[11] = 40.0;
        x[12] = 40.0; y[12] = 25.0;
        x[13] = 40.0; y[13] = 10.0;
        x[14] = 40.0; y[14] = -10.0;
        x[15] = 40.0; y[15] = -25.0;
        for (int i = 0; i < 16 ; i++ )
        {
            z[i] = heightMapZ(x[i], y[i]);
        }

        // randomly selecting a spawn point and saving it a configuration file
        int spawnId = rand() % 16;
        std::ofstream yamlFile;
        yamlFile.open(yamlName.c_str());
        yamlFile << "spawnPos: {x: " << x[spawnId] << " , y: " << y[spawnId] << " , z: " << z[spawnId] << " }\n" << std::endl;  

        // randomly defining the waypoint sequence 
        yamlFile << "goals:"<<std::endl; 
        int waypointNumber = 1;
        int waypointId = spawnId;
        double linearDist = 0;
        while( linearDist < 1000 ) 
        {   
            double old_x = x[waypointId];
            double old_y = y[waypointId];
            double old_z = z[waypointId];
            waypointId = (waypointId + 5 + rand() % 3) % 16;
            yamlFile << "  '"<<waypointNumber<<"': {x: " << x[waypointId] << ", y: " << y[waypointId] << " , z: " << z[waypointId] + 2.0 << ", minDistToValidate: 1.}" <<std::endl; 
            waypointNumber++;
            linearDist += std::sqrt( (x[waypointId]-old_x)*(x[waypointId]-old_x) + (y[waypointId]-old_y)*(y[waypointId]-old_y) + (z[waypointId]-old_z)*(z[waypointId]-old_z) );
            if ( waypointId > 100 ) break; //smth went wrong if we are here; this is being used during tests
        }
        yamlFile.close();
    }
    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(HeightMap)
}
