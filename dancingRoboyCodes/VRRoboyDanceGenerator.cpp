#include "roboy_simulation/VRRoboyDanceGenerator.hpp"
#include <math.h>
#include <Eigen/src/Core/Matrix.h>

using namespace std;
using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(VRRoboyDanceGenerator)




VRRoboyDanceGenerator::VRRoboyDanceGenerator() : ModelPlugin() {
    prevTime = std::chrono::high_resolution_clock::now();
    printf("\n\nInitiated Simulation with %i FPS\n\n", FRAMESPERSEC);
}

VRRoboyDanceGenerator::~VRRoboyDanceGenerator(){}

void VRRoboyDanceGenerator::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
    printf("\nVRRoboyDanceGenerator - loading \n");
    // get the model
    model = _parent;
    //Disable gravity
    _parent->SetGravityMode(false);
    // bind the gazebo update function to OnUpdate
    updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&VRRoboyDanceGenerator::OnUpdate, this, _1));
    // get all joints and the initial pose
    initPose = model->GetWorldPose();

    // Initialize ROS if it is has not been initialized
    if(!ros::isInitialized())
    {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "Roboy_simplified_moveable"); // name
    }

    // Create ros node
    nh = ros::NodeHandlePtr(new ros::NodeHandle("roboy"));
    spinner = boost::shared_ptr<ros::AsyncSpinner>(new ros::AsyncSpinner(1));
    spinner->start();
    ROS_ERROR("HERE creating dance moves");
    //create publisher and subscriber
    external_force_sub = nh->subscribe("/roboy/DanceMoves", 1, &VRRoboyDanceGenerator::ApplyExternalForce, this);
    pose_pub = nh->advertise<roboy_communication_middleware::Pose>("/roboy/pose", 100);
}



/** Applies external force to model and publishes message so that force is displayed in RVIZ
 * External force is applied locally: the given coordinates describe the local position with respect to the respective
 * roboy body part, the force is a vector in local direction
 * FIXME ATTENTION: FOR NOW; THE UNITY SIDE SENDS WORLD SPACE COORDINATES DUE TO ISSUES WITH THE MODEL -> LOCAL SPACE APPLIED DIRECTLY
 * */
void VRRoboyDanceGenerator::ApplyExternalForce(const roboy_communication_simulation::DanceMoves::ConstPtr &msg) {

    Target1 = msg->leftArm;
    Target2 = msg->leftLeg;
    Target3 = msg->rightArm;
    Target4 = msg->rightLeg;
    /** Find concerned body part*/
    /*physics::LinkPtr link = model->GetChildLink(msg->name);
    ForceCounter++;
    string name1="";
    model->GetChildLink(name1)->GetWorldPose();
    if (link != nullptr) {


        math::Vector3 localforce(msg->f_x, msg->f_y, msg->f_z);
        //math::Vector3 worldforce =  link->GetWorldPose().rot.GetInverse() *localforce;
        math::Vector3 localpos(msg->x, msg->y, msg->z); //local pos with respect to link
        //link->AddForceAtWorldPosition(worldforce, worldpos);
        link->AddForceAtWorldPosition(localforce, localpos);
        //link->AddForceAtRelativePosition(localforce, localpos);
        //SetForce


    }*/
}

void VRRoboyDanceGenerator::publishPose()
{
    roboy_communication_middleware::Pose msg;
    int linkCounter = 1; // for rviz messages
    for(auto link:model->GetLinks()){
        math::Pose p = link->GetWorldPose();
        /** rviz message construction & publish*/
        //since in CAD folder no "neck_spinal.stl" specified but model.sdf contains it -> have to ignore it in order to make rviz work
        //for now: workaround / dirty hack -> make prettier or adapt model files
        //TODO: don't know if this is still necessary
        if (link->GetName() == "neck_spinal") {
            continue;
        }
        Vector3d eigenpos(p.pos.x, p.pos.y, p.pos.z);
        Quaterniond eigenrot(p.rot.w, p.rot.x, p.rot.y, p.rot.z);
        // namespace not important -> rviz separates in certain namespaces, choose same one and you're fine
        // got its own node handler nh
        // keep message id the same for same obj (message id #0-> force)
        publishMesh( "roboy_models", "Roboy_simplified_moveable/meshes/CAD", (link->GetName() +".stl").c_str(), eigenpos, eigenrot,
                     0.001, "world", "model", linkCounter, 0);
        linkCounter ++;

        /** second message construction*/
        msg.name.push_back(link->GetName());
        msg.x.push_back(p.pos.x);
        msg.y.push_back(p.pos.y);
        msg.z.push_back(p.pos.z);
        p.rot.Normalize();
        msg.qx.push_back(p.rot.x);
        msg.qy.push_back(p.rot.y);
        msg.qz.push_back(p.rot.z);
        msg.qw.push_back(p.rot.w);

    }
    /** publish second message*/
    pose_pub.publish(msg);
    PoseCounter++;
}

void VRRoboyDanceGenerator::PrintStats(){
    std::time_t currentseconds = std::time(nullptr);
    if(seconds != currentseconds ){
        seconds = currentseconds;
        printf("Sent: %i msgs this second Dance baby dance\n", PoseCounter);
        printf("Force msgs received: %i \n\n", ForceCounter);
        PoseCounter = 0;
        ForceCounter = 0;
    }
}


void VRRoboyDanceGenerator::OnUpdate(const common::UpdateInfo &_info)
{
    /**Restrict frame rate since UNITY (on windows) starts having a backlog of msgs otherwise
     * - for now, unity is unable to do that itself*/
    int deltaframerate =  1000/ FRAMESPERSEC;
    std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - prevTime);
    geometry_msgs::Point Force1,Force2,Force3,Force4;
    //Restriction: more than delta frame rate needs to have passed
    // -> actual fps <= FRAMESPERSECOND (oftentimes a couple of frames less)
    if(milliseconds.count() > deltaframerate){

        prevTime = currentTime;




        physics::LinkPtr link1 = model->GetChildLink("lower_arm_left");
        physics::LinkPtr link2 = model->GetChildLink("fuss_links"); //// berkay added
        physics::LinkPtr link3 = model->GetChildLink("hand_right"); //// berkay added
        physics::LinkPtr link4 = model->GetChildLink("head"); //// berkay added

        math::Vector3 pose1 = link1->GetWorldPose().pos;
        math::Vector3 vel1 = link1->GetWorldLinearVel();
        //'fuss_links', 'fuss_rechts' lower_arm_left

        math::Vector3 pose2 = link1->GetWorldPose().pos; //// berkay added
        math::Vector3 vel2 = link1->GetWorldLinearVel();

        math::Vector3 pose3 = link1->GetWorldPose().pos; //// berkay added
        math::Vector3 vel3 = link1->GetWorldLinearVel();

        math::Vector3 pose4 = link1->GetWorldPose().pos; //// berkay added
        math::Vector3 vel4 = link1->GetWorldLinearVel();

        string name1="";
        if (link1 != nullptr) {  //// berkay changed: from "link" to "link1"

            Force1.x = 100*(Target1.x-pose1.x) -20*vel1.x;
            Force1.y = 100*(Target1.y-pose1.y) -20*vel1.y;
            Force1.z = 100*(Target1.z-pose1.z) -20*vel1.z;

            math::Vector3 localforce1(Force1.x, Force1.y, Force1.z);
            //math::Vector3 worldforce =  link->GetWorldPose().rot.GetInverse() *localforce;
            //math::Vector3 localpos(msg->x, msg->y, msg->z); //local pos with respect to link
            //link->AddForceAtWorldPosition(worldforce, worldpos);
            //link->AddForceAtWorldPosition(localforce, localpos);
            //link->AddForceAtRelativePosition(localforce, localpos);
            link1->SetForce(localforce1);
        }

        if (link2 != nullptr) { //// berkay added

            Force2.x = 100*(Target2.x-pose2.x) -20*vel2.x;
            Force2.y = 100*(Target2.y-pose2.y) -20*vel2.y;
            Force2.z = 100*(Target2.z-pose2.z) -20*vel2.z;

            math::Vector3 localforce2(Force2.x, Force2.y, Force2.z);
            link2->SetForce(localforce2);
        }

        if (link3 != nullptr) { //// berkay added

            Force3.x = 100*(Target3.x-pose3.x) -20*vel3.x;
            Force3.y = 100*(Target3.y-pose3.y) -20*vel3.y;
            Force3.z = 100*(Target3.z-pose3.z) -20*vel3.z;

            math::Vector3 localforce3(Force3.x, Force3.y, Force3.z);
            link3->SetForce(localforce3);
        }

        if (link4 != nullptr) { //// berkay added

            Force4.x = 100*(Target4.x-pose4.x) -20*vel4.x;
            Force4.y = 100*(Target4.y-pose4.y) -20*vel4.y;
            Force4.z = 100*(Target4.z-pose4.z) -20*vel4.z;

            math::Vector3 localforce4(Force4.x, Force4.y, Force4.z);
            link4->SetForce(localforce4);
        }


        prevForce1=Force1;
        prevForce2=Force2;
        prevForce3=Force3;
        prevForce4=Force4;


        publishPose();
        PrintStats();
    }
}