/* -------------------------------------------------------------------------- */
/*                                  INCLUDES                                  */
/* -------------------------------------------------------------------------- */
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <lai_msgs/DesiredTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <lai_msgs/PickAndPlaceAction.h>
#include <deque>
#include <sensor_msgs/JointState.h>
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <visualization_msgs/Marker.h>
#include <ros/package.h>

/* -------------------------------------------------------------------------- */
/*                                  LOGGING                                   */
/* -------------------------------------------------------------------------- */
static std::string session_folder;
static std::ofstream log_file_offshore;
static uint64_t global_seq_id_offshore = 0;
size_t max_joint_state_buffer_size_ = 1000;

/* -------------------------------------------------------------------------- */
/*                              OFFSHORE NODE                                 */
/* -------------------------------------------------------------------------- */
class OffshoreNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

    // ---------------------- Subscribers ----------------------
    ros::Subscriber traj_sub_;             // Subscribe to desired trajectory commands
    ros::Subscriber gripper_cmd_sub_;      // Subscribe to gripper commands
    ros::Subscriber control_mode_sub_;     // Subscribe to control mode changes (auto/manual)
    ros::Subscriber pick_place_goal_sub_;  // Subscribe to Pick & Place goals
    ros::Subscriber joint_states_sub_;     // Subscribe to joint states

    // ---------------------- Publishers -----------------------
    ros::Publisher desired_traj_pub_;      // Publish trajectory to robot controller
    ros::Publisher joint_states_delayed_pub_; // Publish joint states after delay
    ros::Publisher manual_goal_pub_;       // Publish Pick & Place goals in manual mode
    ros::Publisher marker_pub_;            // Publish visual feedback in RViz

    // ---------------------- Buffers --------------------------
    std::deque<std::pair<ros::Time, sensor_msgs::JointState>> joint_states_buffer_; // Joint states buffer

    // ---------------------- Timers ---------------------------
    ros::Timer joint_states_timer_;        // Timer for delayed publishing of joint states

    // ---------------------- Gripper --------------------------
    std::unique_ptr<actionlib::SimpleActionClient<franka_gripper::MoveAction>> gripper_client_;
    bool gripper_open_;                     // True if gripper is currently open


    // ---------------------- Parameters -----------------------
    double return_delay_;                    // Delay for joint state feedback
    std::string control_mode_;               // Current control mode ("auto" or "manual")
    ros::Duration action_timeout_;           // Timeout for action clients

    
    // ---------------------- Action Clients -------------------
    std::unique_ptr<actionlib::SimpleActionClient<lai_msgs::PickAndPlaceAction>> pick_place_client_;

    /* --------------------------------------------------------------------- */
    /*               Check if Pick & Place Action Server is ready            */
    /* ----------------------------------------------------------------------*/

    bool isPickPlaceServerActive() {
         // Return false if the server is not connected
        if (!pick_place_client_->isServerConnected()) {
            ROS_ERROR("[OFFSHORE] Action server non connesso!");
            return false;
        }
        // Check if an action is already running
        actionlib::SimpleClientGoalState state = pick_place_client_->getState();
        if (state == actionlib::SimpleClientGoalState::ACTIVE || 
            state == actionlib::SimpleClientGoalState::PENDING) {
            ROS_WARN("[OFFSHORE] Azione in esecuzione!");
            return false;
        }
        
        return true;  // Server is ready 
    }

public:
/* ------------------------------------------------------------------ */
/*                           Constructor                              */
/* ------------------------------------------------------------------ */
    OffshoreNode()  
        : nh_private_("~"),
        return_delay_(0.0), 
        gripper_open_(true), 
        action_timeout_(90.0) {

        // Read parameters from private namespace        
        nh_private_.param("return_delay", return_delay_, 0.0);
        //nh_private_.param<std::string>("control_mode", control_mode_, "auto");
        //ROS_INFO("Valore letto di control_mode: %s", control_mode_.c_str());
        control_mode_ = "manual"; // Default manual
        
        /* ---------------------- Logging setup ----------------------*/
        if (session_folder.empty()) {
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::stringstream folder_name;
            std::string package_path = ros::package::getPath("franka_delay_node");  
            folder_name << package_path << "/log_file/session_" 
                        << std::put_time(&tm, "%Y%m%d_%H%M%S");
                        
            if (mkdir(folder_name.str().c_str(), 0777) != 0) {
                ROS_WARN_STREAM("[NODE] Impossibile creare cartella: " << folder_name.str());
            }
            session_folder = folder_name.str();
        }
        

        std::stringstream filename;
        filename << session_folder << "/offshore_log" << "_delay_" << (return_delay_) << ".csv";

        // Apro il file per scrivere il log
        log_file_offshore.open(filename.str());

        ROS_INFO_STREAM("[OFFSHORE] Log salvato in: " << filename.str());

        // Header CSV
        log_file_offshore << "seq_id,msg_type,event,stamp_send,stamp_recv,delay_s,buffer_size \n";
        log_file_offshore << std::fixed << std::setprecision(6);
        // Wait for /clock if using simulation time
        if (ros::Time::isSimTime()) {
            ROS_INFO("[OFFSHORE] Attendo che /clock inizi a pubblicare...");
            ros::Time begin = ros::Time::now();
            while (begin.toSec() == 0.0) {
                ros::spinOnce();
                ros::Duration(0.1).sleep();
                begin = ros::Time::now();
            }
            ROS_INFO("[OFFSHORE] /clock attivo.");
        }

        // ---------------------- Subscribers ----------------------
        traj_sub_ = nh_.subscribe("/offshore/traj_cmd", 1, &OffshoreNode::trajCallback, this);
        gripper_cmd_sub_ = nh_.subscribe("/offshore/gripper_cmd", 1, &OffshoreNode::gripperCallback, this);
        joint_states_sub_ = nh_.subscribe("/joint_states", 100, &OffshoreNode::jointStatesCallback, this);
        control_mode_sub_ = nh_.subscribe("/control_mode", 1, &OffshoreNode::controlModeCallback, this);
        pick_place_goal_sub_ = nh_.subscribe("/offshore/pick_and_place_goal", 1, &OffshoreNode::pickPlaceGoalCallback, this);

        // ---------------------- Publishers ----------------------
        desired_traj_pub_ = nh_.advertise<lai_msgs::DesiredTrajectory>("/operational_space_impedance_controller/desired_trajectory", 1);
        joint_states_delayed_pub_ = nh_.advertise<sensor_msgs::JointState>("/joint_states_delayed", 100);
        manual_goal_pub_ = nh_.advertise<lai_msgs::PickAndPlaceGoal>("/manual_pick_place_goal", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("popup_marker", 1);

        // ---------------------- Timer ----------------------
        joint_states_timer_ = nh_.createTimer(ros::Duration(0.002), &OffshoreNode::jointStatesTimerCallback, this);

        // ---------------------- Action clients ----------------------
        gripper_client_ = std::make_unique<actionlib::SimpleActionClient<franka_gripper::MoveAction>>("/franka_gripper/move", true);
        pick_place_client_ = std::make_unique<actionlib::SimpleActionClient<lai_msgs::PickAndPlaceAction>>("/lai/pick_and_place", true);

        ROS_INFO("[OFFSHORE] In attesa dei server action...");


        if (gripper_client_->waitForServer(ros::Duration(5.0))) {
            ROS_INFO("[OFFSHORE] Connesso al server del gripper");
        } else {
            ROS_WARN("[OFFSHORE] Server gripper non disponibile");
        }

        if (control_mode_ == "auto") {
            ROS_INFO("[OFFSHORE] In attesa del server pick&place (fino a connessione)...");
            pick_place_client_->waitForServer();  
            ROS_INFO("[OFFSHORE] Server pick&place connesso!");
        }

        ROS_INFO("[OFFSHORE] Nodo inizializzato. Modalità: %s", control_mode_.c_str());
    }
/* ---------------------------------------------------------------------- */
/*                     Pick & Place Goal Callback                         */
/* ---------------------------------------------------------------------- */
    void pickPlaceGoalCallback(const lai_msgs::PickAndPlaceGoal::ConstPtr& msg) {
        ros::Time now = ros::Time::now();
        if (control_mode_ == "auto") {
            if (!isPickPlaceServerActive()) {
                ROS_ERROR("[OFFSHORE] Impossibile inviare goal - server non pronto");
                return;
            }

            ROS_INFO("[OFFSHORE] Invio goal automatico all'action server");

            ROS_INFO("Pick Position: x=%.3f, y=%.3f, z=%.3f", 
                    msg->pick_position.x, msg->pick_position.y, msg->pick_position.z);
            
            log_file_offshore << global_seq_id_offshore++ << ",pick_place,goal_sent,"
                         << now.toSec() << ",,,\n";

            pick_place_client_->sendGoal(*msg,
                boost::bind(&OffshoreNode::pickPlaceDoneCallback, this, _1, _2),
                boost::bind(&OffshoreNode::pickPlaceActiveCallback, this)
            );
            
            pick_place_client_->waitForResult();  // Wait indefinitely 

        } 
        else if (control_mode_ == "manual") {
            ROS_INFO("[OFFSHORE] Invio goal manuale su topic");
            manual_goal_pub_.publish(*msg);
        }
        else {
            ROS_WARN("[OFFSHORE] Modalità sconosciuta: %s", control_mode_.c_str());
        }
    }
/* ---------------------------------------------------------------------- */
/*                      Pick & Place Done Callback                        */
/* ---------------------------------------------------------------------- */
    void pickPlaceDoneCallback(const actionlib::SimpleClientGoalState& state,
                            const lai_msgs::PickAndPlaceResultConstPtr& result) {
        ros::Time now = ros::Time::now();
        ROS_INFO_STREAM("[OFFSHORE] Risultato operazione: " << state.toString());
        log_file_offshore << global_seq_id_offshore++ << ",pick_place,action_ended,"
                     << now.toSec() << ",,," << state.toString() << "\n";

        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("[OFFSHORE] Operazione completata con successo!");
            showSuccessPopup();  //view popup
        } else {
            ROS_ERROR("[OFFSHORE] Operazione fallita: %s", state.getText().c_str());
        }
    }
/* ---------------------------------------------------------------------- */
/*                    Pick & Place Active Callback                        */
/* ---------------------------------------------------------------------- */
    void pickPlaceActiveCallback() {
        ros::Time now = ros::Time::now();
        ROS_INFO("[OFFSHORE] Pick&Place iniziato...");
          log_file_offshore << global_seq_id_offshore++ << ",pick_place,action_started,"
                     << now.toSec() << ",,,\n";
    }

/* ---------------------------------------------------------------------- */
/*                    Control Mode Change Callback                        */
/* ---------------------------------------------------------------------- */
   void controlModeCallback(const std_msgs::String::ConstPtr& msg) {
    std::string new_mode = msg->data;
    
    if (new_mode == "auto" || new_mode == "manual" || new_mode == "follow") {
        if (new_mode != control_mode_) {
            control_mode_ = new_mode;
            ROS_INFO("[OFFSHORE] Cambiata mod controllo a: %s", control_mode_.c_str());
            
            // Verifica la connessione al server solo per modalità auto
            if (control_mode_ == "auto") {
                if(!pick_place_client_->waitForServer(ros::Duration(2.0))) {
                    ROS_WARN("[OFFSHORE] Server pick&place non disponibile in mod auto");
                }
            }
        }
    } else {
        ROS_WARN("[OFFSHORE] Tentativo di impostare mod non valida: %s", new_mode.c_str());
    }
}

/* ---------------------------------------------------------------------- */
/*                      Trajectory Callback                               */
/* ---------------------------------------------------------------------- */
    void trajCallback(const lai_msgs::DesiredTrajectory::ConstPtr& msg) {
        ros::Time now = ros::Time::now();
        ros::Time stamp_onshore = msg->header.stamp;
        double delay = (now - stamp_onshore).toSec();

        ROS_INFO_STREAM("[OFFSHORE] Ricevuta traiettoria. Delay: " << delay << " s");
        desired_traj_pub_.publish(*msg);
    }
/* ---------------------------------------------------------------------- */
/*                        Gripper Callback                                  */
/* ---------------------------------------------------------------------- */
    void gripperCallback(const std_msgs::String::ConstPtr& msg) {
    std::string command = msg->data;
    franka_gripper::MoveGoal goal;
    goal.speed = 0.05;

    if (command == "open") {
        if (gripper_open_) {
            ROS_INFO("[OFFSHORE] Gripper già aperto. Comando ignorato.");
            return;
        }

        goal.width = 0.08;
        ROS_INFO("[OFFSHORE] Comando ricevuto: APRI il gripper");
    }
    else if (command == "close") {
        if (!gripper_open_) {
            ROS_INFO("[OFFSHORE] Gripper già chiuso. Comando ignorato.");
            return;
        }

        goal.width = 0.029;
        ROS_INFO("[OFFSHORE] Comando ricevuto: CHIUDI il gripper");
    }
    else {
        ROS_WARN("[OFFSHORE] Comando gripper sconosciuto: %s", command.c_str());
        return;
    }

    // Send goal and wait for result with timeout
    gripper_client_->sendGoal(goal);
    bool finished_before_timeout = gripper_client_->waitForResult(ros::Duration(5.0));

    if (!finished_before_timeout) {
        ROS_ERROR("[OFFSHORE] Timeout nel comando al gripper!");
        gripper_client_->cancelGoal();
        return;
    }

    
    actionlib::SimpleClientGoalState state = gripper_client_->getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        gripper_open_ = (command == "open");
        ROS_INFO("[OFFSHORE] Gripper %s con successo.", gripper_open_ ? "aperto" : "chiuso");
    }
    else {
        ROS_ERROR("[OFFSHORE] Fallimento comando gripper: %s", state.getText().c_str());
        // Mantieni lo stato attuale per sicurezza
    }
}


/* ---------------------------------------------------------------------- */
/*                   Joint States Callback                                */
/* ---------------------------------------------------------------------- */

void jointStatesCallback(const sensor_msgs::JointStateConstPtr& msg) {
    // Calcolo il tempo in cui pubblicare (ritardo incluso)
    ros::Time release_time = msg->header.stamp + ros::Duration(return_delay_);
    
    // Log: ricezione e stamp temporale del messaggio
    ROS_INFO_STREAM("[OFFSHORE] jointStatesCallback: received at "
                    << msg->header.stamp.toSec()
                    << ", scheduled release at " << release_time.toSec());

    // Log su file: enqueue
    log_file_offshore << global_seq_id_offshore++ << ",joint_state,enqueue,"
                      << msg->header.stamp.toSec() << ","
                      << release_time.toSec() << ","
                      << return_delay_ << ","
                      << joint_states_buffer_.size() + 1 // quantità dopo push
                      << "\n";
    
    joint_states_buffer_.push_back({release_time, *msg});
}




/* ---------------------------------------------------------------------- */
/*                Timer Callback to Publish Delayed Joint States          */
/* ---------------------------------------------------------------------- */

void jointStatesTimerCallback(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();

    if (!joint_states_buffer_.empty()) {
        ROS_INFO("TIMER running, buffer size: %zu", joint_states_buffer_.size());
        ROS_INFO_STREAM("Now: " << now.toSec()
                        << " next release: " << joint_states_buffer_.front().first.toSec()
                        << " buffer size: " << joint_states_buffer_.size());
    }

    while (!joint_states_buffer_.empty() &&
           joint_states_buffer_.front().first <= now) {
        
         // Prelevo e pubblico
        auto entry = joint_states_buffer_.front();
        auto msg = entry.second;
        ros::Time scheduled = entry.first; //header.stamp + ros::Duration(return_delay_);
        ros::Time original_stamp = msg.header.stamp;
        
        msg.header.stamp = now; // Reimposto header al tempo attual
        joint_states_delayed_pub_.publish(msg);
        // Calcolo ritardo effettivo
        double actual_delay = (now - original_stamp).toSec();

        // Log su file: dequeue
        log_file_offshore << global_seq_id_offshore++ << ",joint_state,dequeue,"
                          << original_stamp.toSec() << ","
                          << now.toSec() << ","
                          << actual_delay << ","
                          << (joint_states_buffer_.size() - 1)// nuova dimensione
                          << "\n";

        // Rimuovo dall'elenco
        joint_states_buffer_.pop_front();
    }
}

/* ---------------------------------------------------------------------- */
/*                          Show Success Popup                            */
/* ---------------------------------------------------------------------- */


void showSuccessPopup() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "fr3_link0";  
    marker.header.stamp = ros::Time::now();
    marker.ns = "popup";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0.5; 
    marker.pose.position.y = 0;
    marker.pose.position.z = 1.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.z = 0.2;  
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.text = "Operation completed successfully!";
    marker.lifetime = ros::Duration(15.0);  
   
    // Wait for RViz subscriber
    while(marker_pub_.getNumSubscribers() < 1) {
        ros::Duration(0.1).sleep();
    }
    marker_pub_.publish(marker);
}

};

/* -------------------------------------------------------------------------- */
/*                                  MAIN                                        */
/* -------------------------------------------------------------------------- */
int main(int argc, char** argv) {
    ros::init(argc, argv, "offshore_node");
    OffshoreNode node;

    // Use async spinner with 2 threads (one for timers, one for subscribers)
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ros::waitForShutdown();
    return 0;
}