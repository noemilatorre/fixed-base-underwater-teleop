/* -------------------------------------------------------------------------- */
/*                                  INCLUDES                                  */
/* -------------------------------------------------------------------------- */

#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Accel.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/client/simple_action_client.h>
#include <lai_msgs/DesiredTrajectory.h>
#include <lai_msgs/PickAndPlaceAction.h>
#include <lai_msgs/PickAndPlaceGoal.h>
#include <ros_ultralytics/obj_info.h> 
#include <deque>
#include <string>
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <ctime>
#include <sstream>
#include <iomanip>

/* -------------------------------------------------------------------------- */
/*                          VARIABLES AND PARAMETERS                          */
/* -------------------------------------------------------------------------- */

static std::string session_folder;
size_t max_joint_state_buffer_size_ = 1000;
static std::ofstream log_file;
static uint64_t global_seq_id = 0;

/* -------------------------------------------------------------------------- */
/*                                    STRUCTURES                              */
/* -------------------------------------------------------------------------- */

// Structure to store timestamped desired trajectory messages
struct TimedMsg {
    ros::Time timestamp;
    lai_msgs::DesiredTrajectory msg;
};

// Structure to store timestamped pick-and-place goals
struct TimedGoal {
    ros::Time timestamp;
    lai_msgs::PickAndPlaceGoal goal;
};

// Structure to store timestamped gripper commands
struct TimedGripperCmd {
    ros::Time timestamp;
    std_msgs::String cmd;
};

// Structure to store delayed joint states from offshore
struct TimedJointState {
    ros::Time timestamp;
    sensor_msgs::JointState state;
};


/* -------------------------------------------------------------------------- */
/*                                ONSHORE NODE                                */
/* -------------------------------------------------------------------------- */

class OnshoreNode {
private:
    
    ros::NodeHandle nh_;

    /* ------------------------ Subscribers & Publishers ------------------- */
    ros::Subscriber cmd_vel_sub_, ee_pose_sub_, gripper_cmd_sub_; 
    ros::Subscriber joint_state_sub_;
    ros::Subscriber gui_goal_sub_, cube_sub_, control_mode_sub_; // nuovo: cubo
    ros::Publisher traj_pub_, gripper_pub_, goal_pub_, error_pub_;

    /* --------------------------- Timers & State --------------------------- */
    ros::Timer timer_;      // Timer to send delayed messages
    ros::Time last_time_;   // For delta time calculation
    bool last_time_initialized_ = false;

    geometry_msgs::Pose pose_des_;
    geometry_msgs::Twist velocity_des_;
    geometry_msgs::PoseStamped ee_pose_;

    bool ee_pose_initialized_ = false;
    bool initialized_ = false;  // True after first pose received
    
    /* ------------------------ Control Parameters --------------------------- */
    double delay_sec_ = 0.0;
    double forward_delay_ = 0.0;
    std::string control_mode_;  // "manual" o "auto"
    bool follow_mode_ = false;  // nuovo: attiva inseguimento
    // Aggiungi nella sezione privata della classe OnshoreNode
    geometry_msgs::Point last_cube_position_;
    bool cube_position_received_ = false;
    ros::Time last_cube_time_;
    double last_cube_confidence_ = 0.0;
    /* ------------------------------- Buffers ------------------------------ */
    std::deque<TimedMsg> traj_buffer_;  // Manual mode commands
    std::deque<TimedGoal> goal_buffer_; // Automatic mode commands
    std::deque<TimedGripperCmd> gripper_buffer_;
    std::deque<TimedJointState> joint_state_buffer_;

public:
    OnshoreNode() : nh_("~")  {
        nh_.param("delay_sec", delay_sec_, 0.0);
        nh_.param("forward_delay", forward_delay_, 0.0);
        //nh_.param<std::string>("control_mode", control_mode_, "auto");  // "manual", "auto", "follow"
        control_mode_ = "manual"; // Default mode

/* -------------------------------------------------------------------------- */
/*                                  LOGGING                                   */
/* -------------------------------------------------------------------------- */

        // Create session folder for logging
        if (session_folder.empty()) {
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::stringstream folder_name;

            std::string package_path = ros::package::getPath("franka_delay_node");
            folder_name << package_path << "/log_file/session_"
                        << std::put_time(&tm, "%Y%m%d_%H%M%S");
                      
            // Create the folder (0777 = all permissions)
            if (mkdir(folder_name.str().c_str(), 0777) != 0) {
                ROS_WARN_STREAM("[ONSHORE] Impossibile creare cartella: " << folder_name.str());
            }
            session_folder = folder_name.str();
        }

        // Open CSV log file
        std::stringstream filename;
        filename << session_folder << "/onshore_log" << "_mode_" << control_mode_
                        << "_delay_" << (delay_sec_ == 0.0 ? 0.0 : forward_delay_) << ".csv";
        log_file.open(filename.str());

        ROS_INFO_STREAM("[ONSHORE] Log salvato in: " << filename.str());

        // CSV Header 
        log_file << "seq_id,msg_type,event,stamp_send,stamp_recv,delay_s,buffer_size \n";
        log_file << std::fixed << std::setprecision(6);

        ROS_INFO_STREAM("[NODE] Log salvato in: " << filename.str());

    
        ROS_INFO_STREAM("Onshore avviato - delay: " << delay_sec_ 
                        << "s, forward: " << forward_delay_
                        << "s, mode: " << control_mode_);
        
/* ---------------------------------------------------------------------- */
/*                                SUBSCRIPTIONS                           */
/* ---------------------------------------------------------------------- */

        // Manual control subscriptions
        cmd_vel_sub_ = nh_.subscribe("/cmd_vel", 1, &OnshoreNode::cmdVelManualCallback, this); //teleop
        ee_pose_sub_ = nh_.subscribe("/operational_space_impedance_controller/pose_EE", 1, &OnshoreNode::eePoseCallback, this); //controllore
        gripper_cmd_sub_ = nh_.subscribe("/gripper_cmd", 1, &OnshoreNode::gripperCmdCallback, this); //teleop
        joint_state_sub_ = nh_.subscribe("/joint_states_delayed", 1, &OnshoreNode::jointStateCallback, this);
        // Automatic control subscription
        gui_goal_sub_ = nh_.subscribe("/gui_goal_msg", 1, &OnshoreNode::guiGoalCallback, this); // ricevo da GUI il goal (PickAndPlaceGoal)
        cube_sub_ = nh_.subscribe("/obj_info", 1, &OnshoreNode::objInfoCallback, this);
        
        control_mode_sub_ = nh_.subscribe("/control_mode", 1, &OnshoreNode::controlModeCallback, this);
        /* --------------------------- PUBLISHERS ----------------------- */
        traj_pub_ = nh_.advertise<lai_msgs::DesiredTrajectory>("/offshore/traj_cmd", 1);
        gripper_pub_ = nh_.advertise<std_msgs::String>("/offshore/gripper_cmd", 1);
        goal_pub_ = nh_.advertise<lai_msgs::PickAndPlaceGoal>("/offshore/pick_and_place_goal", 1);
        error_pub_ = nh_.advertise<std_msgs::Float64>("/tracking_error", 1);


        /* --------------------------- TIMER --------------------------- */
        // Timer to periodically send delayed messages if delay is active
        if (delay_sec_ > 0.5) {
                    timer_ = nh_.createTimer(ros::Duration(0.01), &OnshoreNode::bufferTimerCallback, this);
                }
        }

/* -------------------------------------------------------------------------- */
/*                                  CALLBACKS                                 */
/* -------------------------------------------------------------------------- */

// Callback per cambiare modalità da GUI
void controlModeCallback(const std_msgs::String::ConstPtr& msg) {
    std::string new_mode = msg->data;
    
    if (new_mode == "auto" || new_mode == "follow") {
        if (control_mode_ != new_mode) {
            control_mode_ = new_mode;
            ROS_INFO("[ONSHORE] Modalità cambiata a: %s", control_mode_.c_str());
            
            // Log del cambio modalità
            log_file << global_seq_id++ << ",control_mode,changed,,,," 
                     << control_mode_ << "\n";
        }
    } else if (new_mode == "manual") {
        ROS_WARN("[ONSHORE] Modalità manuale può essere attivata solo da terminale");
    } else {
        ROS_WARN("[ONSHORE] Tentativo di impostare modalità non valida: %s", new_mode.c_str());
    }
}

 void eePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        ee_pose_ = *msg;
        ee_pose_initialized_ = true;
        
        if (!initialized_) {
            pose_des_ = msg->pose;
            initialized_ = true;
            ROS_INFO("Posizione iniziale acquisita");
        }
    }    

// // Callback for manual velocity commands [manual]
    void cmdVelManualCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        if (control_mode_ != "manual") {
            ROS_WARN_THROTTLE(5, "[ONSHORE] Comando manuale ignorato: mod attuale = %s", control_mode_.c_str());
            return;
        }
        ros::Time now = ros::Time::now();
        if (!initialized_) {
            return;
        }
        if (!last_time_initialized_) {
            last_time_ = now;
            last_time_initialized_ = true;
            return;
        }

        double dt = (now - last_time_).toSec(); 
        last_time_ = now;
   
        if (dt > 0.2) dt = 0.05; // Safety

        // Scale velocities
        double scale = 2.0;  // mappa ±0.5 → ±1.0
        double max_speed_ = 0.1; // [m/s] max velocity
         
        // VELOCITÀ LINEARI (X, Y, Z)
        velocity_des_.linear.x = msg->linear.x * scale * max_speed_;
        velocity_des_.linear.y = msg->linear.y * scale * max_speed_;
        velocity_des_.linear.z = msg->linear.z * scale * max_speed_;
        
        // VELOCITÀ ANGOLARE (solo Z per la rotazione)
       // velocity_des_.angular.z = msg->angular.z * scale * max_speed_ * 2.0; // Moltiplicatore per rotazione
    
        // Integrate position
        //double dt = 0.05;  // (20Hz) periodo di campionamento
        pose_des_.position.x += velocity_des_.linear.x * dt;
        pose_des_.position.y += velocity_des_.linear.y * dt;
        pose_des_.position.z += velocity_des_.linear.z * dt;

        // Create trajectory message
        lai_msgs::DesiredTrajectory traj;
        traj.header.stamp = now;
        traj.pose_des = pose_des_;
        traj.velocity_des = velocity_des_;
        traj.acceleration_des = geometry_msgs::Accel();  // acc=0.0
     
        TimedMsg tmsg;
        tmsg.timestamp = now;
        tmsg.msg = traj;
        traj_buffer_.push_back(tmsg);

    
        // Logging and immediate publish if delay=0
        log_file << global_seq_id++ << ",traj,enqueue,"
                << now.toSec() << ",,"
                << "," << traj_buffer_.size() << "\n";
                
        if (delay_sec_ == 0.0) {
            traj_pub_.publish(traj);
            log_file << global_seq_id++ << ",traj,dequeue," 
                     << now.toSec() << "," << now.toSec()
                     << ",0.0," << traj_buffer_.size() << "\n";

            traj_buffer_.pop_back();  // Remove msgs
        } 

        if (!ee_pose_initialized_) {
        ROS_WARN_THROTTLE(5, "[ONSHORE] EE pose non disponibile");
        return;
    }
    
    

    // Calcolo errore di tracking
    double dx = last_cube_position_.x - ee_pose_.pose.position.x;
    double dy = last_cube_position_.y - ee_pose_.pose.position.y;
    double e_xy = std::sqrt(dx*dx + dy*dy);

    // LOGGING DELL'ERRORE PER TUTTE LE MODALITÀ
    logTrackingError(now, e_xy, dx, dy,
                    last_cube_position_.x, last_cube_position_.y, last_cube_position_.z,
                    ee_pose_.pose.position.x, ee_pose_.pose.position.y, ee_pose_.pose.position.z,
                    last_cube_confidence_);

        
    }

// Callback for gripper commands [manual]
    void gripperCmdCallback(const std_msgs::String::ConstPtr& msg) {
        ros::Time now = ros::Time::now();

        TimedGripperCmd tgcmd;
        tgcmd.timestamp = now;
        tgcmd.cmd = *msg;
        gripper_buffer_.push_back(tgcmd);

        log_file << global_seq_id++ << ",gripper,enqueue," 
                 << now.toSec() << ",," << "," << gripper_buffer_.size() << "\n";
        if (delay_sec_ == 0.0) {
            gripper_pub_.publish(*msg);
            log_file << global_seq_id++ << ",gripper,dequeue," 
                     << now.toSec() << "," << now.toSec()
                     << ",0.0," << gripper_buffer_.size() << "\n";
            gripper_buffer_.pop_back();
        }
}


// Callback for automatic pick-and-place goals from GUI
void guiGoalCallback(const lai_msgs::PickAndPlaceGoal::ConstPtr& msg) {
    ROS_INFO("[ONSHORE] Goal ricevuto nel callback");
    if (control_mode_ != "auto") {
        ROS_WARN_THROTTLE(5, "[ONSHORE] Goal automatico ignorato: mod attuale = %s", control_mode_.c_str());
        return;
    }
    ros::Time now = ros::Time::now();

    TimedGoal tgoal;
    tgoal.timestamp = now;
    tgoal.goal = *msg;
    goal_buffer_.push_back(tgoal);

    log_file << global_seq_id++ << ",goal,enqueue," 
                 << now.toSec() << ",," << "," << goal_buffer_.size() << "\n";

    if (delay_sec_ == 0.0) {
        goal_pub_.publish(tgoal.goal);
        log_file << global_seq_id++ << ",goal,dequeue," 
                     << now.toSec() << "," << now.toSec()
                     << ",0.0," << goal_buffer_.size() << "\n";
        goal_buffer_.pop_back();
    }
}

// void objInfoCallback(const ros_ultralytics::obj_info::ConstPtr& msg) {

//     if (!ee_pose_initialized_) {
//         ROS_WARN_THROTTLE(5, "[ONSHORE] EE pose non disponibile");
//         return;
//     }
//     if (msg->centers.empty()) return;

//     // Filtra solo i cubi verdi
//     std::vector<int> green_indices;
//     for (size_t i = 0; i < msg->colors.size(); i++) {
        
//         if (msg->colors[i] == "GREEN") { 
//             green_indices.push_back(i);
//         }
//     }

//     if (green_indices.empty()) {
//         ROS_WARN_THROTTLE(2, "[ONSHORE] Nessun cubo verde rilevato");
//         return;
//     }

//     int selected_index = green_indices[0];
//     auto center = msg->centers[selected_index];
//     ros::Time now = ros::Time::now();

//      // Aggiorna posizione cubo con filtro esponenziale (smoothing)
//     double alpha = 0.2; // coefficiente di smoothing
//     last_cube_position_.x = alpha * center.cx + (1 - alpha) * last_cube_position_.x;
//     last_cube_position_.y = alpha * center.cy + (1 - alpha) * last_cube_position_.y;
//     last_cube_position_.z = alpha * center.cz + (1 - alpha) * last_cube_position_.z;

//     last_cube_time_ = now;
//     last_cube_confidence_ = msg->confidences[selected_index];

//     //AGGIUNGO: Timeout: se il cubo non viene visto da >0.5s, non muovere il robot
//     if ((now - last_cube_time_).toSec() > 0.5) {
//         ROS_WARN_THROTTLE(1, "[FOLLOW] Cubo perso: fermo il robot");
//         return;
//     }

//     // Calcolo errore di tracking
//     double dx = center.cx - ee_pose_.pose.position.x;
//     double dy = center.cy - ee_pose_.pose.position.y;
//     double e_xy = std::sqrt(dx*dx + dy*dy);

//     // Rate limiter: massima distanza per ciclo (ad ogni callback)
//     double max_step = 0.02; // max 2 cm per ciclo
//     if (e_xy > max_step) {
//         dx = dx * (max_step / e_xy);
//         dy = dy * (max_step / e_xy);
//         e_xy = max_step;
//     }

//     if(control_mode_ == "manual" || control_mode_ == "auto") {
//         ROS_WARN_THROTTLE(5, "[ONSHORE] Tracking ignorato: mod attuale = %s", control_mode_.c_str());
//         // MEMORIZZA LA POSIZIONE DEL CUBO (AGGIUNGI QUESTE RIGHE)
//         last_cube_position_.x = center.cx;
//         last_cube_position_.y = center.cy;
//         last_cube_position_.z = center.cz;
//         last_cube_confidence_ = msg->confidences[selected_index];
//         //cube_position_received_ = true;
//         last_cube_time_ = ros::Time::now();
//         return;
//     }
//     // LOGGING DELL'ERRORE PER TUTTE LE MODALITÀ
//     logTrackingError(now, e_xy, dx, dy,
//                     center.cx, center.cy, center.cz,
//                     ee_pose_.pose.position.x, ee_pose_.pose.position.y, ee_pose_.pose.position.z,
//                     msg->confidences[selected_index]);


//     // CALCOLO VELOCITÀ DESIDERATA CON LIMITI
//     double max_speed = 0.03;  // 10 cm/s massimo
//     double kp = 0.2;         // Guadagno proporzionale
    
//     geometry_msgs::Twist velocity_des;
//     velocity_des.linear.x = kp * dx;
//     velocity_des.linear.y = kp * dy;
//     velocity_des.linear.z = 0.0;  // Mantieni altezza fissa
    
//     // Limita la velocità
//     double current_speed = sqrt(velocity_des.linear.x * velocity_des.linear.x + 
//                                velocity_des.linear.y * velocity_des.linear.y);
    
//     //if (current_speed > max_speed) {
//         velocity_des.linear.x = velocity_des.linear.x * (max_speed / current_speed);
//         velocity_des.linear.y = velocity_des.linear.y * (max_speed / current_speed);
//     //}

    
// // CALCOLO POSIZIONE DESIDERATA (per mantenere la traiettoria)
//     geometry_msgs::Pose target_pose;
    
//     if (e_xy < 0.08) {
//         target_pose.position.x = center.cx;
//         target_pose.position.y = center.cy;
//         target_pose.position.z = ee_pose_.pose.position.z; // mantieni altezza
//     } else {
//         target_pose.position.x = center.cx - 0.08 * (dx / e_xy);
//         target_pose.position.y = center.cy - 0.08 * (dy / e_xy);
//         target_pose.position.z = ee_pose_.pose.position.z;
//     }
//     target_pose.orientation = ee_pose_.pose.orientation;

//     // Crea messaggio di traiettoria
//     lai_msgs::DesiredTrajectory traj;
//     traj.header.stamp = now;
//     traj.pose_des = target_pose;
//     traj.velocity_des = velocity_des;  

//     // IMPORTANTE: imposta accelerazione ZERO!
//     traj.acceleration_des.linear.x = 0.0;
//     traj.acceleration_des.linear.y = 0.0;
//     traj.acceleration_des.linear.z = 0.0;
//     traj.acceleration_des.angular.x = 0.0;
//     traj.acceleration_des.angular.y = 0.0;
//     traj.acceleration_des.angular.z = 0.0;
    
//     //traj.acceleration_des = geometry_msgs::Accel();  // accelerazione zero


// //     lai_msgs::DesiredTrajectory traj;
// //     traj.header.stamp = now;
    
// //     if (e_xy < 0.08)
// //     {
// //         traj.pose_des.position.x = center.cx;        // target X del cubo
// //         traj.pose_des.position.y = center.cy;        // target Y del cubo
// // //        traj.pose_des.position.z = center.cz + 0.08; // 5 cm sopra il cubo
// //         traj.pose_des.position.z = ee_pose_.pose.position.z; // mantieni altezza corrente
// //         traj.pose_des.orientation = ee_pose_.pose.orientation;
// //     }
// //     else
// //     {
// //         traj.pose_des.position.x = center.cx - 0.08 * (dx / e_xy); // mantieni distanza di 8 cm
// //         traj.pose_des.position.y = center.cy - 0.08 * (dy / e_xy); // mantieni distanza di 8 cm
// //         traj.pose_des.position.z = ee_pose_.pose.position.z; // mantieni altezza corrente
// //         traj.pose_des.orientation = ee_pose_.pose.orientation;
// //     }
    
// //     // velocità e accelerazioni = 0
// //     traj.velocity_des = geometry_msgs::Twist();
// //     traj.acceleration_des = geometry_msgs::Accel();
    

// // Inserisci in buffer (FIFO)
//     TimedMsg tmsg;
//     tmsg.timestamp = now;
//     tmsg.msg = traj;
//     traj_buffer_.push_back(tmsg);

//     log_file << global_seq_id++ << ",traj_follow,enqueue,"
//              << now.toSec() << ",,"
//              << "," << traj_buffer_.size() << "\n";

//     if (delay_sec_ == 0.0) {
//         traj_pub_.publish(traj);
//         log_file << global_seq_id++ << ",traj_follow,dequeue,"
//                  << now.toSec() << "," << now.toSec()
//                  << ",0.0," << traj_buffer_.size() << "\n";
//     }

//     // Pubblicazione errore per monitoring in tempo reale
//     std_msgs::Float64 err_msg;
//     err_msg.data = e_xy;
//     error_pub_.publish(err_msg);
//     ROS_INFO_STREAM_THROTTLE(1, "[FOLLOW] Errore e_xy = " << e_xy << " m");
// }

/* Callback for object tracking (follow mode)
 - Selects only green cubes within a valid height range
 - Smooths cube position with an exponential filter
 - Rejects sudden jumps (false positives)
 - Computes tracking error (cube vs end-effector)
 - Generates trajectory command if in "follow" mode
 - Stops robot if cube is lost for too long
*/
void objInfoCallback(const ros_ultralytics::obj_info::ConstPtr& msg) {
    static ros::Time last_callback_time = ros::Time::now();
    static geometry_msgs::Twist last_velocity;
    
    ros::Time now = ros::Time::now();
    double dt = (now - last_callback_time).toSec();
    last_callback_time = now;
    static const double MAX_JUMP_DISTANCE = 0.08; // reject jumps > 8 cm 
    static const double MIN_CUBE_HEIGHT = 0.05;   // reject cubes lower than 5 cm
    static const double MAX_CUBE_HEIGHT = 0.08;   // reject cubes higher than8 cm  

    if (dt > 0.2) dt = 0.05; // Safety limit

    // Require end-effector pose to be initialized
    if (!ee_pose_initialized_) {
        ROS_WARN_THROTTLE(5, "[ONSHORE] EE pose non disponibile");
        return;
    }
    
    if (msg->centers.empty()) {
        // No objects detected
        handleLostCube(now);
        return;
    }

     // Filter: keep only green cubes within valid height range
    std::vector<int> valid_indices;
    for (size_t i = 0; i < msg->colors.size(); i++) {
        if (msg->colors[i] == "GREEN" && msg->centers[i].cz >= MIN_CUBE_HEIGHT && msg->centers[i].cz <= MAX_CUBE_HEIGHT) { 
            valid_indices.push_back(i);
            ROS_INFO_THROTTLE(2, "[FOLLOW] Cubo verde valido rilevato: z=%.3f m", msg->centers[i].cz);
        } else if (msg->colors[i] == "GREEN") {
            ROS_WARN_THROTTLE(1, "[FOLLOW] Cubo verde scartato: altezza z=%.3f m non in range [%.3f, %.3f] m", 
                             msg->centers[i].cz, MIN_CUBE_HEIGHT, MAX_CUBE_HEIGHT);
        }
    }

    if (valid_indices.empty()) {
        ROS_WARN_THROTTLE(2, "[ONSHORE] Nessun cubo verde valido rilevato (altezza realistica)");
        handleLostCube(now);
        return;
    }

    // Select cube with highest confidence
    int selected_index = valid_indices[0];
    double max_confidence = msg->confidences[selected_index];
    for (size_t i = 1; i < valid_indices.size(); i++) {
        if (msg->confidences[valid_indices[i]] > max_confidence) {
            selected_index = valid_indices[i];
            max_confidence = msg->confidences[valid_indices[i]];
        }
    }

    auto center = msg->centers[selected_index];
    last_cube_confidence_ = max_confidence;

    // Reject cube if its height is out of valid range
    if (center.cz < MIN_CUBE_HEIGHT || center.cz > MAX_CUBE_HEIGHT) {
        ROS_WARN_THROTTLE(1, "[FOLLOW] Cubo scartato: altezza z=%.3f m non in range [%.3f, %.3f] m", 
                         center.cz, MIN_CUBE_HEIGHT, MAX_CUBE_HEIGHT);
        handleLostCube(now);
        return;
    }

    // Exponential smoothing of cube position (reduces noise)
    double time_since_last_seen = (now - last_cube_time_).toSec();
    double alpha = 0.4; 
    double adaptive_alpha = alpha * std::min(1.0, time_since_last_seen * 2.0);
    
    last_cube_position_.x = adaptive_alpha * center.cx + (1 - adaptive_alpha) * last_cube_position_.x;
    last_cube_position_.y = adaptive_alpha * center.cy + (1 - adaptive_alpha) * last_cube_position_.y;
    last_cube_position_.z = adaptive_alpha * center.cz + (1 - adaptive_alpha) * last_cube_position_.z;
    
    last_cube_time_ = now;

    // Compute tracking error (cube vs EE)
    double dx = last_cube_position_.x - ee_pose_.pose.position.x;
    double dy = last_cube_position_.y - ee_pose_.pose.position.y;
    double e_xy = std::sqrt(dx*dx + dy*dy);

    // Log tracking error
    logTrackingError(now, e_xy, dx, dy,
                    last_cube_position_.x, last_cube_position_.y, last_cube_position_.z,
                    ee_pose_.pose.position.x, ee_pose_.pose.position.y, ee_pose_.pose.position.z,
                    last_cube_confidence_);

    // Publish error for monitoring
    std_msgs::Float64 err_msg;
    err_msg.data = e_xy;
    error_pub_.publish(err_msg);
    
    if (control_mode_ != "follow") {
        ROS_WARN_THROTTLE(5, "[ONSHORE] Tracking ignorato: mod attuale = %s", control_mode_.c_str());
        return;
    }

    // Reject sudden jumps (false positives)
    double jump_distance = std::sqrt(
        std::pow(last_cube_position_.x - center.cx, 2) +
        std::pow(last_cube_position_.y - center.cy, 2) +
        std::pow(last_cube_position_.z - center.cz, 2)
    );
    
    if (jump_distance > MAX_JUMP_DISTANCE) {
        ROS_WARN("[FOLLOW] Salto troppo grande (%.2f m)! Ignoro cubo sospetto.", jump_distance);
        
        // Ripristino la posizione precedente del cubo
        last_cube_position_.x = last_cube_position_.x; 
        last_cube_position_.y = last_cube_position_.y;
        last_cube_position_.z = last_cube_position_.z;
        
        handleLostCube(now);
        return;
    }

     // Proportional velocity control with saturation and dead zone
    double max_speed = 0.05;  // max speed = 5 cm/s <
    double kp = 0.25;         // proportional gain
    double dead_zone = 0.005; // do not move if error < 0.5 cm
    
    geometry_msgs::Twist velocity_des;
    
    if (e_xy < dead_zone) {
        // Dead zone: velocità zero per evitare micro-movimenti
        velocity_des.linear.x = 0;
        velocity_des.linear.y = 0;
        ROS_INFO_THROTTLE(2, "[FOLLOW] In dead zone, errore: %.3f m", e_xy);
    } else {
        double adaptive_kp = kp;
        if (e_xy < 0.1) {
            adaptive_kp = kp * (e_xy / 0.1);  // slow down near target
        }
        
        velocity_des.linear.x = adaptive_kp * dx;
        velocity_des.linear.y = adaptive_kp * dy;
        
        // Limit velocity
        double current_speed = std::sqrt(velocity_des.linear.x * velocity_des.linear.x + 
                                       velocity_des.linear.y * velocity_des.linear.y);
        
        if (current_speed > max_speed) {
            velocity_des.linear.x = velocity_des.linear.x * (max_speed / current_speed);
            velocity_des.linear.y = velocity_des.linear.y * (max_speed / current_speed);
        }
    }
    
    velocity_des.linear.z = 0.0;
    last_velocity = velocity_des;

    // Target pose = current cube XY
    geometry_msgs::Pose target_pose;
    
    target_pose.position.x = last_cube_position_.x;
    target_pose.position.y = last_cube_position_.y;
    target_pose.position.z = ee_pose_.pose.position.z; // CAMBIA
    target_pose.orientation = ee_pose_.pose.orientation;

    // Build trajectory message
    lai_msgs::DesiredTrajectory traj;
    traj.header.stamp = now;
    traj.pose_des = target_pose;
    traj.velocity_des = velocity_des;
    
    // zero acceleration
    traj.acceleration_des.linear.x = 0.0;
    traj.acceleration_des.linear.y = 0.0;
    traj.acceleration_des.linear.z = 0.0;
    traj.acceleration_des.angular.x = 0.0;
    traj.acceleration_des.angular.y = 0.0;
    traj.acceleration_des.angular.z = 0.0;

    // Push trajectory into buffer
    TimedMsg tmsg;
    tmsg.timestamp = now;
    tmsg.msg = traj;
    traj_buffer_.push_back(tmsg);

    log_file << global_seq_id++ << ",traj_follow,enqueue,"
             << now.toSec() << ",,"
             << "," << traj_buffer_.size() << "\n";

    if (delay_sec_ == 0.0) {
        traj_pub_.publish(traj);
        log_file << global_seq_id++ << ",traj_follow,dequeue,"
                 << now.toSec() << "," << now.toSec()
                 << ",0.0," << traj_buffer_.size() << "\n";
    }

    ROS_INFO_STREAM_THROTTLE(1, "[FOLLOW] Errore: " << std::fixed << std::setprecision(3) << e_xy 
                            << " m, Vel: (" << std::setprecision(3) << velocity_des.linear.x 
                            << ", " << velocity_des.linear.y << ") m/s, Cubo z: " 
                            << std::setprecision(3) << center.cz << " m");
}


// Called when the cube is not detected for >0.5 s
// Publishes a stop command to hold the current end-effector pose

void handleLostCube(const ros::Time& now) {
    double time_since_last_seen = (now - last_cube_time_).toSec();
    
    if (time_since_last_seen > 0.5 && control_mode_ == "follow") {
        // Ferma gradualmente il robot
        geometry_msgs::Twist stop_vel;
        stop_vel.linear.x = 0;
        stop_vel.linear.y = 0;
        stop_vel.linear.z = 0;
        
        lai_msgs::DesiredTrajectory stop_cmd;
        stop_cmd.header.stamp = now;
        stop_cmd.pose_des = ee_pose_.pose; // Mantieni posizione corrente
        stop_cmd.velocity_des = stop_vel;
        stop_cmd.acceleration_des = geometry_msgs::Accel();
        
        TimedMsg tmsg;
        tmsg.timestamp = now;
        tmsg.msg = stop_cmd;
        traj_buffer_.push_back(tmsg);
        
        if (delay_sec_ == 0.0) {
            traj_pub_.publish(stop_cmd);
        }
        
        ROS_WARN_THROTTLE(1, "[FOLLOW] Cubo perso: robot fermo");
    }
}

// Callback to periodically check buffers and send delayed messages
void bufferTimerCallback(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();
    while (!traj_buffer_.empty() && (now - traj_buffer_.front().timestamp).toSec() >= forward_delay_) {
        
        ros::Duration actual_delay = now - traj_buffer_.front().timestamp;
        ROS_INFO_STREAM("[ONSHORE] Inviato messaggio. Timestamp ricezione: "
                << traj_buffer_.front().timestamp.toSec()
                << ", timestamp invio: " << now.toSec()
                << ", delay effettivo: " << actual_delay.toSec() << " s");

        traj_pub_.publish(traj_buffer_.front().msg);
        log_file << global_seq_id++ << ",traj,dequeue,"
                     << traj_buffer_.front().timestamp.toSec() << ","
                     << now.toSec() << ","
                     << actual_delay.toSec() << ","
                     << traj_buffer_.size() << "\n";
        
        traj_buffer_.pop_front();
    }

    while (!goal_buffer_.empty() && (now - goal_buffer_.front().timestamp).toSec() >= forward_delay_) {
        ros::Duration actual_delay = now - goal_buffer_.front().timestamp;
        ROS_INFO_STREAM("[ONSHORE] Invio goal ritardato al nodo offshore. Timestamp ricezione: "
                        << goal_buffer_.front().timestamp.toSec()
                        << ", invio: " << now.toSec()
                        << ", delay effettivo: " << actual_delay.toSec() << " s");

        goal_pub_.publish(goal_buffer_.front().goal);
        log_file << global_seq_id++ << ",goal,dequeue,"
                     << goal_buffer_.front().timestamp.toSec() << ","
                     << now.toSec() << ","
                     << actual_delay.toSec() << ","
                     << goal_buffer_.size() << "\n";

        goal_buffer_.pop_front();
    }

    while (!gripper_buffer_.empty() && (now - gripper_buffer_.front().timestamp).toSec() >= forward_delay_) {

        ros::Duration actual_delay = now - gripper_buffer_.front().timestamp;
        ROS_INFO_STREAM("[ONSHORE] Inviato comando gripper. Timestamp ricezione: "
                        << gripper_buffer_.front().timestamp.toSec()
                        << ", invio: " << now.toSec()
                        << ", delay effettivo: " << actual_delay.toSec() << " s");

        gripper_pub_.publish(gripper_buffer_.front().cmd);
        log_file << global_seq_id++ << ",gripper,dequeue,"
                     << gripper_buffer_.front().timestamp.toSec() << ","
                     << now.toSec() << ","
                     << actual_delay.toSec() << ","
                     << gripper_buffer_.size() << "\n";
        gripper_buffer_.pop_front();
    }
       
}
 // Callback to receive delayed joint states from offshore
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    TimedJointState tjs;
    tjs.timestamp = ros::Time::now();  
    tjs.state = *msg;
    joint_state_buffer_.push_back(tjs);

    log_file << global_seq_id++ << ",joint_state,recv,"
             << msg->header.stamp.toSec() << ","   
             << tjs.timestamp.toSec() << ","
             << (tjs.timestamp - msg->header.stamp).toSec() << ","
             << joint_state_buffer_.size() << "\n";

             while (joint_state_buffer_.size() > max_joint_state_buffer_size_) {
        joint_state_buffer_.pop_front();
    }
            
}

void logTrackingError(const ros::Time& timestamp, 
                     double error_xy, double dx, double dy,
                     double cube_x, double cube_y, double cube_z,
                     double ee_x, double ee_y, double ee_z,
                     double confidence = 0.0) {
    
    static std::ofstream error_log_file;
    static bool error_file_initialized = false;
    
    if (!error_file_initialized) {
        std::stringstream error_filename;
        error_filename << session_folder << "/tracking_error_log.csv";
        error_log_file.open(error_filename.str());
        error_log_file << "timestamp,error_xy,dx,dy,cube_x,cube_y,cube_z,ee_x,ee_y,ee_z,confidence,control_mode\n";
        error_log_file << std::fixed << std::setprecision(6);
        error_file_initialized = true;
        ROS_INFO_STREAM("[ONSHORE] Error log salvato in: " << error_filename.str());
    }
    
    // Scrive i dati dell'errore
    error_log_file << timestamp.toSec() << ","
                   << error_xy << ","
                   << dx << ","
                   << dy << ","
                   << cube_x << ","
                   << cube_y << ","
                   << cube_z << ","
                   << ee_x << ","
                   << ee_y << ","
                   << ee_z << ","
                   << confidence << ","
                   << control_mode_ << "\n";
}

};

/* -------------------------------------------------------------------------- */
/*                                    MAIN                                    */
/* -------------------------------------------------------------------------- */

int main(int argc, char** argv) {
    ros::init(argc, argv, "onshore_node");
    OnshoreNode node;
    ros::spin();
    return 0;
}
