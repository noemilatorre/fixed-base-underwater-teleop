#include <ros/ros.h>
#include <lai_msgs/obj_info.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <deque>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <lai_msgs/PickAndPlaceAction.h>
#include <lai_msgs/PickAndPlaceGoal.h>

// Parameters
const int WINDOW_SIZE = 100;
const double OUTLIER_THRESHOLD = 0.008;
const double MIN_POSITION_Z = 0.05;
const double MAX_POSITION_Z = 0.09;
const double PICKING_TIME = 7.0;
const double PLACING_TIME = 7.0;
const double PICK_PLACE_POSITION_Z = 0.06;
const double MIN_DISTANCE_THRESHOLD = 0.02;

int obj_sent = 0;
ros::Publisher filtered_obj_pub;
typedef actionlib::SimpleActionClient<lai_msgs::PickAndPlaceAction> ActionClient;
bool position_sent = false;

// Structure used for filtering each object
struct ObjectFilter {
    std::string label;
    std::deque<double> x_window;
    std::deque<double> y_window;
    std::deque<double> z_window;
    
    ObjectFilter(const std::string& lbl) : label(lbl) {}
};

std::vector<ObjectFilter> object_filters;
// ------ Utility functions --------- //


// Compute the average of a window
// Returns 0.0 if empty

double getAverage(const std::deque<double>& window) {
    if(window.empty()) return 0.0;
    return std::accumulate(window.begin(), window.end(), 0.0) / window.size();
}

// Compute the median of a window
// Returns 0.0 if empty
double getMedian(std::deque<double> window) {
    if(window.empty()) return 0.0;
    std::sort(window.begin(), window.end());
    return (window.size() % 2 == 0) ? 
           (window[window.size()/2 - 1] + window[window.size()/2]) / 2.0 : 
           window[window.size()/2];
}


// Apply filtering to incoming point using median + average
// Outliers are replaced with the last valid value
geometry_msgs::Point applyFilter(ObjectFilter& filter, const geometry_msgs::Point& raw_point) {
geometry_msgs::Point filtered_point;
    
    if(!filter.x_window.empty()) {
        double median_x = getMedian(filter.x_window);
        double median_y = getMedian(filter.y_window);
        double median_z = getMedian(filter.z_window);
        // Outlier rejection
        if(fabs(raw_point.x - median_x) > OUTLIER_THRESHOLD ||
           fabs(raw_point.y - median_y) > OUTLIER_THRESHOLD ||
           fabs(raw_point.z - median_z) > OUTLIER_THRESHOLD) {
            ROS_WARN("Outlier rilevato per %s: (%.4f, %.4f, %.4f)", 
                    filter.label.c_str(), raw_point.x, raw_point.y, raw_point.z);
            filtered_point.x = filter.x_window.back();
            filtered_point.y = filter.y_window.back();
            filtered_point.z = filter.z_window.back();
            return filtered_point;
        }
    }
    // Update filter windows
    filter.x_window.push_back(raw_point.x);
    filter.y_window.push_back(raw_point.y);
    filter.z_window.push_back(raw_point.z);
    if(filter.x_window.size() > WINDOW_SIZE) {
        filter.x_window.pop_front();
        filter.y_window.pop_front();
        filter.z_window.pop_front();
    }
    // Return average values as filtered result
    filtered_point.x = getAverage(filter.x_window);
    filtered_point.y = getAverage(filter.y_window);
    filtered_point.z = getAverage(filter.z_window);
    return filtered_point;
}
// Get existing filter or create a new one
ObjectFilter& getOrCreateFilter(const std::string& label) {
    for(auto& filter : object_filters) {
        if(filter.label == label) {
            return filter;
        }
    }
    
    object_filters.emplace_back(label);
    return object_filters.back();
}


// Check if the object is too close to another one
// Returns true if distance < MIN_DISTANCE_THRESHOLD
bool isTooCloseToOtherObjects(const std::string& label, const geometry_msgs::Point& point) {
    for(const auto& filter : object_filters) {
        if(filter.label != label && !filter.x_window.empty()) {
            // Calcola la distanza
            double avg_x = getAverage(filter.x_window);
            double avg_y = getAverage(filter.y_window);
            double avg_z = getAverage(filter.z_window);
            
            double dx = point.x - avg_x;
            double dy = point.y - avg_y;
            double dz = point.z - avg_z;
            
            double distance = sqrt(dx*dx + dy*dy + dz*dz);
            
            if(distance < MIN_DISTANCE_THRESHOLD) {
                ROS_WARN("Oggetto %s scartato: troppo vicino a %s (distanza: %.4f m)", 
                        label.c_str(), filter.label.c_str(), distance);
                return true;
            }
        }
    }
    return false;
}


/// Publish all filtered objects as PickAndPlaceGoal messages
// Returns true if at least one object was published
bool publishFilteredObjects() {
    if (object_filters.size() == 0) {
        ROS_WARN("Nessun oggetto trovato da pubblicare");
        return false;
    }
    
    ROS_INFO("Oggetti trovati: %ld", object_filters.size());
    for (const auto& filter : object_filters) {
        ROS_INFO("- %s", filter.label.c_str());
    }

    ROS_INFO("--- Pubblicazione oggetti filtrati ---");
    
    // Pubblico tutti gli oggetti filtrati
    for (size_t i = 0; i < object_filters.size(); ++i) {
        if (!object_filters[i].x_window.empty()) {
            
            lai_msgs::PickAndPlaceGoal goal;
            
            // Pick position (filtered average)
            goal.pick_position.x = getAverage(object_filters[i].x_window);
            goal.pick_position.y = getAverage(object_filters[i].y_window) - 0.015;
            goal.pick_position.z = PICK_PLACE_POSITION_Z;
            
            // Orientamento per il pick
            goal.pick_orientation.x = 1.0;
            goal.pick_orientation.y = 0.0;
            goal.pick_orientation.z = 0.0;
            goal.pick_orientation.w = 0.0;
            
            // Tempo di pick
            goal.picking_time = PICKING_TIME;
            
           // Pick position (filtered average)
            goal.place_position.x = getAverage(object_filters[i].x_window);
            goal.place_position.y = -getAverage(object_filters[i].y_window);
            goal.place_position.z = PICK_PLACE_POSITION_Z;
            
            // Orientamento di place
            goal.place_orientation.x = 1.0;
            goal.place_orientation.y = 0.0;
            goal.place_orientation.z = 0.0;
            goal.place_orientation.w = 0.0;
            
            // Tempo di place
            goal.placing_time = PLACING_TIME;
            
            // Larghezza gripper
            goal.gripper_width = 0.029;
            
            // Pubblica il goal
            filtered_obj_pub.publish(goal);
            
            ROS_INFO("Goal pubblicato per oggetto %s:", object_filters[i].label.c_str());
            ROS_INFO("  Pick: x=%.4f, y=%.4f, z=%.4f", 
                goal.pick_position.x, goal.pick_position.y, goal.pick_position.z);
            ROS_INFO("  Place: x=%.4f, y=%.4f, z=%.4f", 
                goal.place_position.x, goal.place_position.y, goal.place_position.z);
            
            // Aspetta un po' prima del prossimo oggetto per non intasare
            ros::Duration(0.1).sleep();
        }
    }
    
    return true;
}


// Timer callback to publish objects once
void timerCallback(const ros::TimerEvent&) {
    if (!position_sent) {
        if (publishFilteredObjects()) {
            position_sent = true;
            ROS_INFO("Tutti gli oggetti filtrati sono stati pubblicati");
        } else {
            ROS_INFO("Nessun oggetto da pubblicare, riprovo...");
        }
    }
}

// Subscriber callback: process incoming objects and apply filtering
void objsInfoCallback(const lai_msgs::obj_info::ConstPtr& msg) {
    ROS_INFO("--- Nuovo messaggio ricevuto ---");
    for(size_t i = 0; i < msg->labels.size(); ++i) {
        geometry_msgs::Point point;
        point.x = msg->centers[i].cx;
        point.y = msg->centers[i].cy;
        point.z = msg->centers[i].cz;
        const geometry_msgs::Point& center = point;
        const std::string& label = msg->classes[i];
        const float position_z = msg->centers[i].cz;
        
        if(position_z < MIN_POSITION_Z || position_z > MAX_POSITION_Z) {
            ROS_WARN("Oggetto %s scartato", label.c_str());
            continue;
        }

        // Controllo prossimità con oggetti diversi
        if(isTooCloseToOtherObjects(label, center)) {
            continue;
        }
        
        ObjectFilter& filter = getOrCreateFilter(label);
        geometry_msgs::Point filtered_center = applyFilter(filter, center);
        
        ROS_INFO("Oggetto: %s", label.c_str());
    }
    ROS_INFO("-------------------------------");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "obj_position_pub_node");
    ros::NodeHandle nh;

    // Crea il client per l'action server
    ActionClient client("lai/pick_and_place", true);
    ROS_INFO("In attesa del server action...");
    client.waitForServer();
    ROS_INFO("Server action connesso!");
    
    // Subscriber for object info
    ros::Subscriber sub = nh.subscribe<lai_msgs::obj_info>("obj_info", 10, objsInfoCallback);
    
    // Publisher for filtered PickAndPlace goals
    filtered_obj_pub = nh.advertise<lai_msgs::PickAndPlaceGoal>("filtered_obj_position", 10);
    
    // Timer to trigger publishing after 7 seconds
    ros::Timer timer = nh.createTimer(ros::Duration(7.0), timerCallback, true);
    
    ROS_INFO("Nodo obj_position_pub avviato. Ascolto su 'obj_info', pubblico su 'filtered_obj_position'");
    
    ros::spin();
    return 0;
}