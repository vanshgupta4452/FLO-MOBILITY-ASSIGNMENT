#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <algorithm>

struct Point2D
{
    double x, y;

    Point2D(double x_ = 0.0, double y_ = 0.0)
        : x(x_), y(y_) {}

    double dist(const Point2D& other) const
    {
        return std::hypot(x - other.x, y - other.y);
    }
};



class PathSmoother
{
public:

    static std::vector<Point2D> smooth(
        const std::vector<Point2D>& points,
        double resolution = 0.05)
    {
        if (points.size() < 2)
            return points;

        std::vector<Point2D> result;

        for (size_t i = 0; i < points.size() - 1; i++)
        {
            double d = points[i].dist(points[i + 1]);

            if (d < 1e-6)
                continue;

            int steps =
                std::max(
                    1,
                    static_cast<int>(std::ceil(d / resolution)));

            for (int s = 0; s < steps; s++)
            {
                double t =
                    static_cast<double>(s) / steps;

                Point2D p;

                p.x =
                    points[i].x +
                    (points[i + 1].x - points[i].x) * t;

                p.y =
                    points[i].y +
                    (points[i + 1].y - points[i].y) * t;

                result.push_back(p);
            }
        }

        result.push_back(points.back());

        return result;
    }
};



class DynamicBubblePlanner : public rclcpp::Node
{
public:

    DynamicBubblePlanner()
        : Node("dynamic_bubble_planner")
    {
       

        declare_parameter(
            "path_file",
            "src/smoothing-rpp/waypoints/waypoint.csv");

        declare_parameter("frame_id", "odom");

        declare_parameter("path_resolution", 0.05);

        declare_parameter("safe_distance", 0.7);

        declare_parameter("detour_offset", 1.2);

        declare_parameter("lookahead_points", 80);


        frame_id_ =
            get_parameter("frame_id").as_string();

        path_resolution_ =
            get_parameter("path_resolution").as_double();

        safe_distance_ =
            get_parameter("safe_distance").as_double();

        detour_offset_ =
            get_parameter("detour_offset").as_double();

        lookahead_points_ =
            get_parameter("lookahead_points").as_int();


     

        tf_buffer_ =
            std::make_shared<tf2_ros::Buffer>(
                get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_);




        rclcpp::QoS qos(rclcpp::KeepLast(1));
        qos.reliable().transient_local();

        path_pub_ =
            create_publisher<nav_msgs::msg::Path>(
                "/path",
                qos);

        raw_path_pub_ =
            create_publisher<nav_msgs::msg::Path>(
                "/raw_path",
                qos);




        scan_sub_ =
            create_subscription<sensor_msgs::msg::LaserScan>(
                "/scan",
                10,
                std::bind(
                    &DynamicBubblePlanner::scanCallback,
                    this,
                    std::placeholders::_1));




        loadPathFromCSV();

        RCLCPP_INFO(
            get_logger(),
            "Dynamic Bubble Planner Started");
    }

private:



    std::string frame_id_;

    double path_resolution_;

    double safe_distance_;

    double detour_offset_;

    int lookahead_points_;

    bool path_loaded_ = false;

    bool avoidance_active_ = false;

    double robot_x_ = 0.0;

    double robot_y_ = 0.0;

    Point2D saved_obstacle_;

    std::vector<Point2D> global_path_;

    std::vector<Point2D> obstacles_;

    std::vector<Point2D> locked_path_;



    std::shared_ptr<tf2_ros::Buffer>
        tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener>
        tf_listener_;




    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
        path_pub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr
        raw_path_pub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_sub_;


    void loadPathFromCSV()
    {
        std::string file =
            get_parameter("path_file").as_string();

        std::ifstream in(file);

        if (!in.is_open())
        {
            RCLCPP_ERROR(
                get_logger(),
                "Cannot open CSV");

            return;
        }

        std::vector<Point2D> waypoints;

        std::string line;

        while (std::getline(in, line))
        {
            if (line.empty())
                continue;

            std::stringstream ss(line);

            double x, y;

            char comma;

            if (ss >> x >> comma >> y)
            {
                waypoints.emplace_back(x, y);
            }
        }

        if (waypoints.empty())
        {
            RCLCPP_ERROR(
                get_logger(),
                "No waypoints found");

            return;
        }

        publishPath(waypoints, raw_path_pub_);

        global_path_ =
            PathSmoother::smooth(
                waypoints,
                path_resolution_);

        publishPath(global_path_, path_pub_);

        path_loaded_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Loaded %zu path points",
            global_path_.size());
    }


    bool updateRobotPose()
    {
        try
        {
            geometry_msgs::msg::TransformStamped tf =
                tf_buffer_->lookupTransform(
                    frame_id_,
                    "base_link",
                    tf2::TimePointZero);

            robot_x_ = tf.transform.translation.x;

            robot_y_ = tf.transform.translation.y;

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            return false;
        }
    }


    int getClosestPathIndex()
    {
        double min_dist = 1e9;

        int best_idx = 0;

        for (size_t i = 0; i < global_path_.size(); i++)
        {
            double d = std::hypot(
                robot_x_ - global_path_[i].x,
                robot_y_ - global_path_[i].y);

            if (d < min_dist)
            {
                min_dist = d;

                best_idx = i;
            }
        }

        return best_idx;
    }



    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        obstacles_.clear();

        double angle = msg->angle_min;

        for (const auto& r : msg->ranges)
        {
            if (!std::isfinite(r))
            {
                angle += msg->angle_increment;
                continue;
            }

            if (r < 0.15 || r > 5.0)
            {
                angle += msg->angle_increment;
                continue;
            }

            geometry_msgs::msg::PointStamped laser_pt;

            laser_pt.header.frame_id =
                msg->header.frame_id;

            laser_pt.header.stamp =
                msg->header.stamp;

            laser_pt.point.x =
                r * std::cos(angle);

            laser_pt.point.y =
                r * std::sin(angle);

            laser_pt.point.z = 0.0;

            try
            {
                geometry_msgs::msg::PointStamped odom_pt;

                odom_pt =
                    tf_buffer_->transform(
                        laser_pt,
                        frame_id_,
                        tf2::durationFromSec(0.1));

                Point2D obs;

                obs.x = odom_pt.point.x;

                obs.y = odom_pt.point.y;

                obstacles_.push_back(obs);
            }
            catch (tf2::TransformException &ex)
            {
            }

            angle += msg->angle_increment;
        }

        generateAvoidancePath();
    }

    // ========================================================
    // SAME OBSTACLE
    // ========================================================

    bool sameObstacle(
        const Point2D& a,
        const Point2D& b)
    {
        return a.dist(b) < 0.3;
    }

    // ========================================================
    // MAIN PLANNER
    // ========================================================

    void generateAvoidancePath()
    {
        if (!path_loaded_)
            return;

        if (!updateRobotPose())
            return;

        int nearest_idx =
            getClosestPathIndex();

        int search_end =
            std::min(
                nearest_idx + lookahead_points_,
                (int)global_path_.size());

        bool found_collision = false;

        Point2D detected_obstacle;

        int start_idx = -1;

        int end_idx = -1;

        // ====================================================
        // CHECK ONLY LOCAL FORWARD PATH
        // ====================================================

        for (int i = nearest_idx;
             i < search_end;
             i++)
        {
            for (const auto& obs : obstacles_)
            {
                if (global_path_[i].dist(obs)
                    < safe_distance_)
                {
                    if (!found_collision)
                    {
                        detected_obstacle = obs;

                        found_collision = true;
                    }

                    if (start_idx == -1)
                        start_idx = i;

                    end_idx = i;
                }
            }
        }

        // ====================================================
        // NO COLLISION
        // ====================================================

        if (!found_collision)
        {
            avoidance_active_ = false;

            publishPath(global_path_, path_pub_);

            return;
        }

        // ====================================================
        // KEEP OLD AVOIDANCE PATH
        // ====================================================

        if (avoidance_active_)
        {
            if (sameObstacle(
                    detected_obstacle,
                    saved_obstacle_))
            {
                publishPath(
                    locked_path_,
                    path_pub_);

                return;
            }
        }

        // ====================================================
        // CREATE NEW DETOUR
        // ====================================================

        saved_obstacle_ = detected_obstacle;

        start_idx =
            std::max(
                nearest_idx,
                start_idx - 5);

        end_idx =
            std::min(
                (int)global_path_.size() - 1,
                end_idx + 5);

        std::vector<Point2D> new_path;

        // ====================================================
        // BEFORE OBSTACLE
        // ====================================================

        for (int i = 0; i < start_idx; i++)
        {
            new_path.push_back(global_path_[i]);
        }

        // ====================================================
        // DETOUR
        // ====================================================

        Point2D start =
            global_path_[start_idx];

        Point2D end =
            global_path_[end_idx];

        double dx = end.x - start.x;

        double dy = end.y - start.y;

        double len = std::hypot(dx, dy);

        if (len > 1e-6)
        {
            dx /= len;

            dy /= len;

            double px = -dy;

            double py = dx;

            int num_points = 40;

            for (int i = 0; i <= num_points; i++)
            {
                double t =
                    static_cast<double>(i)
                    / num_points;

                Point2D p;

                p.x =
                    start.x +
                    t * (end.x - start.x);

                p.y =
                    start.y +
                    t * (end.y - start.y);

                // sinusoidal local bubble
                double shift =
                    std::sin(t * M_PI)
                    * detour_offset_;

                p.x += px * shift;

                p.y += py * shift;

                new_path.push_back(p);
            }
        }

        // ====================================================
        // AFTER OBSTACLE
        // ====================================================

        for (size_t i = end_idx + 1;
             i < global_path_.size();
             i++)
        {
            new_path.push_back(global_path_[i]);
        }

        // ====================================================
        // FINAL SMOOTHING
        // ====================================================

        auto smooth_path =
            PathSmoother::smooth(
                new_path,
                path_resolution_);

        locked_path_ = smooth_path;

        avoidance_active_ = true;

        publishPath(
            locked_path_,
            path_pub_);
    }

    // ========================================================
    // PATH PUBLISH
    // ========================================================

    void publishPath(
        const std::vector<Point2D>& path,
        const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr& pub)
    {
        nav_msgs::msg::Path msg;

        msg.header.stamp = now();

        msg.header.frame_id = frame_id_;

        for (const auto& pt : path)
        {
            geometry_msgs::msg::PoseStamped pose;

            pose.header = msg.header;

            pose.pose.position.x = pt.x;

            pose.pose.position.y = pt.y;

            pose.pose.position.z = 0.0;

            pose.pose.orientation.w = 1.0;

            msg.poses.push_back(pose);
        }

        pub->publish(msg);
    }
};

// ============================================================
// MAIN
// ============================================================

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<DynamicBubblePlanner>());

    rclcpp::shutdown();

    return 0;
}