#include "geometry_msgs/msg/detail/polygon__struct.hpp"
#include "rclcpp/timer.hpp"
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/polygon.hpp>
#include <attach_shelf_interfaces/srv/change_footprint_polygon.hpp>

using Polygon = geometry_msgs::msg::Polygon;
using ChangeFootprintPolygon = attach_shelf_interfaces::srv::ChangeFootprintPolygon;
using namespace std::chrono_literals;

namespace nav2_apps{
    class ChangeFootprintService : public rclcpp::Node{
        public:
            ChangeFootprintService(const rclcpp::NodeOptions & nodeOptions) : Node("change_footprint_service", nodeOptions){
                this -> local_publisher_ = this -> create_publisher<Polygon>("/local_costmap/footprint", 10);
                this -> global_publisher_ = this -> create_publisher<Polygon>("/global_costmap/footprint", 10);
                this -> service_ = this -> create_service<ChangeFootprintPolygon>("/change_footprint", 
                [this](const ChangeFootprintPolygon::Request::SharedPtr request, const ChangeFootprintPolygon::Response::SharedPtr response){
                    this -> timer_ -> cancel();
                  

                    this -> timer_ = this -> create_wall_timer(250ms, [this, request](){
                        this -> local_publisher_ -> publish(request -> polygon);
                        this -> global_publisher_ -> publish(request -> polygon);
                    });
                    response -> success = true;
                });
                
            }
        protected:
            rclcpp::Service<ChangeFootprintPolygon>::SharedPtr service_;
            
            rclcpp::Publisher<Polygon>::SharedPtr local_publisher_;
            rclcpp::Publisher<Polygon>::SharedPtr global_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;

            Polygon polygon;
    };

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_apps::ChangeFootprintService)
