#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>

// Dependencias de mensajes de PX4
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>

using namespace std::chrono_literals;

class OffboardControl : public rclcpp::Node {
public:
    OffboardControl() : Node("offboard_control"), offboard_setpoint_counter_(0) {
        
        // 1. Declaración de Publicadores
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        // 2. Temporizador a 20 Hz (50 ms)
        timer_ = this->create_wall_timer(50ms, std::bind(&OffboardControl::timer_callback, this));
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;
    
    uint64_t offboard_setpoint_counter_; // Contador para la regla de los 500ms

    void timer_callback() {
        // Enviar el tipo de control y la coordenada SIEMPRE (Flujo continuo)
        publish_offboard_control_mode();
        publish_trajectory_setpoint();

        // 3. Lógica de Armado y Offboard
        if (offboard_setpoint_counter_ == 20) {
            // Después de 1 segundo de latidos, armamos y tomamos control
            this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6); // 6 es el ID del modo Offboard
            this->arm();
        }

        if (offboard_setpoint_counter_ < 21) {
            offboard_setpoint_counter_++;
        }
    }

    void arm() {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "¡Comando de Armado enviado!");
    }

    void publish_offboard_control_mode() {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true; // Le decimos a PX4 que vamos a controlar la posición
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void publish_trajectory_setpoint() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        // Despegue a 5 metros de altura (Recuerda: Eje Z invertido en NED)
        msg.position = {0.0, 0.0, -5.0};
        msg.yaw = -3.14; // Mirando hacia la dirección opuesta (180 grados) (aproximadamente)
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg{};
        msg.param1 = param1;
        msg.param2 = param2;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        vehicle_command_publisher_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardControl>());
    rclcpp::shutdown();
    return 0;
}