#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

enum JoyStick {
    M_PLUS_MINUS = 3,
    ACT_SELECT = 4,
    ACT_SELECT_MOTOR = 5,
    ACT_DECREASE_SPEED = 1,
    ACT_INCREASE_SPEED = 3
};

class IndArmController : public rclcpp::Node
{
public:
    IndArmController(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~IndArmController();

private:
    // Timer for publishing joint states
    void init_timer();
    void joint_state_pub_timer_callback();

    // Publishers, subscribers, and services initialization
    void init_subs_pubs_srvs();
    void joy_callback_(sensor_msgs::msg::Joy::ConstSharedPtr joy);
    
    // Teleop service initialization
    bool init_teleop_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    // Control logic for motors
    void toggle_motor_id_for_rotation();
    void selected_the_toggled_motor_id();
    void increase_motor_speed();
    void decrese_motor_speed();
    void change_motor_direction();

    // Logger
    void log(const std::string &message) { RCLCPP_INFO(this->get_logger(), "%s", message.c_str()); }
    void log_num(int num) { RCLCPP_INFO(this->get_logger(), "%d", num); }

    // Publishers, subscribers, services, and variables
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr init_teleop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_teleop_srv_;
    rclcpp::TimerBase::SharedPtr joint_state_pub_timer_;

    sensor_msgs::msg::JointState target_joint_state_;
    int motor_num;
    int toggle_motor_id = 1;
    int motor_select_;
    double step_size_;
    double max_speed;
    double publish_rate;
    double pos;
    bool init_teleop_;
    int button_value_ACT_SELECT_MOTOR, button_value_ACT_TOGGLE_MOTOR, button_value_ACT_INCREASE_SPEED, button_value_ACT_DECREASE_SPEED, button_value_M_PLUS_MINUS;
    bool debug_;
    double high, low;
    
    std::string log_header_;
    
    // Motor position array
    std::vector<double> motor_positions_;
};

IndArmController::IndArmController(const rclcpp::NodeOptions &options)
    : Node("my_arm_control"), motor_positions_(4, 0.0)  // Initialize motor positions to size 4 with zeroes
{
    debug_ = false;
    max_speed = 10;
    init_teleop_ = false;
    button_value_M_PLUS_MINUS = 0;
    button_value_ACT_INCREASE_SPEED = 0;
    button_value_ACT_DECREASE_SPEED = 0;
    button_value_ACT_TOGGLE_MOTOR = 0;
    button_value_ACT_SELECT_MOTOR = 0;
    motor_select_ = 1;
    step_size_ = 1.0;
    publish_rate = 0.1;
    log("Flags are set");
int ret_into_motion_mode_pos = 0;
int ret_into_motion_enable= 0;
    high = 628;
    low = -628;
    motor_num = 2;
    log_header_ = "[IndArmController]: ";
    
    target_joint_state_.name.resize(4);
    target_joint_state_.position.resize(4);
    target_joint_state_.velocity.resize(4);
    target_joint_state_.effort.resize(4);

    init_subs_pubs_srvs();
    init_timer();

    if (debug_)
        RCLCPP_INFO(this->get_logger(), "%sInitialized", log_header_.c_str());
}

IndArmController::~IndArmController()
{
    if (debug_)
        RCLCPP_INFO(this->get_logger(), "%sDestructor called", log_header_.c_str());
}

void IndArmController::init_timer()
{
    joint_state_pub_timer_ = rclcpp::create_timer(
        this->get_node_base_interface(),
        this->get_node_timers_interface(),
        this->get_clock(),
        rclcpp::Duration(0.01, 0),
        std::bind(&IndArmController::joint_state_pub_timer_callback, this));
}

void IndArmController::init_subs_pubs_srvs()
{
    log("Initializing Subscribers and Publishers");
    
    // Publisher for joint commands
    joint_command_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 1,
        std::bind(&IndArmController::joy_callback_, this, std::placeholders::_1));
}

bool IndArmController::init_teleop_srv_callback_(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    log("Initializing teleop");
    target_joint_state_.header.stamp = this->now();
    for (int i = 1; i <= motor_num; i++)
    {
        target_joint_state_.position[i - 1] = pos;
    }
    joint_state_pub_->publish(target_joint_state_);
    init_teleop_ = true;
    res->success = true;
    res->message = "Successfully Initialized Teleop";
    return true;
}

void IndArmController::toggle_motor_id_for_rotation()
{
    static int button_pressed_ACT_TOGGLE_MOTOR = 0;

    if (button_value_ACT_TOGGLE_MOTOR == 1 && button_pressed_ACT_TOGGLE_MOTOR == 0)
    {
        button_pressed_ACT_TOGGLE_MOTOR = 1;
        toggle_motor_id = (toggle_motor_id + 1) % (motor_num + 1);
        log("Toggle_motor_id to:");
        log_num(toggle_motor_id);
    }
    else if (button_value_ACT_TOGGLE_MOTOR == 0 && button_pressed_ACT_TOGGLE_MOTOR == 1)
    {
        button_pressed_ACT_TOGGLE_MOTOR = 0;
    }
}

void IndArmController::selected_the_toggled_motor_id()
{
    static int button_pressed_ACT_SELECT_MOTOR = 0;

    if (button_value_ACT_SELECT_MOTOR == 1 && button_pressed_ACT_SELECT_MOTOR == 0)
    {
        button_pressed_ACT_SELECT_MOTOR = 1;
        if (motor_select_ > 0)
        {
            log("Trying to enable and set the mode");
            if (0 != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "%s Error setting actuator mode", log_header_.c_str());
            }
            if (0 != 0)
            {
                RCLCPP_ERROR(this->get_logger(), "%s Error enabling actuator", log_header_.c_str());
            }
            if (0 == 0 && 0 == 0)
            {
                motor_select_ = toggle_motor_id;
                log("Motor enabled and set to mode");
                log("Motor_select__id to:");
                log_num(motor_select_);
            }
            else
            {
                log("Error in enabling and setting the motor ");
            }
        }
    }
    
    else if (button_value_ACT_SELECT_MOTOR == 0 && button_pressed_ACT_SELECT_MOTOR == 1)
    {
        button_pressed_ACT_SELECT_MOTOR = 0;
    }
}

void IndArmController::increase_motor_speed()
{
    static int button_pressed_ACT_INCREASE_SPEED = 0;

    if (button_value_ACT_INCREASE_SPEED == 1 && button_pressed_ACT_INCREASE_SPEED == 0)
    {
        button_pressed_ACT_INCREASE_SPEED = 1;
        motor_select_ = toggle_motor_id;

        if (step_size_ < max_speed)
        {
            step_size_++;
        }
        log("ACT_INCREASE_SPEED to: ");
        log_num(step_size_);
    }
    else if (button_value_ACT_INCREASE_SPEED == 0 && button_pressed_ACT_INCREASE_SPEED == 1)
    {
        button_pressed_ACT_INCREASE_SPEED = 0;
    }
}

void IndArmController::decrese_motor_speed()
{
    static int button_pressed_ACT_DECREASE_SPEED = 0;

    if (button_value_ACT_DECREASE_SPEED == 1 && button_pressed_ACT_DECREASE_SPEED == 0)
    {
        button_pressed_ACT_DECREASE_SPEED = 1;
        motor_select_ = toggle_motor_id;
        if (step_size_ > 1)
        {
            step_size_--;
        }
        log("ACT_DECREASE_SPEED to: ");
        log_num(step_size_);
    }
    else if (button_value_ACT_DECREASE_SPEED == 0 && button_pressed_ACT_DECREASE_SPEED == 1)
    {
        button_pressed_ACT_DECREASE_SPEED = 0;
    }
}

void IndArmController::change_motor_direction()
{
    static int button_pressed_M_PLUS_MINUS = 0;
    if (button_value_M_PLUS_MINUS == 1 && button_pressed_M_PLUS_MINUS == 0)
    {
        button_pressed_M_PLUS_MINUS = 1;
        pos = pos * -1;
        target_joint_state_.header.stamp = this->now();
        for (int i = 1; i <= motor_num; i++)
        {
            motor_positions_[i - 1] = pos;
            target_joint_state_.position[i - 1] = pos;
        }
        joint_state_pub_->publish(target_joint_state_);
        log("Joint_state_ Updated");
    }
    else if (button_value_M_PLUS_MINUS == 0 && button_pressed_M_PLUS_MINUS == 1)
    {
        button_pressed_M_PLUS_MINUS = 0;
    }
}

void IndArmController::joint_state_pub_timer_callback()
{

    toggle_motor_id_for_rotation();
    selected_the_toggled_motor_id();
    increase_motor_speed();
    decrese_motor_speed();
    change_motor_direction();
    // Log motor positions for debugging
    for (size_t i = 0; i < motor_positions_.size(); ++i)
    {
        RCLCPP_INFO(this->get_logger(), "Motor %ld Position: %f", i + 1, motor_positions_[i]);
    }

    // Publish joint commands
    std_msgs::msg::Float64MultiArray joint_command_msg;
    joint_command_msg.data = motor_positions_;
    joint_command_pub_->publish(joint_command_msg);
}

void IndArmController::joy_callback_(sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
 button_value_ACT_TOGGLE_MOTOR = joy->buttons[JoyStick::ACT_SELECT];

        button_value_ACT_SELECT_MOTOR = joy->buttons[JoyStick::ACT_SELECT_MOTOR];

        button_value_ACT_INCREASE_SPEED = joy->buttons[ACT_INCREASE_SPEED];

        button_value_ACT_DECREASE_SPEED = joy->buttons[ACT_DECREASE_SPEED];

        button_value_M_PLUS_MINUS = joy->axes[JoyStick::M_PLUS_MINUS];

 
}

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(IndArmController)


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IndArmController>(rclcpp::NodeOptions());
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception thrown: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
