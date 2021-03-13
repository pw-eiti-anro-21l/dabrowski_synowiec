#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/action/rotate_absolute.hpp>

#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif

#define KEYCODE_W 0x57
#define KEYCODE_A 0x41
#define KEYCODE_D 0x44
#define KEYCODE_S 0x53

class KeyboardReader
{
public:

  void readOne(char * c)
  {
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if(events > 0)
      {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x57)
        {
          *c = KEYCODE_W;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x41)
        {
          *c = KEYCODE_A;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x44)
        {
          *c = KEYCODE_D;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x53)
        {
          *c = KEYCODE_S;
          return;
        }
          return;
        }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

class TeleopTurtle
{
public:
  TeleopTurtle();
  int keyLoop();

private:
  void spin();
  void sendGoal(float theta);
  void goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future);
  void cancelGoal();
  
  rclcpp::Node::SharedPtr nh_;
  double linear_, angular_, l_scale_, a_scale_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SharedPtr rotate_absolute_client_;
  rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr goal_handle_;
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_ = rclcpp::Node::make_shared("teleop_turtle");
  nh_->declare_parameter("scale_angular", rclcpp::ParameterValue(2.0));
  nh_->declare_parameter("scale_linear", rclcpp::ParameterValue(2.0));
  nh_->get_parameter("scale_angular", a_scale_);
  nh_->get_parameter("scale_linear", l_scale_);

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 1);
  rotate_absolute_client_ = rclcpp_action::create_client<turtlesim::action::RotateAbsolute>(nh_, "turtle1/rotate_absolute");
}

void TeleopTurtle::sendGoal(float theta)
{
  auto goal = turtlesim::action::RotateAbsolute::Goal();
  goal.theta = theta;
  auto send_goal_options = rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future)
    {
      RCLCPP_DEBUG(nh_->get_logger(), "Goal response received");
      this->goal_handle_ = future.get();
    };
  rotate_absolute_client_->async_send_goal(goal, send_goal_options);
}

void TeleopTurtle::goalResponseCallback(std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future)
{
  RCLCPP_DEBUG(nh_->get_logger(), "Goal response received");
  this->goal_handle_ = future.get();
}

void TeleopTurtle::cancelGoal()
{
 if (goal_handle_)
 {
   RCLCPP_DEBUG(nh_->get_logger(), "Sending cancel request");
   try
   {
     rotate_absolute_client_->async_cancel_goal(goal_handle_);
   }
   catch (...)
   {
     // This can happen if the goal has already terminated and expired
   }
 }
}

KeyboardReader input;

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  rclcpp::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  int rc = teleop_turtle.keyLoop();
  input.shutdown();
  rclcpp::shutdown();
  
  return rc;
}

void TeleopTurtle::spin()
{
  while (rclcpp::ok())
  {
    rclcpp::spin_some(nh_);
  }
}

int TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;

  std::thread{std::bind(&TeleopTurtle::spin, this)}.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");
  puts("Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations. 'F' to cancel a rotation.");
  puts("'Q' to quit.");


  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return -1;
    }

    linear_=angular_=0;
    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_G:
        RCLCPP_DEBUG(nh_->get_logger(), "G");
        sendGoal(0.0f);
        break;
      case KEYCODE_T:
        RCLCPP_DEBUG(nh_->get_logger(), "T");
        sendGoal(0.7854f);
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(nh_->get_logger(), "R");
        sendGoal(1.5708f);
        break;
      case KEYCODE_E:
        RCLCPP_DEBUG(nh_->get_logger(), "E");
        sendGoal(2.3562f);
        break;
      case KEYCODE_D:
        RCLCPP_DEBUG(nh_->get_logger(), "D");
        sendGoal(3.1416f);
        break;
      case KEYCODE_C:
        RCLCPP_DEBUG(nh_->get_logger(), "C");
        sendGoal(-2.3562f);
        break;
      case KEYCODE_V:
        RCLCPP_DEBUG(nh_->get_logger(), "V");
        sendGoal(-1.5708f);
        break;
      case KEYCODE_B:
        RCLCPP_DEBUG(nh_->get_logger(), "B");
        sendGoal(-0.7854f);
        break;
      case KEYCODE_F:
        RCLCPP_DEBUG(nh_->get_logger(), "F");
        cancelGoal();
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        return 0;
    }
   

    geometry_msgs::msg::Twist twist;
    twist.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_->publish(twist);    
      dirty=false;
    }
  }


  return 0;
}



