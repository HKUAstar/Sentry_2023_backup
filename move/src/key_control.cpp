#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>
#include <termios.h>
#include <unistd.h>
using namespace std;
    
class keyboard{
    public:
    keyboard();
    ~keyboard();
    int kbhit();
    int getch();
    private:
    struct termios initial_settings, new_settings;
    int peek_character;
};

int main(int argc, char *argv[]){
  int key_nr;
  char Input = ' ';
  string input = "";
  keyboard keyb;
  setlocale(LC_ALL,"");
  ros::init(argc,argv,"ks");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::String>("keyb_input",10);

  std_msgs::String msg;
  int count = 0;
  ros::Rate r(50);

  msg.data = input + Input;
  pub.publish(msg);
  ROS_INFO("发送的消息:%s",msg.data.c_str());
  while(Input!='0' && ros::ok()){
    key_nr = keyb.getch();
    Input = key_nr;
    msg.data = input + Input;
    pub.publish(msg);
    ROS_INFO("发送的消息:%s",msg.data.c_str());
  }
  //while(ros::ok() && Input!='0'){
  //  if(keyb.kbhit()){
  //    key_nr = keyb.getch();
  //    Input = key_nr;
  //  }
  //  msg.data = Input;
  //  pub.publish(msg);
  //  ROS_INFO("发送的消息:%c",msg.data);
  //  r.sleep();
  //  ros::spinOnce();
  //}
  return 0;
}
    
keyboard::keyboard(){
  tcgetattr(0,&initial_settings);
  new_settings = initial_settings;
  new_settings.c_lflag &= ~ICANON;
  new_settings.c_lflag &= ~ECHO;
  new_settings.c_lflag &= ~ISIG;
  new_settings.c_cc[VMIN] = 1;
  new_settings.c_cc[VTIME] = 0;
  tcsetattr(0, TCSANOW, &new_settings);
  peek_character=-1;
}
        
keyboard::~keyboard(){
  tcsetattr(0, TCSANOW, &initial_settings);
}
            
int keyboard::kbhit(){
  unsigned char ch;
  int nread;
  if (peek_character != -1) return 1;
  new_settings.c_cc[VMIN]=0;
  tcsetattr(0, TCSANOW, &new_settings);
  nread = read(0,&ch,1);
  new_settings.c_cc[VMIN]=1;
  tcsetattr(0, TCSANOW, &new_settings);

  if (nread == 1){
    peek_character = ch;
    return 1;
  }
  return 0;
}
                
int keyboard::getch(){
  char ch;

  if (peek_character != -1){
  ch = peek_character;
  peek_character = -1;
  }
  else read(0,&ch,1);
    return ch;
}
