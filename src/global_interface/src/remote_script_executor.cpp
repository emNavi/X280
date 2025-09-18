#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib>

#include <sys/wait.h>
#include <spawn.h>
#include <signal.h>
#include <thread>
#include <unistd.h>
#include <sys/stat.h>

extern char **environ;

bool isFile(const std::string &path) {
    struct stat sb;
    return (stat(path.c_str(), &sb) == 0 && S_ISREG(sb.st_mode));
}

// 异步执行，但不产生僵尸进程。接收的每条命令会创建一个新进程，适用于频率不高的接收
void runScriptAsync(const std::string &input) {
    pid_t pid;
    // char* argv[] = { (char*)"bash", (char*)cmd.c_str(), nullptr };
    char* argv[4] = {nullptr, nullptr, nullptr, nullptr};

    if(isFile(input)) {
        // 输入是文件
        argv[0] = (char*)"bash";
        argv[1] = (char*)input.c_str();
        argv[2] = nullptr;
    } else {
        // 输入是命令
        argv[0] = (char*)"bash";
        argv[1] = (char*)"-c";
        argv[2] = (char*)input.c_str();
        argv[3] = nullptr;
    }

    if(posix_spawn(&pid, "/bin/bash", nullptr, nullptr, argv, environ) == 0) {
        ROS_INFO("Spawned process PID: %d", pid);
        // 使用单独线程等待回收，避免阻塞 callback
        std::thread([pid](){
            int status;
            waitpid(pid, &status, 0);
        }).detach();
    } else {
        ROS_ERROR("Failed to spawn process");
    }
    ROS_INFO("Executing command or script: %s", input.c_str());
}

void callback(const std_msgs::String::ConstPtr& msg) {
    runScriptAsync(msg->data);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "script_executor");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/uav_run_script", 10, callback);
    ros::spin();
    return 0;
}