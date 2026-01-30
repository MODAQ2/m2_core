/**
 * @file m2_supervisor.cpp
 * @author Casey Nichols (casey.nichols@nlr.gov)
 * @brief ros node supervising the MODAQ system
 * @version 0.1
 * @date 2024-10-02
 * 
 * @copyright Copyright (c) 2024
 * 
 *
*/
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "modaq_messages/msg/systemmsg.hpp"
#include "EmailSender.h"
#include "Logger.h"
#include "Snoozer.h"
#include <memory>
#include "BagAnalyzer.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

class M2Supervisor : public rclcpp::Node
{
public:
  M2Supervisor()
      : Node("M2Supervisor"), emlr(), logr(), snzr(), bagAnalyzer()
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("loggerPath", "");
    this->declare_parameter<int>("loggerLimitBytes", 0);
    this->declare_parameter<std::string>("emailSendAddr", "");
    this->declare_parameter<std::string>("emailSendPwd", "");
    this->declare_parameter<std::vector<std::string>>("emailGroup1", {});
    this->declare_parameter<std::vector<std::string>>("emailGroup2", {});

    std::string loggerPath = this->get_parameter("loggerPath").as_string();
    int loggerLimitBytes = this->get_parameter("loggerLimitBytes").as_int();
    std::string emailSendAddr = this->get_parameter("emailSendAddr").as_string();
    std::string emailSendPwd = this->get_parameter("emailSendPwd").as_string();
    grp1 = this->get_parameter("emailGroup1").as_string_array();
    grp2 = this->get_parameter("emailGroup2").as_string_array();

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "BagRecorder");

    if (!parameters_client->wait_for_service(2s)) {
      RCLCPP_ERROR(this->get_logger(), "BagRecorder service not available.");
      return;
    }

    std::vector<rclcpp::Parameter> parameters;
    try {
      parameters = parameters_client->get_parameters({"loggedTopics", "dataFolder"});
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Error getting parameters: %s", e.what());
      return;
    }

    loggedTopics = parameters[0].as_string_array();
    folderPath = parameters[1].as_string();

    RCLCPP_INFO(this->get_logger(), "loggedTopic[0]: %s", loggedTopics[0].c_str());
    RCLCPP_INFO(this->get_logger(), "emailGroup2: %s", grp2[0].c_str());

    logr.setBasePathandSize(loggerPath, loggerLimitBytes);
    emlr.setCredentials("smtp.gmail.com:587", emailSendAddr, emailSendPwd);

    grp1and2 = grp1;
    grp1and2.insert(grp1and2.end(), grp2.begin(), grp2.end());

    sysmsg_sub = this->create_subscription<modaq_messages::msg::Systemmsg>("/system_messenger", 10, std::bind(&M2Supervisor::messenger_callback, this, _1));
    //timer_bag_analyze = this->create_wall_timer(10000ms, std::bind(&M2Supervisor::bag_analyzer_callback, this));

    publisher_sysmsg = this->create_publisher<modaq_messages::msg::Systemmsg>("/system_messenger", 10);
    modaq_messages::msg::Systemmsg initmsg;
    initmsg.email_option = 0;
    initmsg.header.stamp = now();
    initmsg.message_tag = "M2Supervisor Started";
    initmsg.log_enable = true;
    publisher_sysmsg->publish(initmsg);
  }

private:
  rclcpp::Subscription<modaq_messages::msg::Systemmsg>::SharedPtr sysmsg_sub;
  rclcpp::Publisher<modaq_messages::msg::Systemmsg>::SharedPtr publisher_sysmsg;
  rclcpp::TimerBase::SharedPtr timer_bag_analyze;

  EmailSender emlr;
  Logger logr;
  Snoozer snzr;
  BagAnalyzer bagAnalyzer;
  std::ostringstream loggerString;
  std::vector<std::string> grp1and2;
  std::vector<std::string> grp1;
  std::vector<std::string> grp2;
  std::vector<std::string> loggedTopics;
  std::chrono::time_point<std::chrono::high_resolution_clock> start;
  std::string folderPath;

  void messenger_callback(modaq_messages::msg::Systemmsg msngr)
  {
    std::cout << "Received Message with Tag: " << msngr.message_tag << " and timestamp: ";
    std::cout << std::fixed << std::setprecision(3) << (double)msngr.header.stamp.sec + (double)msngr.header.stamp.nanosec / 1e9 << std::endl;

    if (msngr.snooze_enable)
    {
      RCLCPP_INFO(this->get_logger(), "Snooze Enabled");

      if (checkSnooze(msngr))
      {
        RCLCPP_INFO(this->get_logger(), "Message Snoozed");
        return;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Proceeding with processing after snooze cleared");

    if (msngr.email_option != 0)
    {
      std::cout << "Attempting Email" << std::endl;
      std::cout << "email success: " << sendEmail(msngr, &emlr) << std::endl;
    }

    if (msngr.log_enable != 0)
    {
      writeLogFile(msngr);
    }
  }

  void writeLogFile(modaq_messages::msg::Systemmsg msgnr)
  {
    loggerString.str("");
    loggerString.clear();
    std::time_t now = std::time(nullptr);
    std::tm *localTime = std::localtime(&now);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", localTime);
    loggerString << std::string(buffer) << " , "
                 << msgnr.header.stamp.sec << " , "
                 << msgnr.message_tag << " , "
                 << msgnr.message_body;

    logr.log(loggerString.str());
  }

  bool sendEmail(modaq_messages::msg::Systemmsg msgEml, EmailSender *sendr)
  {
    if (msgEml.email_option == 1)
    {
      return sendr->sendEmail(grp1, "M2 Email: " + msgEml.message_tag, msgEml.message_body);
    }
    if (msgEml.email_option == 2)
    {
      return sendr->sendEmail(grp1and2, "M2 Email: " + msgEml.message_tag, msgEml.message_body);
    }
    else
    {
      return false;
    }
  }

  bool checkSnooze(modaq_messages::msg::Systemmsg msgSnz)
  {
    return snzr.snoozeInput(msgSnz.message_tag, msgSnz.snooze_time);
  }

  void bag_analyzer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Processing checking bag");
    startTime();
    auto newestMcapFile = bagAnalyzer.findNewestMcapFile(folderPath);
    if (!newestMcapFile)
    {
      std::cout << "No .mcap files found." << std::endl;
      return;
    }
    std::cout << "Folder Search: ";
    stopTime();
    for (const auto &topicNameToFind : loggedTopics)
    {
      startTime();

      try
      {
        auto [topicName, messageCount] = bagAnalyzer.analyzeBag(newestMcapFile->string(), topicNameToFind);
        std::cout << "Found topic: " << topicName << ", message count: " << messageCount << std::endl;
      }
      catch (const std::exception &e)
      {
        std::cerr << "Error analyzing topic: " << topicNameToFind << ", error: " << e.what() << std::endl;
      }
      std::cout << "Topic Search: " << topicNameToFind << "  ";
      stopTime();
    }
  }

  void startTime()
  {
    start = std::chrono::high_resolution_clock::now();
  }

  void stopTime()
  {
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time elapsed: " << duration.count() << " milliseconds" << std::endl;
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<M2Supervisor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
