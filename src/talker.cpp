/**Copyright (c) 2019, Charan Karthikeyan Parthasarathy Vasanthi
 *All rights reserved.

 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are met:
 *
 *1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 *2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.

 *3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.

 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * @file talker.cpp
 * @author Charan Karthikeyan Parthasarathy Vasanthi
 * @copyright BSD-3-Clause
 * @brief Publisher for simple publishing and subcribing from ros tutorial page
*/
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/custome_message_service.h"
/**
 * Message that is to be changed to
*/
extern std::string defaultMessage = "This ouput has been changed by Charan";

/**
 * @brief Fuction to changing the text using the custome_message_service service
 * @param request-The request sent to the service
 * @param response-The response by te service to the client
 * @return bool
*/
bool customeMsg(beginner_tutorials::custome_message_service::Request &requ,
                beginner_tutorials::custome_message_service::Response &resp) {
  defaultMessage = requ.inp_message;
  ROS_WARN_STREAM("Changed to the string set by Charan to");
  resp.out_message = requ.inp_message;
  return true;
}


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 * @brief The main function of the talker
 * @param argc- The value of argc
 * @param argv- The value of argv
 * @return None
 */
int main(int argc, char **argv) {
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
  static tf::TransformBroadcaster tf_br;
  tf::Transform trans;
// %EndTag(INIT)%
  int freq = 20;
  if (argc > 1) {
    freq = atoi(argv[1]);
  }
  if (freq > 0) {
    ROS_DEBUG_STREAM("The frequency of the loop is \t:" << freq);
  } else if (freq < 0) {
    ROS_ERROR_STREAM("The frequency cannot be negative");
    ROS_WARN_STREAM("Setting to default frequency to 20Hz");
    freq = 20;
  } else if (freq == 0) {
    ROS_FATAL_STREAM("Frequency is 0. Input frequency cannot be 0");
    ROS_WARN_STREAM("Setting to default frequency to 20Hz");
  }
  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */

  ros::NodeHandle n;


  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::ServiceServer server = n.advertiseService("customeMsg", customeMsg);

// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(freq);
// %EndTag(LOOP_RATE)%

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  int count = 0;
  while (ros::ok()) {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
// %Tag(FILL_MESSAGE)%
    std_msgs::String msg;

    std::stringstream ss;
    ss << defaultMessage << count;
    msg.data = ss.str();
// %EndTag(FILL_MESSAGE)%

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", msg.data.c_str());
// %EndTag(ROSCONSOLE)%

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(msg);
// %EndTag(PUBLISH)%
    trans.setOrigin(tf::Vector3(cos(ros::Time::now().toSec()),
                    sin(ros::Time::now().toSec()),0.0));
    tf::Quaternion q;
    q.setRPY(0,0,1);
    trans.setRotation(q);
    tf_br.sendTransform(tf::StampedTransform(trans,ros::Time::now(),"hello", 
                       "bye"));
// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
    ++count;
  }


  return 0;
}
