/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>
#include <ros/console.h>

#include <tf/transform_listener.h>

#include <interactive_markers/interactive_marker_client.h>

using namespace interactive_markers;

class ImTunnel
{
public:
  tf::TransformListener tf_;
  interactive_markers::InteractiveMarkerClient client_;
  ros::Publisher pub_;
  ros::NodeHandle node_handle_;
  ros::Timer timer_;
  unsigned subscribers_;
  std::string topic_ns_;

  ImTunnel( std::string target_frame, std::string topic_ns ) :
    client_(tf_, target_frame, topic_ns ),
    subscribers_(0),
    topic_ns_(topic_ns)
  {
    client_.setInitCb( boost::bind(&ImTunnel::initCb, this, _1 ) );
    client_.setUpdateCb( boost::bind(&ImTunnel::updateCb, this, _1 ) );
    client_.setResetCb( boost::bind(&ImTunnel::resetCb, this, _1 ) );
    client_.setStatusCb( boost::bind(&ImTunnel::statusCb, this, _1, _2, _3 ) );

    pub_ = node_handle_.advertise<visualization_msgs::InteractiveMarkerUpdate>(
        topic_ns+"/tunneled", 1000,
        boost::bind(&ImTunnel::connectCallback, this, _1) );

    timer_ = node_handle_.createTimer(ros::Duration(0.05), boost::bind(&ImTunnel::timerCb, this, _1 ) );
  }

  typedef visualization_msgs::InteractiveMarkerInitConstPtr InitConstPtr;
  typedef visualization_msgs::InteractiveMarkerUpdateConstPtr UpdateConstPtr;

  void connectCallback(const ros::SingleSubscriberPublisher& pub)
  {
    ROS_INFO_STREAM( "New subsriber: " << pub.getSubscriberName() );
    client_.shutdown();
    client_.subscribe( topic_ns_ );
    usleep(100000);
  }

  void timerCb( const ros::TimerEvent& )
  {
    client_.update();
  }

  void updateCb( const UpdateConstPtr& msg )
  {
    ROS_INFO("updateCb");
    pub_.publish(msg);
  }

  void initCb( const InitConstPtr& msg )
  {
    ROS_INFO("initCb");
    visualization_msgs::InteractiveMarkerUpdate update;
    update.markers = msg->markers;
    update.seq_num = msg->seq_num;
    update.server_id = msg->server_id;
    pub_.publish(update);
  }

  void statusCb( InteractiveMarkerClient::StatusT status,
      const std::string& server_id,
      const std::string& msg )
  {
    std::string status_string[]={"INFO","WARN","ERROR"};
    ROS_INFO_STREAM( "(" << status_string[(unsigned)status] << ") " << server_id << ": " << msg );
  }

  void resetCb( const std::string& server_id )
  {
  }
};

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  ros::init(argc, argv, "im_tunnel");
  {
    ImTunnel im_tunnel( "/base_link", "basic_controls" );
    ros::spin();
  }
}
