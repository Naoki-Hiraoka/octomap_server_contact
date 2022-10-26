// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <octomap_server_contact/octomap_server_contact.h>
#include <algorithm>

namespace octomap_server_contact
{
  OctomapServerContact::OctomapServerContact(const ros::NodeHandle &privateNh, const ros::NodeHandle &nh)
    : OctomapServer(privateNh, nh)
  {
    delete this->m_octree;
    this->m_octree = nullptr;
    this->m_octree = new OcTreeContact(this->m_res);
    this->octreeContact_ = dynamic_cast<OcTreeContact*>(this->m_octree);

    ros::NodeHandle pnh = privateNh;
    this->contactSensorSub_ = pnh.subscribe("contact_sensors_in", 1, &OctomapServerContact::insertContactSensorCallback, this);

    // cloud_inと同じ点を与えると、occupied cellを消してしまうので、cloud_inに含まれない、消したい部分の点群のみを与えること
    m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (pnh, "freespace_cloud_in", 5);
    m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
    m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServerContact::insertFreeCloudCallback, this, boost::placeholders::_1));
  }

  void OctomapServerContact::insertContactSensorCallback(const jsk_recognition_msgs::ContactSensorArray::ConstPtr& msg) {
    for(int i=0; i<msg->datas.size(); i++){
      if(!msg->datas[i].contact) continue;

      tf::StampedTransform sensorToWorldTf;
      try {
	this->m_tfListener.waitForTransform(this->m_worldFrameId,
					    msg->datas[i].header.frame_id,
					    msg->datas[i].header.stamp,
					    ros::Duration(3.0));
        this->m_tfListener.lookupTransform(this->m_worldFrameId,
                                           msg->datas[i].header.frame_id,
                                           msg->datas[i].header.stamp,
                                           sensorToWorldTf);
      } catch(tf::TransformException& ex){
        ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
      }

      octomap::point3d point(sensorToWorldTf.getOrigin().x(),
                             sensorToWorldTf.getOrigin().y(),
                             sensorToWorldTf.getOrigin().z());

      octomap::OcTreeKey key;
      if (this->m_octree->coordToKeyChecked(point, key)){

        updateMinKey(key, this->m_updateBBXMin);
        updateMaxKey(key, this->m_updateBBXMax);
        this->octreeContact_->updateNode(key, this->octreeContact_->getProbHitContactSensorLog());
      }

    }
    this->publishAll(msg->header.stamp);
  }

  void OctomapServerContact::insertFreeCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
    ros::WallTime startTime = ros::WallTime::now();

    PCLPointCloud pc; // input cloud for filtering and ground-detection
    pcl::fromROSMsg(*cloud, pc);

    tf::StampedTransform sensorToWorldTf;
    try {
      m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    // set up filter for height range, also removes NANs:
    pcl::PassThrough<PCLPoint> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);
    pcl::PassThrough<PCLPoint> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);
    pcl::PassThrough<PCLPoint> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    // directly transform to map frame:
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    // just filter height range:
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    PCLPointCloud pc_ground = pc;
    pc_ground.header = pc.header;

    PCLPointCloud pc_nonground; // empty
    pc_nonground.header = pc.header;

    insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG("FreeSpace Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc.size(), pc_nonground.size(), total_elapsed);

    publishAll(cloud->header.stamp);
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_server_contact");

  const ros::NodeHandle nh;
  const ros::NodeHandle private_nh("~");

  octomap_server_contact::OctomapServerContact server(private_nh, nh);
  ros::spin();

}
