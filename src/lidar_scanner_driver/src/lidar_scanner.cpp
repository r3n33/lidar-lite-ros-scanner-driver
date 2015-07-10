/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Renee Love
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Case Western Reserve University nor the names of its
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

#include <lidar_scanner_driver/lidar_scanner.h>
#include <ros/console.h>

/* TODO: Things I'd like to see completed here:
 *
 * TODO: Send laser scans with less than 360 points.
 * TODO: Get V2 LIDAR-Lite sensor for more fun!
 */

namespace lidar_scanner_driver 
{
	LidarScanner::LidarScanner(const std::string& port, uint32_t baud_rate, boost::asio::io_service& io)
							  : port_(port), baud_rate_(baud_rate), shutting_down_(false), serial_(io, port_) 
	{
    	serial_.set_option(boost::asio::serial_port_base::baud_rate(baud_rate_));
	}

	void LidarScanner::poll(sensor_msgs::LaserScan::Ptr scan) 
	{
		uint8_t num_points_ = 0;
	    bool scan_ready = false;

		rpms=0;

		scan->angle_min = 0.0;
		scan->angle_max = 2.0*M_PI;
		scan->angle_increment = (2.0*M_PI/360.0);
		scan->range_min = 0.01;
		scan->range_max = 6.0;
		scan->ranges.resize(360);
		scan->intensities.resize(360);

		int scan_position;
		int distance;
		double degreesPerSecond;
		int scan_time_ms;
		char dummy;

		while (!shutting_down_ && !scan_ready) 
		{
			boost::asio::streambuf response;

			boost::asio::read_until(serial_, response, "\r\n" );

		    std::istream response_stream(&response);
		    response_stream >> scan_position;
		    response_stream >> dummy;
		    response_stream >> distance;
		    response_stream >> dummy;
		    response_stream >> degreesPerSecond;
		    response_stream >> dummy;
		    response_stream >> scan_time_ms;

		    //ROS_INFO( "Read Point: %d, %d, %f, %d" , scan_position, distance, degreesPerSecond, scan_time_ms );

		    if ( scan_time_ms > 25 )
		    {
		    	ROS_WARN( "LIDAR-Lite sampling took %d milliseconds", scan_time_ms );
		    }

		    rpms = degreesPerSecond * 60.0 / 360.0;

			scan->ranges[scan_position] = distance / 100.0; //centimeter to meter conversion
			scan->intensities[scan_position] = distance;
			scan->time_increment = degreesPerSecond; //seconds between scan poins

			if ( ++num_points_ >= 5)
			{
				scan_ready = true;
			}
		} 
	}
};
