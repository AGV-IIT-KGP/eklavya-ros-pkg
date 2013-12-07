#ifndef __FeatureExtractor_H_
#define __FeatureExtractor_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "Feature.h"

namespace Perception {
namespace RoadDetection {

	class FeatureExtractor {
		
	public:
		virtual Feature extractFeature(const sensor_msgs::ImageConstPtr& inputImage);
		
	};
	
}
}

#endif
