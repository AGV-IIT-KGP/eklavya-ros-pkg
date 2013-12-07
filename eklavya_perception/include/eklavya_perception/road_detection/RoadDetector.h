#ifndef __RoadDetector_H_
#define __RoadDetector_H_

#include "FeatureExtractor.h"
#include "Classifier.h"
#include <list>

using namespace std;

namespace Perception {
namespace RoadDetection {

	class RoadDetector {
	
	private:
		list<FeatureExtractor> featureExtractors;
		Classifier classifier;
		
	public:
		RoadDetector(Classifier &classifier) {
			this->classifier = classifier;
		}
		
		RoadDetector(list<FeatureExtractor> &featureExtractors, Classifier &classifier) {
			this->featureExtractors = featureExtractors;
			this->classifier = classifier;
		}
		
		list<FeatureExtractor> getFeatureExtractors() {
			return featureExtractors;
		}
		
	};

}
}

#endif
