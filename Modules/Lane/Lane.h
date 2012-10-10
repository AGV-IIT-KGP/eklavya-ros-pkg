namespace Lanespace
{
  class LaneDetection
  {
  public:
    static void markLane(IplImage *input_img, char** map, int choice, int scale);
    static IplImage* colorBasedLaneDetection(IplImage* frame_in,int maxvalue_B,int maxvalue_G,int maxvalue_R,int minvalue_B,int minvalue_G,int minvalue_R,int vote,int length,int merge);
    static IplImage* applyHoughTransform(IplImage* img, int vote, int length, int merge);
    static void initImage(IplImage *dst);
    static IplImage* joinResult(IplImage* color_gray,IplImage* hough_gray);
    static void initializeLaneVariables(IplImage *img);
    
  };
}
