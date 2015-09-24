#include <opencv2/opencv.hpp>
#include <iterator>

class BetterBlobDetector : public cv::SimpleBlobDetector
{
public:
    
    BetterBlobDetector(const cv::SimpleBlobDetector::Params &parameters = cv::SimpleBlobDetector::Params());
    
    const std::vector < std::vector<cv::Point> > getContours();
    void getBlobData(size_t idx, double &area, double &circularity, double &inertia, double &perimeter, double &convexity);
protected:
    virtual void detectImpl( const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, const cv::Mat& mask=cv::Mat()) const;
    virtual void findBlobs(const cv::Mat &image, const cv::Mat &binaryImage,
                           std::vector<Center> &centers, std::vector < std::vector<cv::Point> >&contours) const;
private:
    cv::SimpleBlobDetector::Params params;
    
};
