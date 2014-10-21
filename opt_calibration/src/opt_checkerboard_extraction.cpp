#include <calibration_common/algorithms/automatic_checkerboard_finder.h>
#include <calibration_common/algorithms/plane_extraction.h>
#include <calibration_common/base/pcl_conversion.h>
#include <calibration_common/ceres/plane_fit.h>
#include <calibration_common/objects/checkerboard.h>
#include <calibration_common/pinhole/pinhole.h>

#include <open_ptrack/opt_calibration/opt_checkerboard_extraction.h>

namespace open_ptrack
{
namespace opt_calibration
{

OPTCheckerboardExtraction::Status OPTCheckerboardExtraction::perform(boost::shared_ptr<cb::PinholeView<cb::Checkerboard> > & color_view,
                                                                     boost::shared_ptr<cb::Checkerboard> & extracted_checkerboard) const
{
  assert(checkerboard_ and color_sensor_ and not image_.empty());
  cb::AutomaticCheckerboardFinder cb_finder;
  cb::Cloud2 corners(cb::Size2(checkerboard_->rows(), checkerboard_->cols()));
  cb_finder.setImage(image_);
  if (not cb_finder.find(*checkerboard_, corners))
    return EXTRACTED_NONE;

  std::stringstream ss;
  ss << "view_" << color_sensor_->frameId().substr(1);

  color_view = boost::make_shared<cb::PinholeView<cb::Checkerboard> >();
  color_view->setId(ss.str());
  color_view->setObject(checkerboard_);
  color_view->setPoints(corners);
  color_view->setSensor(color_sensor_);
  color_view->setId(ss.str());

  extracted_checkerboard = boost::make_shared<cb::Checkerboard>(*color_view);

  return EXTRACTED_COLOR;
}

OPTCheckerboardExtraction::Status OPTCheckerboardExtraction::perform(boost::shared_ptr<cb::PinholeView<cb::Checkerboard> > & color_view,
                                                                     boost::shared_ptr<cb::DepthViewPCL<cb::PlanarObject> > & depth_view,
                                                                     boost::shared_ptr<cb::Checkerboard> & extracted_checkerboard,
                                                                     boost::shared_ptr<cb::PlanarObject> & extracted_plane) const
{
  if (perform(color_view, extracted_checkerboard) == EXTRACTED_NONE)
    return EXTRACTED_NONE;

  assert(depth_sensor_ and cloud_);

  cb::PointPlaneExtraction<pcl::PointXYZ>::Ptr plane_extractor;

  plane_extractor = boost::make_shared<cb::PointPlaneExtraction<pcl::PointXYZ> >();
  plane_extractor->setInputCloud(cloud_);
  plane_extractor->setRadius(std::min(checkerboard_->width(), checkerboard_->height()) / 1.5);

  cb::Point3 center = depth_transform_ * extracted_checkerboard->center();
  pcl::PointXYZ p;
  p.x = center[0];
  p.y = center[1];
  p.z = center[2];
  plane_extractor->setPoint(p);

  cb::PlaneInfo plane_info;

  if (not plane_extractor->extract(plane_info))
    return EXTRACTED_COLOR;

  std::stringstream ss;
  ss << "view_" << depth_sensor_->frameId().substr(1);

  depth_view = boost::make_shared<cb::DepthViewPCL<cb::PlanarObject> >();
  depth_view->setId(ss.str());
  depth_view->setData(cloud_);
  depth_view->setPoints(*plane_info.indices_);
  depth_view->setSensor(depth_sensor_);
  depth_view->setObject(checkerboard_);

  extracted_plane = boost::make_shared<cb::PlanarObject>();
  extracted_plane->setParent(depth_sensor_);
  extracted_plane->setFrameId(ss.str());
  cb::Cloud3 cloud_tmp(cb::PCLConversion<cb::Scalar>::toPointMatrix(*cloud_, depth_view->points()));
  extracted_plane->setPlane(cb::PlaneFit<cb::Scalar>::robustFit(cloud_tmp, plane_info.std_dev_));

  return EXTRACTED_COLOR_AND_DEPTH;
}

} /* namespace opt_calibration */
} /* namespace open_ptrack */
