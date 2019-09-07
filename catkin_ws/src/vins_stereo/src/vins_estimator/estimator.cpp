#include "estimator.h"

Estimator::Estimator(Parameters& config_para):config(config_para)
{

    feature_manager = new Feature(config.feature_point_max_number,config.feature_min_dist,config.pub_track_image);
}
