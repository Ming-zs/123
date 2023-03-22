#ifndef FIND_ARMOR_BOX__
#define FIND_ARMOR_BOX__

#include "Visual/ArmorFinder/inc/FindLightBlob.h"
#include <memory>

class FindArmorBox {
  private:
    FindLightBlob find_light_blob;

    bool match_armor_a_boxes(const cv::Mat &src, const LightBlobs &light_blobs,
                             ArmorBoxes &armor_boxes);

  public:

    bool find_armor_box(cv::Mat &src, ArmorBox &box);

    FindArmorBox();
};

using ArmorFinder = FindArmorBox;

#endif