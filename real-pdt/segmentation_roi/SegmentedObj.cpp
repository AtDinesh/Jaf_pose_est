#include "SegmentedObj.h"
#include "camera/Camera.h"
#include "QImage"
#include "AncillaryMethods.h"


SegmentedObj::SegmentedObj()
{
}

bool SegmentedObj::get2DBoundingBoxInImagePlane(Vector<double>& bbox, const Camera& camera)
{
    Vector<double> pos2D_tmp(3);   // temporary variable

    camera.WorldToImage(pos3DMinX, Globals::WORLD_SCALE, pos2D_tmp);
    double minX = pos2D_tmp(0);

    camera.WorldToImage(pos3DMaxX, Globals::WORLD_SCALE, pos2D_tmp);
    double maxX = pos2D_tmp(0);

    camera.WorldToImage(pos3DMinY, Globals::WORLD_SCALE, pos2D_tmp);
    double minY = pos2D_tmp(1);

    camera.ProjectToGP(pos3DMaxY, Globals::WORLD_SCALE, pos3DMaxY);

    camera.WorldToImage(pos3DMaxY, Globals::WORLD_SCALE, pos2D_tmp);

    // Left
    bbox(0) = minX;
    // Top
    bbox(1) = minY;
    // Width
    bbox(2) = maxX-minX;
    // Height
    bbox(3) = pos2D_tmp(1)-minY;

    double incHeightValue = bbox(3)*Globals::inc_height_ratio;
    bbox(3) += incHeightValue;
    bbox(1) -= incHeightValue/2.0;

    double incWidthValue = bbox(2)*Globals::inc_width_ratio;
    bbox(2) += incWidthValue;
    bbox(0) -= incWidthValue/2.0;

    // Check Left
    if(bbox(0) >= Globals::dImWidth)
        return false;
    else if(bbox(0) < 0)
        bbox(0) = 0;

    // Check Top
    if(bbox(1) >= Globals::dImHeight)
        return false;
    else if(bbox(1) < 0)
        bbox(1) = 0;

    // Check Width
    if(bbox(2) <= 0)
        return false;
    else if(bbox(0)+bbox(2) >= Globals::dImWidth) // Check right
        bbox(2) = Globals::dImWidth - bbox(0) - 1;

    // Check Height
    if(bbox(3) <= 0)
        return false;
    else if(bbox(3) >= Globals::dImHeight)
        bbox(3) = Globals::dImHeight - 1;

    return true;
}
