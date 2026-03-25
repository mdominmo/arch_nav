#ifndef ARCH_NAV_UTILS_GNSS_LOCAL_FRAME_HPP_
#define ARCH_NAV_UTILS_GNSS_LOCAL_FRAME_HPP_

namespace arch_nav::utils::gnss_local_frame {

double lat_to_ned_x(double lat, double ref_lat);
double lon_to_ned_y(double lon, double ref_lon, double ref_lat);
double alt_to_ned_z(double alt, double ref_alt);

double ned_x_to_lat(double x, double ref_lat);
double ned_y_to_lon(double y, double ref_lon, double ref_lat);
double ned_z_to_alt(double z, double ref_alt);

}  // namespace arch_nav::utils::gnss_local_frame

#endif  // ARCH_NAV_UTILS_GNSS_LOCAL_FRAME_HPP_
