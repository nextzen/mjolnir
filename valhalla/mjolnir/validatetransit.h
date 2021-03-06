#ifndef VALHALLA_MJOLNIR_VALIDATETRANSIT_H
#define VALHALLA_MJOLNIR_VALIDATETRANSIT_H

#include <boost/property_tree/ptree.hpp>
#include <unordered_set>

#include <valhalla/baldr/graphid.h>

namespace valhalla {
namespace mjolnir {

struct OneStopTest {
  std::string origin;
  std::string destination;
  std::string route_id;
  std::string date_time;

  bool operator < (const OneStopTest& other) const {
    return origin < other.origin;
  }
};
/**
 * Class used to test graph tile information at the transit level.
 */
class ValidateTransit {
 public:

  /**
   * Validate the transit level graph tile information.
   * @param pt            property tree containing the hierarchy configuration
   * @param all_tiles     unordered set of all the transit tiles.
   * @param onestoptests  list of origin and destinations to test
   */
  static void Validate(const boost::property_tree::ptree& pt,
                       const std::unordered_set<baldr::GraphId>& all_tiles,
                       const std::vector<OneStopTest>& onestoptests);

};

}
}

#endif  // VALHALLA_MJOLNIR_VALIDATETRANSIT_H
