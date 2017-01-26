#include <string>
#include <vector>

#include "mjolnir/validatetransit.h"
#include "mjolnir/graphbuilder.h"
#include "config.h"

using namespace valhalla::mjolnir;

#include <ostream>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/optional.hpp>

#include <valhalla/midgard/point2.h>
#include <valhalla/midgard/aabb2.h>
#include <valhalla/midgard/polyline2.h>
#include <valhalla/midgard/logging.h>

namespace bpo = boost::program_options;

boost::filesystem::path config_file_path;
std::vector<std::string> input_files;

bool ParseArguments(int argc, char *argv[]) {

  bpo::options_description options(
    "valhalla_validate_transit " VERSION "\n"
    "\n"
    " Usage: valhalla_validate_transit [options] <protocolbuffer_input_file>\n"
    "\n"
    "valhalla_validate_transit is a program that validates the transit graph and "
    "schedule at a particular time.  It will not use the route tiles at all.  It "
    "will only the transit tiles."
    "\n"
    "\n");

  options.add_options()
      ("help,h", "Print this help message.")
      ("version,v", "Print the version of this software.")
      ("config,c",
        boost::program_options::value<boost::filesystem::path>(&config_file_path)->required(),
        "Path to the json configuration file.")
      // positional arguments
      ("input_files", boost::program_options::value<std::vector<std::string> >(&input_files)->multitoken());

  bpo::positional_options_description pos_options;
  pos_options.add("input_files", 16);

  bpo::variables_map vm;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(options).positional(pos_options).run(), vm);
    bpo::notify(vm);

  } catch (std::exception &e) {
    std::cerr << "Unable to parse command line options because: " << e.what()
      << "\n" << "This is a bug, please report it at " PACKAGE_BUGREPORT
      << "\n";
    return false;
  }

  if (vm.count("help")) {
    std::cout << options << "\n";
    return true;
  }

  if (vm.count("version")) {
    std::cout << "valhalla_validate_transit " << VERSION << "\n";
    return true;
  }

  if (vm.count("config")) {
    if (boost::filesystem::is_regular_file(config_file_path))
      return true;
    else
      std::cerr << "Configuration file is required\n\n" << options << "\n\n";
  }

  return false;
}

int main(int argc, char** argv) {

  if (!ParseArguments(argc, argv))
    return EXIT_FAILURE;

  //check what type of input we are getting
  boost::property_tree::ptree pt;
  boost::property_tree::read_json(config_file_path.c_str(), pt);

  //configure logging
  boost::optional<boost::property_tree::ptree&> logging_subtree = pt.get_child_optional("mjolnir.logging");
  if(logging_subtree) {
    auto logging_config = valhalla::midgard::ToMap<const boost::property_tree::ptree&, std::unordered_map<std::string, std::string> >(logging_subtree.get());
    valhalla::midgard::logging::Configure(logging_config);
  }

  std::string testfile;
  std::vector<OneStopTest> onestoptests;

  testfile = std::string("acc.tests");
  onestoptests = ParseTestFile(testfile);
  std::sort(onestoptests.begin(), onestoptests.end());

  //ParseLogFile("out.log");

  //return EXIT_SUCCESS;


  // Validate transit
  ValidateTransit::Validate(pt,onestoptests);

  return EXIT_SUCCESS;
}

