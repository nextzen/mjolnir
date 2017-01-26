#include "mjolnir/validatetransit.h"
#include "mjolnir/graphtilebuilder.h"
#include "mjolnir/dataquality.h"
#include "mjolnir/osmrestriction.h"

#include <future>
#include <thread>
#include <queue>

#include <boost/algorithm/string/regex.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/algorithm/string.hpp>

#include <valhalla/midgard/logging.h>
#include <valhalla/midgard/sequence.h>
#include <valhalla/baldr/datetime.h>
#include <valhalla/baldr/tilehierarchy.h>
#include <valhalla/baldr/graphid.h>
#include <valhalla/baldr/graphconstants.h>
#include <valhalla/baldr/graphtile.h>
#include <valhalla/baldr/graphreader.h>

using namespace valhalla::midgard;
using namespace valhalla::baldr;
using namespace valhalla::mjolnir;

// Struct to hold stats information during each threads work
struct validate_stats {
  uint32_t failure_count;

  // Accumulate stats from all threads
  void operator()(const validate_stats& other) {
    failure_count += other.failure_count;
  }
};

bool WalkTransitLines(const GraphId& n_graphId, GraphReader& reader,
                      const GraphId& tileid, std::mutex& lock,
                      const std::string& end_id, const std::string& date_time,
                      const std::string& route_id) {

  lock.lock();
  const GraphTile* endnodetile = reader.GetGraphTile(n_graphId);
  lock.unlock();

  const NodeInfo* n_info = endnodetile->node(n_graphId);
  GraphId currentNode = n_graphId;
  GraphId prev_Node;

  uint32_t j = 0;
  uint32_t localtime = DateTime::seconds_from_midnight(date_time);
  uint32_t date = DateTime::days_from_pivot_date(DateTime::get_formatted_date(date_time));
  uint32_t dow  = DateTime::day_of_week_mask(date_time);
  uint32_t date_created = endnodetile->header()->date_created();
  uint32_t day = date - date_created;
  uint32_t time_added = 0;

  bool bDone = false;

  while (j < n_info->edge_count() && time_added <= 14400) {

    const DirectedEdge* de =
        endnodetile->directededge(n_info->edge_index() + j);

    // all should be a transit lines
    if (de->IsTransitLine() && de->endnode() != prev_Node) {

      const TransitDeparture* departure = endnodetile->GetNextDeparture(
          de->lineid(), localtime, day, dow, false,
          false, false);

      if (departure) {
        const TransitRoute* route = endnodetile->GetTransitRoute(departure->routeid());
        if (endnodetile->GetName(route->one_stop_offset()) == route_id) {
          prev_Node = currentNode;
          currentNode = de->endnode();
          //get the new tile if needed.
          if (endnodetile->id() != currentNode.Tile_Base()) {
            lock.lock();
            endnodetile = reader.GetGraphTile(currentNode);
            lock.unlock();
          }
          //get new end node and start over.
          n_info = endnodetile->node(currentNode);

          const TransitStop* transit_stop = endnodetile->GetTransitStop(
              n_info->stop_index());

          if (endnodetile->GetName(transit_stop->one_stop_offset()) == end_id) {
            bDone = true;
            break;
          }
          localtime = departure->departure_time() + departure->elapsed_time();
          time_added = 0;
          j = 0;
          continue;
        }
      }
    }
    // if we get here add 30 sec and try again.  up to 4 hrs.
    if (j+1 == n_info->edge_count()) {
      time_added += 30;
      localtime += 30;
      j=0;
      continue;
    }
    j++;
  }
  return bDone;
}

void validate(const boost::property_tree::ptree& pt, std::mutex& lock,
              std::unordered_set<GraphId>::const_iterator tile_start,
              std::unordered_set<GraphId>::const_iterator tile_end,
              const std::vector<OneStopTest>& onestoptests,
              std::promise<validate_stats>& results) {

  uint32_t failure_count = 0;
  GraphReader reader_transit_level(pt);
  const TileHierarchy& hierarchy_transit_level = reader_transit_level.GetTileHierarchy();

  // Iterate through the tiles in the queue and find any that include stops
  for(; tile_start != tile_end; ++tile_start) {
    // Get the next tile Id from the queue and get a tile builder
    if(reader_transit_level.OverCommitted())
      reader_transit_level.Clear();
    GraphId tile_id = tile_start->Tile_Base();

    lock.lock();
    GraphId transit_tile_id = GraphId(tile_id.tileid(), tile_id.level()+1, tile_id.id());
    const GraphTile* transit_tile = reader_transit_level.GetGraphTile(transit_tile_id);
    GraphTileBuilder tilebuilder(hierarchy_transit_level, transit_tile_id, true);
    lock.unlock();

    for (uint32_t i = 0; i < tilebuilder.header()->nodecount(); i++) {
      NodeInfo& nodeinfo = tilebuilder.node_builder(i);

      // all should be multiusetransit stop, but check just to be sure.
      if (nodeinfo.type() == NodeType::kMultiUseTransitStop) {

        const TransitStop* transit_stop = transit_tile->GetTransitStop(
            nodeinfo.stop_index());

        OneStopTest ost;
        ost.origin = transit_tile->GetName(transit_stop->one_stop_offset());
        auto p = std::equal_range(onestoptests.begin(), onestoptests.end(), ost);
        for (auto t = p.first; t != p.second; ++t) {

          GraphId currentNode = GraphId(transit_tile->id().tileid(),transit_tile->id().level(), i);
          GraphId tileid = transit_tile->id();

          if (!WalkTransitLines(currentNode, reader_transit_level,
                                tileid, lock,t->destination, t->date_time,
                                t->route_id)) {
            LOG_ERROR("Test from " + t->origin + " to " + t->destination + " @ " +
                      t->date_time + " route id " + t->route_id + " failed.");
            failure_count++;
          } else {
            LOG_INFO("Test from " + t->origin + " to " + t->destination + " @ " +
                     t->date_time + " route id " + t->route_id + " passed.");
          }
        }
      }
    }
  }
  // Send back the statistics
  results.set_value({failure_count});
}

namespace valhalla {
namespace mjolnir {

std::vector<OneStopTest> ParseTestFile(const std::string& filename) {
  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{","};
  std::vector<OneStopTest> onestoptests;
  std::string default_date_time = baldr::DateTime::get_testing_date_time();

  // Open file
  std::string line;
  std::ifstream file(filename);
  if (file.is_open()) {
    while (getline(file, line)) {
      tokenizer tok{line, sep};
      uint32_t field_num = 0;
      OneStopTest onestoptest{};
      for (const auto &t : tok) {
        switch (field_num) {
          case 0:
            onestoptest.origin = remove_parens(t);
            break;
          case 1:
            onestoptest.destination = remove_parens(t);
            break;
          case 2:
            onestoptest.route_id = remove_parens(t);
            break;
          case 3:
            onestoptest.date_time = remove_parens(t);
            break;
        }
        field_num++;
      }
      if (onestoptest.date_time.empty())
        onestoptest.date_time = default_date_time;

      onestoptests.emplace_back(std::move(onestoptest));
    }
    file.close();
  } else {
    std::cout << "One stop test file: " << filename << " not found" << std::endl;
  }

  return onestoptests;
}

/*
void ParseLogFile(const std::string& filename) {
  typedef boost::tokenizer<boost::char_separator<char>> tokenizer;
  boost::char_separator<char> sep{" "};

  std::string type, origin, dest, tranisit_route;
  bool route=false;
  origin = "";
  dest = "";
  tranisit_route = "";
  int num = 0;
  // Open file
  std::string line;
  std::ifstream file(filename);
  if (file.is_open()) {
    while (getline(file, line)) {
      tokenizer tok{line, sep};
      uint32_t field_num = 0;
      OneStopTest onestoptest{};
      for (const auto &t : tok) {
        switch (field_num) {
          case 2:
            type = remove_parens(t);
            break;
          case 3:
            if (type == "[ANALYTICS]") {
              auto o_s = remove_parens(t);
              std::string transit, value;
              std::size_t found = o_s.find_last_of("::");
              if (found != std::string::npos) {
                transit = o_s.substr(0,found-1);
                value = o_s.substr(found+1);
              }

              if (transit == "transit_route_stopid") {
                if (route) {
                  std::cout << origin << "," << dest << "," << tranisit_route << std::endl;
                  origin = "";
                  dest = "";
                  tranisit_route = "";
                }
                tranisit_route = value;
                route = true;
              } else if (transit == "transit_stopid") {
                if (route && origin.empty())
                  origin = value;
                else dest = value;
              } else if (transit == "trip_length") {
                //std::cout << ++num << std::endl;
              }
            } else if (type == "[INFO]"){
              origin = "";
              dest = "";
              tranisit_route = "";
              type = "";
              route = false;
            } else if (route) {
              std::cout << origin << "," << dest << "," << tranisit_route << std::endl;
              origin = "";
              dest = "";
              tranisit_route = "";
              type = "";
              route = false;
            }
            break;
        }
        field_num++;
      }
    }
    file.close();
  } else {
    std::cout << "One stop test file: " << filename << " not found" << std::endl;
  }

}*/

// Enhance the local level of the graph
void ValidateTransit::Validate(const boost::property_tree::ptree& pt,
                               const std::unordered_set<GraphId>& all_tiles,
                               const std::vector<OneStopTest>& onestoptests) {

  unsigned int thread_count = std::max(static_cast<unsigned int>(1), std::thread::hardware_concurrency());

  LOG_INFO("Validating transit network.");

  auto t1 = std::chrono::high_resolution_clock::now();
  if (!all_tiles.size()) {
    LOG_INFO("No transit tiles found. Transit will not be validated.");
    return;
  }

  if (!onestoptests.size()) {
    LOG_INFO("No transit tests found. Transit will not be validated.");
    return;
  }

  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread> > threads(thread_count);

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<validate_stats> > results;

  // Start the threads, divvy up the work
  LOG_INFO("Validating " + std::to_string(all_tiles.size()) + " transit tiles...");
  size_t floor = all_tiles.size() / threads.size();
  size_t at_ceiling = all_tiles.size() - (threads.size() * floor);
  std::unordered_set<GraphId>::const_iterator tile_start, tile_end = all_tiles.begin();

  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    results.emplace_back();
    threads[i].reset(
        new std::thread(validate, std::cref(pt.get_child("mjolnir")),
                        std::ref(lock), tile_start, tile_end,
                        std::cref(onestoptests), std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }


  // Check all of the outcomes, to see about maximum density (km/km2)
  validate_stats stats{};
  uint32_t failure_count = 0;
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
      failure_count += stats.failure_count;
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }

  if (failure_count)
    LOG_ERROR("There were " + std::to_string(failure_count) + " failures!");


  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  LOG_INFO("Finished validating transit network - took " + std::to_string(secs) + " secs");

}

// Enhance the local level of the graph
void ValidateTransit::Validate(const boost::property_tree::ptree& pt,
                               const std::vector<OneStopTest>& onestoptests) {

  unsigned int thread_count = std::max(static_cast<unsigned int>(1), std::thread::hardware_concurrency());

  LOG_INFO("Validating transit network.");

  auto t1 = std::chrono::high_resolution_clock::now();
  std::unordered_set<GraphId> all_tiles;

  // Bail if nothing
  auto hierarchy_properties = pt.get_child("mjolnir");
  auto transit_dir = hierarchy_properties.get_optional<std::string>("transit_dir");
  if(!transit_dir || !boost::filesystem::exists(*transit_dir) || !boost::filesystem::is_directory(*transit_dir)) {
    LOG_INFO("Transit directory not found. Transit will not be added.");
    return;
  }
  // Also bail if nothing inside
  transit_dir->push_back('/');
  GraphReader reader(hierarchy_properties);
  const auto& hierarchy = reader.GetTileHierarchy();
  auto local_level = hierarchy.levels().rbegin()->first;
  if(boost::filesystem::is_directory(*transit_dir + std::to_string(local_level + 1) + "/")) {
    boost::filesystem::recursive_directory_iterator transit_file_itr(*transit_dir + std::to_string(local_level +1 ) + "/"), end_file_itr;
    for(; transit_file_itr != end_file_itr; ++transit_file_itr) {
      if(boost::filesystem::is_regular(transit_file_itr->path()) && transit_file_itr->path().extension() == ".gph") {
        auto graph_id = GraphTile::GetTileId(transit_file_itr->path().string());
        auto local_graph_id = graph_id;
        local_graph_id.fields.level -= 1;
        if(GraphReader::DoesTileExist(hierarchy_properties, local_graph_id)) {
          const GraphTile* tile = reader.GetGraphTile(local_graph_id);
          all_tiles.emplace(local_graph_id);
          const std::string destination_path = pt.get<std::string>("mjolnir.tile_dir") + '/' + GraphTile::FileSuffix(graph_id, hierarchy);
          boost::filesystem::path filename = destination_path;
          // Make sure the directory exists on the system and copy to the tile_dir
          if (!boost::filesystem::exists(filename.parent_path()))
            boost::filesystem::create_directories(filename.parent_path());
          boost::filesystem::copy_file(transit_file_itr->path(),destination_path,boost::filesystem::copy_option::overwrite_if_exists);
        }
      }
    }
  }
  if (!all_tiles.size()) {
    LOG_INFO("No transit tiles found. Transit will not be validated.");
    return;
  }

  if (!onestoptests.size()) {
    LOG_INFO("No transit tests found. Transit will not be validated.");
    return;
  }

  // A place to hold worker threads and their results
  std::vector<std::shared_ptr<std::thread> > threads(thread_count);

  // An atomic object we can use to do the synchronization
  std::mutex lock;

  // A place to hold the results of those threads (exceptions, stats)
  std::list<std::promise<validate_stats> > results;

  // Start the threads, divvy up the work
  LOG_INFO("Validating " + std::to_string(all_tiles.size()) + " transit tiles...");
  size_t floor = all_tiles.size() / threads.size();
  size_t at_ceiling = all_tiles.size() - (threads.size() * floor);
  std::unordered_set<GraphId>::const_iterator tile_start, tile_end = all_tiles.begin();

  // Atomically pass around stats info
  for (size_t i = 0; i < threads.size(); ++i) {
    // Figure out how many this thread will work on (either ceiling or floor)
    size_t tile_count = (i < at_ceiling ? floor + 1 : floor);
    // Where the range begins
    tile_start = tile_end;
    // Where the range ends
    std::advance(tile_end, tile_count);
    // Make the thread
    results.emplace_back();
    threads[i].reset(
        new std::thread(validate, std::cref(pt.get_child("mjolnir")),
                        std::ref(lock), tile_start, tile_end,
                        std::cref(onestoptests), std::ref(results.back())));
  }

  // Wait for them to finish up their work
  for (auto& thread : threads) {
    thread->join();
  }


  // Check all of the outcomes, to see about maximum density (km/km2)
  validate_stats stats{};
  uint32_t failure_count = 0;
  for (auto& result : results) {
    // If something bad went down this will rethrow it
    try {
      auto thread_stats = result.get_future().get();
      stats(thread_stats);
      failure_count += stats.failure_count;
    }
    catch(std::exception& e) {
      //TODO: throw further up the chain?
    }
  }

  if (failure_count)
    LOG_ERROR("There were " + std::to_string(failure_count) + " failures!");


  auto t2 = std::chrono::high_resolution_clock::now();
  uint32_t secs = std::chrono::duration_cast<std::chrono::seconds>(t2 - t1).count();
  LOG_INFO("Finished validating transit network - took " + std::to_string(secs) + " secs");

}

}
}
