#include <foundation/union_find.h>
#include <map/tracks_manager.h>

#include <optional>
#include <sstream>
#include <unordered_set>

namespace {

struct TrackLengths {
  uint16_t imageLen;
  uint16_t trackIdLen;
};

struct TrackRecord {
  int featureID;
  float x;
  float y;
  float scale;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  // int segm;
  // int inst;
};

template <class S>
int GetTracksFileVersion(S& fstream) {
  const auto current_position = fstream.tellg();
  std::string line;
  std::getline(fstream, line);
  int version = 0;
  if (line.find(sfmmap::TracksManager::TRACKS_HEADER) == 0) {
    version = std::atoi(line.substr(sfmmap::TracksManager::TRACKS_HEADER.length() + 2).c_str());
  } else {
    fstream.seekg(current_position);
  }
  return version;
}

template <class S>
void WriteToStreamCurrentVersion(S& ostream, const sfmmap::TracksManager& manager) {
  ostream << manager.TRACKS_HEADER << "_v" << manager.TRACKS_VERSION << std::endl;
  TrackLengths tl;
  TrackRecord tr;
  const auto shotsIDs = manager.GetShotIds();
  for (const auto& shotID : shotsIDs) {
    const auto observations = manager.GetShotObservations(shotID);
    for (const auto& observation : observations) {
      tl.imageLen = static_cast<uint16_t>(shotID.length());
      tl.trackIdLen = static_cast<uint16_t>(observation.first.length());
      tr.featureID = observation.second.feature_id;
      tr.x = observation.second.point(0);
      tr.y = observation.second.point(1);
      tr.scale = observation.second.scale;
      tr.r = static_cast<uint8_t>(observation.second.color(0));
      tr.g = static_cast<uint8_t>(observation.second.color(1));
      tr.b = static_cast<uint8_t>(observation.second.color(2));
      // tr.segm = observation.second.segmentation_id;
      // tr.inst = observation.second.instance_id;
      ostream.write(reinterpret_cast<char*>(&tl), sizeof(tl));
      ostream << shotID << observation.first;
      ostream.write(reinterpret_cast<char*>(&tr), sizeof(tr));
    }
  }
}

sfmmap::Observation InstanciateObservation(double x, double y, double scale, int id,
                                           int r, int g, int b,
                                           int segm = sfmmap::Observation::NO_SEMANTIC_VALUE,
                                           int inst = sfmmap::Observation::NO_SEMANTIC_VALUE) {
  sfmmap::Observation observation;
  observation.point << x, y;
  observation.scale = scale;
  observation.feature_id = id;
  observation.color << r, g, b;
  observation.segmentation_id = segm;
  observation.instance_id = inst;
  return observation;
}

void SeparateLineByTabs(const std::string& line, std::vector<std::string>& elems) {
  elems.clear();
  std::stringstream ss(line);
  std::string elem;
  while (std::getline(ss, elem, '\t')) {
    elems.push_back(elem);
  }
}

template <class S>
sfmmap::TracksManager InstanciateFromStreamV0(S& fstream) {
  sfmmap::TracksManager manager;
  std::string line;
  std::vector<std::string> elems;
  constexpr int N_ENTRIES = 8;
  elems.reserve(N_ENTRIES);
  while (std::getline(fstream, line)) {
    SeparateLineByTabs(line, elems);
    if (elems.size() != N_ENTRIES) {
      throw std::runtime_error("Invalid line: expected " + std::to_string(N_ENTRIES) + " values");
    }
    const sfmmap::ShotId image = elems[0];
    const sfmmap::TrackId trackID = elems[1];
    int featureID = std::stoi(elems[2]);
    double x = std::stod(elems[3]);
    double y = std::stod(elems[4]);
    double scale = 0.0;
    int r = std::stoi(elems[5]);
    int g = std::stoi(elems[6]);
    int b = std::stoi(elems[7]);
    auto observation = InstanciateObservation(x, y, scale, featureID, r, g, b);
    manager.AddObservation(image, trackID, observation);
  }
  return manager;
}

template <class S>
sfmmap::TracksManager InstanciateFromStreamV1(S& fstream) {
  sfmmap::TracksManager manager;
  std::string line;
  std::vector<std::string> elems;
  constexpr int N_ENTRIES = 9;
  elems.reserve(N_ENTRIES);
  while (std::getline(fstream, line)) {
    SeparateLineByTabs(line, elems);
    if (elems.size() != N_ENTRIES) {
      throw std::runtime_error("Invalid line: expected " + std::to_string(N_ENTRIES) + " values");
    }
    const sfmmap::ShotId image = elems[0];
    const sfmmap::TrackId trackID = elems[1];
    int featureID = std::stoi(elems[2]);
    double x = std::stod(elems[3]);
    double y = std::stod(elems[4]);
    double scale = std::stod(elems[5]);
    int r = std::stoi(elems[6]);
    int g = std::stoi(elems[7]);
    int b = std::stoi(elems[8]);
    auto observation = InstanciateObservation(x, y, scale, featureID, r, g, b);
    manager.AddObservation(image, trackID, observation);
  }
  return manager;
}

template <class S>
sfmmap::TracksManager InstanciateFromStreamV2(S& fstream) {
  sfmmap::TracksManager manager;
  std::string line;
  std::vector<std::string> elems;
  constexpr int N_ENTRIES = 11;
  elems.reserve(N_ENTRIES);
  while (std::getline(fstream, line)) {
    SeparateLineByTabs(line, elems);
    if (elems.size() != N_ENTRIES) {
      throw std::runtime_error("Invalid line: expected " + std::to_string(N_ENTRIES) + " values");
    }
    const sfmmap::ShotId image = elems[0];
    const sfmmap::TrackId trackID = elems[1];
    int featureID = std::stoi(elems[2]);
    double x = std::stod(elems[3]);
    double y = std::stod(elems[4]);
    double scale = std::stod(elems[5]);
    int r = std::stoi(elems[6]);
    int g = std::stoi(elems[7]);
    int b = std::stoi(elems[8]);
    int segm = std::stoi(elems[9]);
    int inst = std::stoi(elems[10]);
    auto observation = InstanciateObservation(x, y, scale, featureID, r, g, b, segm, inst);
    manager.AddObservation(image, trackID, observation);
  }
  return manager;
}

sfmmap::TracksManager InstanciateFromFilenameBinaryV2(std::ifstream& fstream,
                                                      const std::string& filename) {
  if (fstream.is_open()) fstream.close();
  std::ifstream fs(filename, std::ios_base::binary);
  sfmmap::TracksManager manager;
  char buffer[131072];
  std::string version;
  std::getline(fs, version);
  TrackLengths tl;
  TrackRecord tr;
  while (!fs.eof()) {
    fs.read(reinterpret_cast<char*>(&tl), sizeof(TrackLengths));
    fs.read(buffer, tl.imageLen + tl.trackIdLen);
    std::string image(buffer, tl.imageLen);
    std::string trackID(buffer + tl.imageLen, tl.trackIdLen);
    fs.read(reinterpret_cast<char*>(&tr), sizeof(TrackRecord));
    auto observation = InstanciateObservation(tr.x, tr.y, tr.scale,
                                              tr.featureID, tr.r, tr.g, tr.b, -1, -1);
    manager.AddObservation(image, trackID, observation);
  }
  return manager;
}

template <class S>
sfmmap::TracksManager InstanciateFromStreamT(S& fstream, const std::string& filename = "") {
  int version = GetTracksFileVersion(fstream);
  switch (version) {
    case 0:   return InstanciateFromStreamV0(fstream);
    case 1:   return InstanciateFromStreamV1(fstream);
    case 2:   return InstanciateFromStreamV2(fstream);
    case 102: return InstanciateFromFilenameBinaryV2(fstream, filename);
    default:  throw std::runtime_error("Unknown tracks manager file version");
  }
}

}  // namespace

namespace sfmmap {

void TracksManager::AddObservation(const ShotId& shot_id, const TrackId& track_id,
                                   const Observation& observation) {
  tracks_per_shot_[shot_id][track_id] = observation;
  shots_per_track_[track_id][shot_id] = observation;
}

void TracksManager::RemoveObservation(const ShotId& shot_id, const TrackId& track_id) {
  auto find_shot = tracks_per_shot_.find(shot_id);
  if (find_shot == tracks_per_shot_.end()) {
    throw std::runtime_error("Accessing invalid shot ID");
  }
  auto find_track = shots_per_track_.find(track_id);
  if (find_track == shots_per_track_.end()) {
    throw std::runtime_error("Accessing invalid track ID");
  }
  find_shot->second.erase(track_id);
  find_track->second.erase(shot_id);
}

int TracksManager::NumShots() const { return tracks_per_shot_.size(); }
int TracksManager::NumTracks() const { return shots_per_track_.size(); }

bool TracksManager::HasShotObservations(const ShotId& shot) const {
  return tracks_per_shot_.count(shot) > 0;
}

std::vector<ShotId> TracksManager::GetShotIds() const {
  std::vector<ShotId> shots;
  shots.reserve(tracks_per_shot_.size());
  for (const auto& it : tracks_per_shot_) {
    shots.push_back(it.first);
  }
  return shots;
}

std::vector<TrackId> TracksManager::GetTrackIds() const {
  std::vector<TrackId> tracks;
  tracks.reserve(shots_per_track_.size());
  for (const auto& it : shots_per_track_) {
    tracks.push_back(it.first);
  }
  return tracks;
}

Observation TracksManager::GetObservation(const ShotId& shot, const TrackId& track) const {
  auto find_shot = tracks_per_shot_.find(shot);
  if (find_shot == tracks_per_shot_.end()) {
    throw std::runtime_error("Accessing invalid shot ID");
  }
  auto find_track = find_shot->second.find(track);
  if (find_track == find_shot->second.end()) {
    throw std::runtime_error("Accessing invalid track ID");
  }
  return find_track->second;
}

const std::unordered_map<TrackId, Observation>&
TracksManager::GetShotObservations(const ShotId& shot) const {
  auto find_shot = tracks_per_shot_.find(shot);
  if (find_shot == tracks_per_shot_.end()) {
    throw std::runtime_error("Accessing invalid shot ID");
  }
  return find_shot->second;
}

const std::unordered_map<ShotId, Observation>&
TracksManager::GetTrackObservations(const TrackId& track) const {
  auto find_track = shots_per_track_.find(track);
  if (find_track == shots_per_track_.end()) {
    throw std::runtime_error("Accessing invalid track ID");
  }
  return find_track->second;
}

TracksManager TracksManager::ConstructSubTracksManager(
    const std::vector<TrackId>& tracks,
    const std::vector<ShotId>& shots) const {
  std::unordered_set<ShotId> shots_set;
  for (const auto& id : shots) {
    shots_set.insert(id);
  }
  TracksManager subset;
  for (const auto& track_id : tracks) {
    auto find_track = shots_per_track_.find(track_id);
    if (find_track == shots_per_track_.end()) continue;
    for (const auto& obs : find_track->second) {
      const auto& shot_id = obs.first;
      if (shots_set.find(shot_id) == shots_set.end()) continue;
      subset.AddObservation(shot_id, track_id, obs.second);
    }
  }
  return subset;
}

std::vector<TracksManager::KeyPointTuple>
TracksManager::GetAllCommonObservations(const ShotId& shot1,
                                        const ShotId& shot2) const {
  auto find_shot1 = tracks_per_shot_.find(shot1);
  auto find_shot2 = tracks_per_shot_.find(shot2);
  if (find_shot1 == tracks_per_shot_.end() ||
      find_shot2 == tracks_per_shot_.end()) {
    throw std::runtime_error("Accessing invalid shot ID");
  }
  std::vector<KeyPointTuple> tuples;
  for (const auto& p : find_shot1->second) {
    auto find = find_shot2->second.find(p.first);
    if (find != find_shot2->second.end()) {
      tuples.emplace_back(p.first, p.second, find->second);
    }
  }
  return tuples;
}

std::unordered_map<TracksManager::ShotPair, int, HashPair>
TracksManager::GetAllPairsConnectivity(const std::vector<ShotId>& shots,
                                       const std::vector<TrackId>& tracks) const {
  std::unordered_map<ShotPair, int, HashPair> common_per_pair;
  std::vector<TrackId> tracks_to_use;
  if (tracks.empty()) {
    for (const auto& kv : shots_per_track_) {
      tracks_to_use.push_back(kv.first);
    }
  } else {
    tracks_to_use = tracks;
  }
  std::unordered_set<ShotId> shots_to_use;
  if (shots.empty()) {
    for (const auto& kv : tracks_per_shot_) {
      shots_to_use.insert(kv.first);
    }
  } else {
    for (const auto& shot : shots) {
      shots_to_use.insert(shot);
    }
  }
  for (const auto& track_id : tracks_to_use) {
    auto find_track = shots_per_track_.find(track_id);
    if (find_track == shots_per_track_.end()) continue;
    const auto& track_obs = find_track->second;
    for (const auto& it1 : track_obs) {
      const auto& shot_id1 = it1.first;
      if (shots_to_use.find(shot_id1) != shots_to_use.end()) {
        for (const auto& it2 : track_obs) {
          const auto& shot_id2 = it2.first;
          if (shot_id1 < shot_id2 &&
              shots_to_use.find(shot_id2) != shots_to_use.end()) {
            ++common_per_pair[{shot_id1, shot_id2}];
          }
        }
      }
    }
  }
  return common_per_pair;
}

TracksManager TracksManager::MergeTracksManager(
    const std::vector<const TracksManager*>& tracks_managers) {
  using FeatureIdPair = std::pair<ShotId, int>;
  using SingleTrackId = std::pair<TrackId, int>;
  std::vector<std::unique_ptr<UnionFindElement<SingleTrackId>>> uf_elements;
  std::unordered_map<FeatureIdPair, std::vector<int>, HashPair> observations;
  for (int i = 0; i < (int)tracks_managers.size(); ++i) {
    const auto& manager = tracks_managers[i];
    for (const auto& track_obs : manager->shots_per_track_) {
      int element_id = uf_elements.size();
      for (const auto& shot_obs : track_obs.second) {
        observations[{shot_obs.first, shot_obs.second.feature_id}].push_back(element_id);
      }
      uf_elements.emplace_back(std::make_unique<UnionFindElement<SingleTrackId>>(
          std::make_pair(track_obs.first, i)));
    }
  }
  TracksManager merged;
  if (uf_elements.empty()) {
    return merged;
  }
  for (const auto& obs_group : observations) {
    if (obs_group.second.empty()) continue;
    auto* e1 = uf_elements[obs_group.second[0]].get();
    for (size_t j = 1; j < obs_group.second.size(); ++j) {
      auto* e2 = uf_elements[obs_group.second[j]].get();
      Union(e1, e2);
    }
  }
  const auto clusters = GetUnionFindClusters(&uf_elements);
  for (size_t i = 0; i < clusters.size(); ++i) {
    const auto& tracks_group = clusters[i];
    TrackId merged_track_id = std::to_string(i);
    for (const auto& node : tracks_group) {
      int manager_id = node->data.second;
      const TrackId& track_id = node->data.first;
      const auto& observations_map = tracks_managers[manager_id]->shots_per_track_.at(track_id);
      for (const auto& shot_obs : observations_map) {
        merged.AddObservation(shot_obs.first, merged_track_id, shot_obs.second);
      }
    }
  }
  return merged;
}

TracksManager TracksManager::InstanciateFromFile(const std::string& filename) {
  std::ifstream istream(filename);
  if (istream.is_open()) {
    return ::InstanciateFromStreamT(istream, filename);
  } else {
    throw std::runtime_error("Can't read tracks manager file");
  }
}

void TracksManager::WriteToFile(const std::string& filename) const {
  std::ofstream ostream(filename, std::ios_base::binary);
  if (ostream.is_open()) {
    ::WriteToStreamCurrentVersion(ostream, *this);
  } else {
    throw std::runtime_error("Can't write tracks manager file");
  }
}

TracksManager TracksManager::InstanciateFromString(const std::string& str) {
  throw std::runtime_error("Instantiation from string not supported: " + str);
}

std::string TracksManager::AsString() const {
  throw std::runtime_error("Serialization to string not supported");
}

std::string TracksManager::TRACKS_HEADER = "OPENSFM_TRACKS_VERSION";
int TracksManager::TRACKS_VERSION = 102;

}  // namespace sfmmap
