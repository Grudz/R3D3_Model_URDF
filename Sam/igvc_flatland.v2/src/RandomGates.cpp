#include <igvc_flatland/RandomGates.h>
#include <flatland_server/layer.h>
#include <pluginlib/class_list_macros.h>
#include <random>
#include <chrono>

using namespace flatland_server;

namespace flatland_plugins
{

void RandomGates::OnInitialize(const YAML::Node &config)
{
  // Load YAML parameters
  YamlReader plugin_reader(config);
  double gate_width = plugin_reader.Get<double>("gate_width", 1.0);
  YamlReader gate_coords_reader = plugin_reader.Subnode("gates", YamlReader::LIST);
  std::vector<flatland_server::Vec2> gate_coords = gate_coords_reader.AsList<flatland_server::Vec2>(1, -1);
  std::string layer_name = plugin_reader.Get<std::string>("layer", "");
  Layer *layer = NULL;
  std::vector<std::string> cfr_names;
  for (auto & it : world_->layers_name_map_) {
    for (auto & v_it : it.first) {
      if (v_it == layer_name) {
        layer = it.second;
        cfr_names = it.first;
        break;
      }
    }
  }
  if (layer == NULL) {
    throw ("no such layer name!");
  }

  // Sample system time in nanoseconds and use lowest 32 bits to seed RNG
  std::chrono::time_point<std::chrono::high_resolution_clock> time_sample = std::chrono::high_resolution_clock::now();
  uint64_t long_sample = std::chrono::duration_cast<std::chrono::nanoseconds>(time_sample.time_since_epoch()).count();
  uint32_t truncated_sample = (uint32_t)(long_sample & 0x00000000FFFFFFFF);
  std::default_random_engine generator(truncated_sample);

  // Sample RNG to randomize open gate for current run
  std::uniform_int_distribution<int> distribution(0, gate_coords.size() - 1);
  int open_gate = distribution(generator);

  // Spawn each gate except for the randomly selected one
  for (int i = 0; i < gate_coords.size(); i++) {
    if (i != open_gate) {
      // Vertices of the gate edges
      std::vector<b2Vec2> v(4);
      double offset_x = -layer->body_->GetPhysicsBody()->GetPosition().x;
      double offset_y = -layer->body_->GetPhysicsBody()->GetPosition().y;
      v[0].Set(gate_coords[i].x - 0.5 * gate_width + offset_x, gate_coords[i].y - 0.1 + offset_y);
      v[1].Set(gate_coords[i].x - 0.5 * gate_width + offset_x, gate_coords[i].y + 0.1 + offset_y);
      v[2].Set(gate_coords[i].x + 0.5 * gate_width + offset_x, gate_coords[i].y + 0.1 + offset_y);
      v[3].Set(gate_coords[i].x + 0.5 * gate_width + offset_x, gate_coords[i].y - 0.1 + offset_y);

      // Edges of the gate shape
      std::vector<b2EdgeShape> new_gate(4);
      new_gate[0].Set(v[0], v[1]);
      new_gate[1].Set(v[1], v[2]);
      new_gate[2].Set(v[2], v[3]);
      new_gate[3].Set(v[3], v[0]);

      // Populate the layer with the gate shape
      for (auto & gate_edge : new_gate) {
        b2FixtureDef fixture_def;
        fixture_def.shape = &gate_edge;
        uint16_t categoryBits = layer->cfr_->GetCategoryBits(cfr_names);
        fixture_def.filter.categoryBits = categoryBits;
        fixture_def.filter.maskBits = categoryBits;
        layer->body_->physics_body_->CreateFixture(&fixture_def);
      }
    }
  }
}

}

PLUGINLIB_EXPORT_CLASS(flatland_plugins::RandomGates, flatland_server::WorldPlugin)
