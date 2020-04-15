#ifndef IGVC_FLATLAND_RANDOM_GATES_H_
#define IGVC_FLATLAND_RANDOM_GATES_H_

#include <Box2D/Box2D.h>
#include <flatland_server/types.h>
#include <flatland_server/world.h>
#include <flatland_server/world_plugin.h>

namespace flatland_plugins
{

class RandomGates : public flatland_server::WorldPlugin
{
public:
    void OnInitialize ( const YAML::Node &config );
};

}

#endif // IGVC_FLATLAND_RANDOM_GATES_H_
