#ifndef ED_PLUGIN_H_
#define ED_PLUGIN_H_

#include <class_loader/class_loader.h>
#define ED_REGISTER_PLUGIN(Derived)  CLASS_LOADER_REGISTER_CLASS(Derived, ed::Plugin)

#include <tue/config/configuration.h>

#include "ed/init_data.h"
#include "ed/arbitraryDataBuffer.h"
#include <boost/shared_ptr.hpp>

namespace ed {

struct WorldModel;
struct UpdateRequest;

struct PluginInput
{
    PluginInput(const WorldModel& world_, const std::vector<UpdateRequestConstPtr>& deltas_)
        : world(world_), deltas(deltas_) {}

    const WorldModel& world;
    const std::vector<UpdateRequestConstPtr>& deltas;
};

class Plugin
{

    friend class PluginContainer;

public:

    virtual ~Plugin() {}

    // Old
    virtual void configure(tue::Configuration config) {}
    virtual void initialize() {}
    virtual void process(const WorldModel& world, UpdateRequest& req) {}

    // New
    virtual void initialize(InitData& init) {}
    virtual void process(const PluginInput& data, UpdateRequest& req) {}

    const std::string& name() const { return name_; }
    
    double getLoopFrequency(){ return loop_frequency_; };
    
    //std::vector<ArbitrayDataBuffer>* getArbitraryDataBuffer(){ return &arbitraryData_;};
   // std::vector<ArbitrayDataBuffer> getArbitraryDataBuffer(){ return arbitraryData_;};
    
        std::vector< boost::shared_ptr<ArbitrayDataBuffer> > arbitraryData_;

private:

    std::string name_;
    
    double loop_frequency_;

};

}

#endif
