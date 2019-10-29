#ifndef ED_PLUGIN_H_
#define ED_PLUGIN_H_

#include <class_loader/class_loader.h>
#define ED_REGISTER_PLUGIN(Derived)  CLASS_LOADER_REGISTER_CLASS(Derived, ed::Plugin)

#include <tue/config/configuration.h>

#include "ed/init_data.h"
#include "ed/arbitraryDataBuffer.h"
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp> 

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

    virtual ~Plugin() 
    {
            if(dataBuffer_)
            {
                   removeDatabuffer();             
            }
    }

    // Old
    virtual void configure(tue::Configuration config) {}
    virtual void initialize() {}
    virtual void process(const WorldModel& world, UpdateRequest& req) {}

    // New
    virtual void initialize(InitData& init) {}
    virtual void process(const PluginInput& data, UpdateRequest& req) {}

    const std::string& name() const { return name_; }
    
    double getLoopFrequency(){ return loop_frequency_; };
    
    boost::shared_ptr< std::vector< boost::shared_ptr<ArbitrayDataBuffer> > > arbitraryDatabuffers_;
    
    bool newBufferDesired(std::string bufferName)
    {
            std::vector< boost::shared_ptr<ArbitrayDataBuffer> >::iterator it;           
            
            for(it = arbitraryDatabuffers_->begin(); it != arbitraryDatabuffers_->end(); it++)
            {
                boost::shared_ptr<ArbitrayDataBuffer> databuffer = *it;
            
                std::cout << "new buffer desired bufferName = " << databuffer->name  << " input name = " << bufferName << std::endl;
                
                if(databuffer->name == bufferName)
                {                        
                        dataBuffer_ = databuffer;
                        std::cout << "For pluging " << name_ << " buffer with same name exists. databuffer = " << dataBuffer_ << std::endl;
                        return false;
                }
            }
            
            return true;
    }
    
    void createDatabuffer(boost::shared_ptr<ArbitrayDataBuffer> sharedPointer, std::string bufferName)
    { 
           if(dataBuffer_)
           {
                  std::cout << "WARN: Old databuffer removed and new one created." << std::endl;
                  removeDatabuffer();
           }
            
           dataBuffer_ = sharedPointer;
           dataBuffer_->name = bufferName;
           
           arbitraryDatabuffers_->push_back(dataBuffer_);
           std::cout << "For pluging " << name_ << ": dataBuffer created with name = " << bufferName << std::endl;
        };
        
     void removeDatabuffer()
     {
             std::vector<boost::shared_ptr<ArbitrayDataBuffer> >::iterator it = std::find(arbitraryDatabuffers_->begin(), arbitraryDatabuffers_->end(), dataBuffer_);
             if(it != arbitraryDatabuffers_->end() )
             {
                     arbitraryDatabuffers_->erase (it); // TODO make other plugins aware that this item is removed
             }
                   
             dataBuffer_.reset();
             
             std::cout << "Destructor: databuffer_ = " << dataBuffer_ << std::endl;
           //  dataBuffer_ = 0;
     }
     
     boost::shared_ptr<ArbitrayDataBuffer> getDataBuffer( ) {return dataBuffer_; }
         

private:

    std::string name_;
    
    double loop_frequency_;
    
    boost::shared_ptr<ArbitrayDataBuffer> dataBuffer_; // TODO acces to multiple buffers?
    
};

}

#endif
