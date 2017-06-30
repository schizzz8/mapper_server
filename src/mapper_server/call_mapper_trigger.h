#pragma once

#include <srrg_messages/sensor_message_sorter.h>
#include <srrg_messages/message_timestamp_synchronizer.h>
#include <srrg_messages/pinhole_image_message.h>

#include "mapper.h"

namespace mapper_server {
  
  class CallMapperTrigger: public srrg_core::SensorMessageSorter::Trigger{
  public:
    CallMapperTrigger(srrg_core::SensorMessageSorter* sorter,
		       int priority,
               Mapper* mapper_,
		       std::vector<srrg_core::MessageTimestampSynchronizer>* synchronizers_=0);
    virtual void action(std::tr1::shared_ptr<srrg_core::BaseSensorMessage> msg);
    
    inline Mapper* mapper() {return _mapper;}
  protected:
    Mapper* _mapper;
    std::vector<srrg_core::MessageTimestampSynchronizer>* _synchronizers;
    srrg_core::PinholeImageMessage* _depth_img, *_rgb_img;
  };
}
