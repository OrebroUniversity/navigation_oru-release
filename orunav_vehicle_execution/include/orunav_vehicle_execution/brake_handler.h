#ifndef DYNAMIC_CONSTRAINT_BRAKE_HANDLER_H
#define DYNAMIC_CONSTRAINT_BRAKE_HANDLER_H

#include <sauna_msgs/BrakeCommand.h>
#include <map>

class BrakeHandler {
 public:
  bool update(const sauna_msgs::BrakeCommandConstPtr &command, bool &slow_down, std::vector<int> &ids) {
    
    // Do we have this id already?
    if (commands_.count(command->sender_id) == 0) {
      commands_.insert(std::pair<int, sauna_msgs::BrakeCommand>(command->sender_id, *command));
    }
    else {
      // Owerwrite the current instance.
      commands_[command->sender_id] = *command;
    }

    if (doBraking(ids))
      return true;
    
    slow_down = doSlowDown(ids);
    
    return false;
  }
  
 private:
  bool doBraking(std::vector<int> &ids) const {
    ids.clear();
    for (std::map<int, sauna_msgs::BrakeCommand>::const_iterator it=commands_.begin(); it!=commands_.end(); ++it) {
      if (it->second.brake_flag == it->second.BRAKE) {
	ids.push_back(it->first);
      }
    }
    if (ids.empty())
      return false;
    return true;
  }

  bool doSlowDown(std::vector<int> &ids) const {
    ids.clear();
    for (std::map<int, sauna_msgs::BrakeCommand>::const_iterator it=commands_.begin(); it!=commands_.end(); ++it) {
      if (it->second.brake_flag == it->second.SLOW_DOWN) {
	ids.push_back(it->first);
      }
    }
    if (ids.empty())
      return false;
    return true;
  }

  
  std::map<int, sauna_msgs::BrakeCommand> commands_;
};

#endif

