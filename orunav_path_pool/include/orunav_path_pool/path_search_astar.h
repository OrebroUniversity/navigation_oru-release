#ifndef PATH_SEARCH_ASTAR_H
#define PATH_SEARCH_ASTAR_H

#include <orunav_path_pool/graph_astar.h>
#include <orunav_generic/interfaces.h>
#include <orunav_generic/path_utils.h>

class PathSearchASTAR 
{
 public:
  PathSearchASTAR()
    {
    }

  void assignPaths(const std::vector<orunav_generic::Path> &paths)
  {
    paths_ = paths;
    // Setup the graph
    // 1), need to find all nodes (start / end poses) - only one node should be added per pose.
    // 2), add all paths as edges, for each path find the start pose, look up the corresponding node and add the adge as an "out edge". -> the goal pose is added into the edge.nextNode as a pointer...
    // 3), fill the graph
    graph_.clear();
    nodes_.clear();

    // Note, Path means the same as "StateContainer".
    orunav_generic::Path start_states = orunav_generic::getStartStatesFromPaths(paths);
    orunav_generic::Path goal_states = orunav_generic::getGoalStatesFromPaths(paths);

    assert(start_states.sizePath() == goal_states.sizePath());
    assert(paths.size() == start_states.sizePath());

    orunav_generic::Path nodes;
    const double max_offsets = 0.01;

    orunav_generic::addUniqueStates(nodes, start_states, max_offsets, 0.1, 0.1);
    orunav_generic::addUniqueStates(nodes, goal_states, max_offsets, 0.1, 0.1);
    
    // Fill in the nodes.
    for (unsigned int i = 0; i < nodes.sizePath(); i++) {
      astar_graph_plan::XYTh_GraphNode n;
      n.setId(i);
      const orunav_generic::Pose2d &p = nodes.getPose2d(i);
      n.setState(p[0], p[1], p[2]);
      nodes_.push_back(n); // Memory allocation goes here.
    }

    // Fix the edges (get the start pose node id),
    for (unsigned int i = 0; i < paths.size(); i++) {
      orunav_generic::State2d start_s(start_states, i);
      orunav_generic::State2d goal_s(goal_states, i);
      int idx_start = orunav_generic::findState(nodes, start_s, max_offsets, 0.1, 0.1);
      int idx_goal = orunav_generic::findState(nodes, goal_s, max_offsets, 0.1, 0.1);
      assert(idx_start >= 0);
      assert(idx_goal >= 0);
      
      astar_graph_plan::OutGraphEdge e;
      e.id = i;
      e.nextNode = &nodes_[idx_goal];
      e.cost = orunav_generic::getTotalDistance(paths[i]);
      nodes_[idx_start].addOutEdge(e);
    }
    
    for (unsigned int i = 0; i < nodes_.size(); i++) {
      graph_.addNode(&nodes_[i]);
    }
  }

  bool findPath(orunav_generic::Path &p, const orunav_generic::Pose2d &start, const orunav_generic::Pose2d &goal, double maxDistOffset, double maxAngleOffset ) const 
  {
    astar_graph_plan::SearchContext ctxt;
    ctxt.graph = graph_;

    // The provided start pose is given using the localization system and could therefore be a bit off. Therefore we do an initial check here to assign the values to be from a node (if suifficient close to one).
    orunav_generic::Path start_states = orunav_generic::getStartStatesFromPaths(paths_);
    orunav_generic::State2d s(start, 0.);
    //FPA: changed to use offsets instead of hardcoded values
    //int idx = orunav_generic::findState(start_states, s, 0.1, 0.1, 0.1);
    int idx = orunav_generic::findState(start_states, s, maxDistOffset, maxAngleOffset, 0.5);
    if (idx < 0) {
      return false; 
    }
    orunav_generic::Pose2d start_state = start_states.getPose2d(idx);
    ctxt.spt = { start_state[0], start_state[1], start_state[2] };
    ctxt.ept = { goal[0], goal[1], goal[2] };
    vector <int> edge_ids;
    if(ctxt.findPath(edge_ids))
      {
	cout << "here is the A* solution:\n";
	for(unsigned int i = 0; i < edge_ids.size(); i++)
	  {
	    cout << "Step: " << i << ": Edge id  " <<edge_ids[i] << endl;
	  }
      }
    if (edge_ids.empty())
      return false;
    
    
    std::vector<orunav_generic::Path> tmp;
    for (unsigned int i = 0; i < edge_ids.size(); i++) 
      {
	tmp.push_back(paths_[edge_ids[i]]);
      }
    
    p = orunav_generic::concatenateDirectionalPaths(tmp);

    return true;
  }

  void printDebug() { // cannot be const due to graph_
    std::cout << "number of nodes : " << nodes_.size() << std::endl;
    std::cout << "number of paths : " << paths_.size() << std::endl;
    graph_.printNodes();
  }

 private:
  std::vector<astar_graph_plan::XYTh_GraphNode> nodes_; // Store as vectors to avoid allocation and deallocation crap. Note edges are not needed to be stored, there the data is copied...
  astar_graph_plan::ListGraph graph_;
  std::vector<orunav_generic::Path> paths_;
};




#endif
