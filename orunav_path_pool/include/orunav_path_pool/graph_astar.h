



#ifndef  _SPLINES_GRAPH_SEARCH_H_
#define  _SPLINES_GRAPH_SEARCH_H_



#include <iostream>
#include <map>
#include <vector>
#include <cmath>

#include "../../src/stlastar.h" // See header for copyright and usage information

namespace astar_graph_plan
{

#define NORM_ANG(a) (atan2 (sin(a), cos(a)))

     typedef struct
     {
	  double x,y,th;
     } XYTH_t;
     
     typedef struct
     {
	  size_t id;
	  XYTH_t state;
     } XYTH_with_id_t;
     
     typedef struct
     {
	  size_t id; // if of the edge

	  //ids of start and end point of the edge
	  size_t sPt_id, ePt_id;   
	  // cost of traversing the edge
	  double cost;
	  
     } _file_edge_t;
     

//
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//
     class XYTh_GraphNode;

     typedef struct 
     {
	  size_t id;
	  XYTh_GraphNode* nextNode;
          double cost;

     }OutGraphEdge;


     class XYTh_GraphNode
     {
	  XYTH_t state;
	  int    id;


     public :
	 
	  std::vector<OutGraphEdge> outEdges;
	  
	  
	  XYTh_GraphNode()
	  {
	       //adjNodes.clear();
	       //transSplines.clear();
	  };

	  ~XYTh_GraphNode()
	  {
	       outEdges.clear();
	  };
  
	  //

	  void addOutEdge(OutGraphEdge &e)
	  {
	       this->outEdges.push_back(e);
	  };

	  //

	  void setState(XYTH_t& s)
	  {this->state = s;};
	  //
	  void setState(double x, double y, double th)
	  {
	       XYTH_t s = {x,y,th};  this->state = s;  
	  };

	  XYTH_t getState()
	  {
	       return state;
	  };
	  ////////////////
	  void setId(size_t id){this->id = id;} 
	  size_t getId(){return id;};
	  //
	  void printNodeInfo()
	  {
	    std::cout << "Node " << id
		    << " : (" << state.x << ", " << state.y << ", " << state.th << ")\n";
	  }
	  
	  void printSuccessors()
	  {
	       for(unsigned int j = 0; j < outEdges.size(); j++) 
	       {
		 std::cout << "  ========>>>    ";
		    outEdges[j].nextNode->printNodeInfo();
		    
	       }  
	  }
     };
     


//
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//

     class ListGraph
     {
	  
	  std::map<size_t, XYTh_GraphNode*>  nodes_m;
	  
     public:
	  
	  ListGraph()
	  { 
	       this->clear();
	  };


	  ~ListGraph()
	  { 
	    //	       std::cout << __func__ << std::endl; 
	       this->clear();
	  };
	  //
	  void  clear()
	  { 
	       nodes_m.clear();
	  };

	  //
	  inline void addNode(XYTh_GraphNode* node)
	  {
	       size_t id = node->getId();
	       nodes_m[id] = node;
	  };


	  void printNodes()
	  {
	    std::cout << "--------- Graph Nodes---------- :\n";

	       std::map<size_t,XYTh_GraphNode*>::iterator it;
	       
	       for (it = nodes_m.begin(); it!=nodes_m.end(); ++it)
	       {
		    it->second->printNodeInfo();
		    std::cout << "   SUCCESSORS:\n";
		    it->second->printSuccessors();
	       }
	       
	  };

	  //---------
	  XYTh_GraphNode* getNode(int nId)
	  {
	       std::map<size_t, XYTh_GraphNode*>::iterator it;
	       
	       it = nodes_m.find(nId);

	       if(it == nodes_m.end())
		    return NULL;
	       else
		    return it->second;
	  };

	  //--------- 
	  XYTh_GraphNode* getNode(XYTH_t &s)
	  {
	    std::cerr << "HELLO!" << std::endl;
	       std::map<size_t,XYTh_GraphNode*>::iterator it;
	       
	       for (it = nodes_m.begin(); it!=nodes_m.end(); ++it)
	       {
		    XYTH_t s0 = it->second->getState(); 
        if((fabs(s0.x -  s.x) < 1) &&
           (fabs(s0.y -  s.y) < 1)
           //(fabs(NORM_ANG(s0.th - s.th)) < 0.01)
           )
			 return it->second;
	       }
	       
	       return (NULL);
	  };
	  //
	   
     };

///////////////////////////////////////////////////////////////////////////////

     class SearchContext
     {
     
     public:
	  ListGraph graph;
	  XYTH_t    spt, ept;
	  
    
	  bool isEdgeCollisionFree(int arc_id)
	  {return true;};

	  bool findPath(std::vector <int>& arc_ids);
	  void loadGraph(std::string& fileName);
	  int  readGraphEntities(string &fn,
				 std::vector< XYTH_with_id_t> &graphPoses,
				 std::vector<_file_edge_t> &graphEdges);
	  
     };

  
}; // namespace spl_net
#endif
