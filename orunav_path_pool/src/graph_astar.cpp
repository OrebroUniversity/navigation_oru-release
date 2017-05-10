/////////////////////////////////////////////////////////////////////////////////////
// 
// Abdelbaki Bouguerra

#include <fstream>
#include <sstream>


#include "orunav_path_pool/graph_astar.h"
#include <angles/angles.h>

namespace astar_graph_plan
{
     

     /////////////////////////////////////////////////////////////////
     //
     class XYThSearchNode
     {

	  SearchContext *ctxt;
	  
     public:

	  XYTh_GraphNode*  myGraphNode; 
	  double arrival_cost;
	  size_t arrival_edge_id;
	  
	  //---
	  XYThSearchNode() 
	       {
		    myGraphNode = NULL;
	       };
	  //---
	  XYThSearchNode(XYTh_GraphNode*  graphNode, SearchContext* ctxt) 
	       { 
		    this->ctxt = ctxt;
		    myGraphNode = graphNode;
	       }
          //---

	  double GoalDistanceEstimate( XYThSearchNode &nodeGoal );
	  bool   IsGoal( XYThSearchNode &nodeGoal );

	  bool GetSuccessors( AStarSearch<XYThSearchNode> *astarsearch, 
			      XYThSearchNode *parent_node );

	  double GetCost( XYThSearchNode &successor );
	  bool   IsSameState( XYThSearchNode &rhs );
  
	  void PrintNodeInfo(); 
	  void setContext(SearchContext *c)
	       {ctxt = c;};
     };



//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤

     bool XYThSearchNode::IsSameState( XYThSearchNode &rhs )
     {
	  // same state  is simply when (x,y, th) are the same
	  XYTH_t s1 =  this->myGraphNode->getState();
	  XYTH_t s0 =  rhs.myGraphNode->getState();;
	  if( (fabs(s1.x   - s0.x) <= 0.0001) && 
	      (fabs(s1.y   - s0.y) <= 0.0001) && 
	      (fabs(angles::normalize_angle(s1.th  - s0.th)) <= 0.0001) )
	       return true;
	  else
	       return false;
     }

/////////////////////////////////////////////////////////////////////////

     void XYThSearchNode::PrintNodeInfo()
     {
	  this->myGraphNode->printNodeInfo();
     }

///////////////////////////////////////////////////////////////////////////
// Here's the heuristic function that estimates the distance from a Node
// to the Goal. 

     double XYThSearchNode::GoalDistanceEstimate( XYThSearchNode &gNode )
     {
	  XYTH_t s0 =  this->myGraphNode->getState();
	  XYTH_t s1 =  gNode.myGraphNode->getState();;
	
	  double dx = (s1.x - s0.x); 
	  double dy = (s1.y - s0.y);
	  //double dth = (double) th;
	  return sqrt(dx*dx + dy*dy);
     }

/////////////////////////////////////////////////////////////////////////////

     bool XYThSearchNode::IsGoal( XYThSearchNode &nodeGoal )
     {
 
	  if( IsSameState( nodeGoal ))
	  {
	       nodeGoal.arrival_cost     = this->arrival_cost;
	       nodeGoal.arrival_edge_id  = this->arrival_edge_id;
	       return true;
	  }
	  else
	       return false;
     }


/////////////////////////////////////////////////////////////////////////////////
// This generates the successors to the given Node. 
// It uses a helper function called
// AddSuccessor to give the successors to the AStar class. 
// The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application

     bool XYThSearchNode::
     GetSuccessors(AStarSearch<XYThSearchNode> *astarsearch, 
		   XYThSearchNode *parent_node )
     {
	  // push each possible move except allowing the search to go backwards

	  	  
	  if(myGraphNode != NULL)
	  {
	       unsigned int nSucc = myGraphNode->outEdges.size();
	       for(unsigned int i = 0; i < nSucc; i++)
	       {
		    OutGraphEdge e =  myGraphNode->outEdges[i];
		    XYThSearchNode newNode (e.nextNode, this->ctxt) ;

		    newNode.arrival_cost = e.cost;
		    newNode.arrival_edge_id = e.id;

		     if(parent_node && newNode.IsSameState(*parent_node))
			  continue;
		     else
		     {
			  // the path leading to new node should be collision free <<<<	
			  if( ctxt->isEdgeCollisionFree(e.id))
			  {
			       astarsearch->AddSuccessor(newNode);
			  }
		     }

		}
		return true;     
	   }
	   else
		return false;
      };

 /////////////////////////////////////////////////////////////////////////////////
 // given this node, what does it cost to move to successor. In the case
 // of our map the answer is the map terrain value at this node since that is 
 // conceptually where we're moving

     double XYThSearchNode::GetCost( XYThSearchNode &successor )
     {
	  return (successor.arrival_cost);
     }

//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤

// Main planning fn

     bool SearchContext::findPath(vector <int>& sol_edge_ids)
     {
	 	  
       //	  spt.th = spt.th;
       //	  ept.th = ept.th;


   
	  // Create a start-state search node
	  XYTh_GraphNode *gn_s = graph.getNode(spt);
	  if(gn_s == NULL)
	  {
	       cout << "A*: Initial State does not belong to the search graph\n";
	       return false;
	  }
	  // Define the goal-state search node
	  XYTh_GraphNode *gn_e = graph.getNode(ept);
	  if(gn_e == NULL)
	  {
	       cout << "A*: Goal State does not belong to the search graph\n";
	       return false;
	  }
	  XYThSearchNode nodeStart(gn_s, this);
	  XYThSearchNode nodeEnd  (gn_e, this);
	  
	  nodeStart.arrival_cost = 0.0;
	  nodeStart.arrival_edge_id = 0;

	  cout << "A-Star Search init state\n";
	  nodeStart.PrintNodeInfo();
	  cout << "A-Star Search goal state\n";
	  nodeEnd.PrintNodeInfo();
      
	  // Set Start and goal states
	  AStarSearch<XYThSearchNode> astarsearch;
	  astarsearch.SetStartAndGoalStates(nodeStart, nodeEnd );

	  unsigned int SearchState;
	  unsigned int SearchSteps = 0;

	  bool result = false;

	  do
	  {
	       SearchState = astarsearch.SearchStep();
	       SearchSteps++;
	  }
	  while( SearchState == AStarSearch<XYThSearchNode>::SEARCH_STATE_SEARCHING );
	  //

	  if( SearchState == AStarSearch<XYThSearchNode>::SEARCH_STATE_SUCCEEDED )
	  {
	       cout << "A-Star Search found goal state\n";
	       
	       XYThSearchNode *node = astarsearch.GetSolutionStart();
       	       int steps = 0;
       	       node->PrintNodeInfo();
	       sol_edge_ids.clear();
	       for( ;; )
	       {
		    node = astarsearch.GetSolutionNext();
	   
		    if( !node )
		    {
			 break;
		    }
		    sol_edge_ids.push_back(node->arrival_edge_id);
		    node->PrintNodeInfo();
		    steps ++;
	       };

	       cout << "Solution steps " << steps << endl;
       
	       // Once we are done with the solution, we free the nodes up
	       astarsearch.FreeSolutionNodes();
	       result = true;
       	  }
	  else if( SearchState == AStarSearch<XYThSearchNode>::SEARCH_STATE_FAILED ) 
	  {
	       cout << "A-Star Search terminated. Did not find goal state\n";
	  }
	  
	  astarsearch.EnsureMemoryFreed();
   
   	  return result;
     };


/////////////////////////////////////////////////////////////////////////////
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤

     void SearchContext::loadGraph(string& fileName)
     {
	  
	  size_t nodeId = 1;
	  
	  vector< XYTH_with_id_t> gphPoses;
	  vector<_file_edge_t> gphEdges;
    	  
	  readGraphEntities(fileName, gphPoses, gphEdges);
   

	  for(unsigned int i = 0; i < gphPoses.size(); i++)
	  {
	       XYTh_GraphNode* gn = graph.getNode(gphPoses[i].id);
 
	       if(gn == NULL)
	       {
		    gn = new XYTh_GraphNode();
		    gn->setState (gphPoses[i].state);
		    gn->setId(gphPoses[i].id);
		    graph.addNode(gn);
	       }
	       else
	       {
		    cout << __func__ ;
		    cout << ": WARNING: Pose already in the graph. Pose not added\n";
	       }

	  }

	  //
	  for(unsigned int i = 0; i <  gphEdges.size(); i++)
	  {
	       _file_edge_t edge =  gphEdges[i];
	       	       
	       XYTh_GraphNode* gn0 = graph.getNode(edge.sPt_id);
	       XYTh_GraphNode* gn1 = graph.getNode(edge.ePt_id);
 
	       // add the edge info
	       OutGraphEdge e;
	       e.id = edge.id;
	       e.nextNode = gn1;
	       e.cost = edge.cost;
	       gn0->addOutEdge(e);
       	  };
     };

     //------------------------------------------------------------------
     void eliminateLeadingWhiteSpace(std::string &str)
     {
	  unsigned found = str.find(" ");
	  while (found == 0)
	  {
	       str = str.substr(found+1);
	       found = str.find(" ");
	  };
     }
     
     /////////////////////////////////////////////////////////////////////

     void extractPoses( ifstream & ifs, vector< XYTH_with_id_t> &graphPoses)
     {
	  cout << __func__ << endl;
	  size_t id; 
	  XYTH_t  pt;

	  while (ifs.good() )
	  {
	       std::string line;
	       std::getline(ifs,line);

	       // cout << " Read line: \n";
	       // cout << line << endl;
 
	       eliminateLeadingWhiteSpace(line);
	       if(line.empty())
		    continue;

	       if(line.find("#") == 0) // this line is a comment
		    continue;
	       
	       if(line.find("EndPoses") == 0) // this line is a comment
		    break;

	       std::stringstream ss;
	       ss << line;
	       ss >> id;
	       ss >> pt.x;
	       ss >> pt.y;
	       ss >> pt.th;
	       // cout << "  Extracted point : "
	       // 	    << id << "  (" << pt.x << ", " << pt.y << ", " << pt.th << ")\n";

	       XYTH_with_id_t pose;
	       pose.id = id;
	       pose.state = pt;
	       graphPoses.push_back(pose);
	  }
     };

     /////////////////////////////////////////////////////////////////////
     void extractEdges( ifstream & ifs, vector<_file_edge_t> &graphEdges)
     {
	  cout << __func__ << endl;
	  size_t e_id, pt0_id, pt1_id; 
	  double e_cost;

	  while (ifs.good() )
	  {
	       std::string line;
	       std::getline(ifs,line);

	       // cout << " Read line: \n";
	       // cout << line << endl;
 
	       eliminateLeadingWhiteSpace(line);
	       if(line.empty())
		    continue;

	       if(line.find("#") == 0) // this line is a comment
		    continue;
	       
	       if(line.find("EndEdges") == 0) // this line is a comment
		    break;
	       
	       std::stringstream ss;
	       ss << line;
	       ss >> e_id;
	       ss >> pt0_id;
	       ss >> pt1_id;
	       ss >> e_cost;

	       // cout <<   "  Extracted Edge : "
	       //      << e_id 
	       //      << " (" << pt0_id << " ====>>> " << pt1_id << "):" << e_cost << "\n";

	       _file_edge_t e;
	       e.id = e_id;
	       e.sPt_id = pt0_id;
	       e.ePt_id = pt1_id;
	       e.cost = e_cost;
	       graphEdges.push_back(e);
	  }
     }
     
//
//////////////////////////////////////////////////////////////////////////////////
//
     int SearchContext::readGraphEntities(string &fn,
					  vector< XYTH_with_id_t> &gphPoses,
 					  vector<_file_edge_t> &gphEdges)
     {

	  ifstream ifs(fn.c_str(), ifstream::in);

	  if (ifs.fail())
	  {
	       cout << "Failed to open splines' file " << fn << endl;
	       cout << "exiting....... \n";
	       exit(1);
	  }

	  //
	  while (ifs.good() )
	  {
	       std::string line;
	       std::getline(ifs,line);

	       eliminateLeadingWhiteSpace(line);

	       if(line.find("#") == 0) // this line is a comment
		    continue;

	       //---
	       if(line.find("BeginPoses") == 0) // this line is a comment
	       {
		    extractPoses(ifs, gphPoses);
	       };
	       //----
	       if(line.find("BeginEdges") == 0) // this line is a comment
	       {
		    extractEdges(ifs, gphEdges);
	       };
	  }

	  ifs.close();
   	  return 0;
     };



} //  namespace spl_net


//
///////////////////////////////////////////////////////////////
//
//
