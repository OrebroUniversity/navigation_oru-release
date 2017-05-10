
#include <iostream>

#include "orunav_path_pool/orunav_path_pool.h"


using namespace astar_graph_plan;

//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤
//¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤¤

void buildTestGraph(ListGraph& graph)
{

     // add 5 nodes

     // node 1
     XYTh_GraphNode *n1 = new XYTh_GraphNode();
     n1->setId(1);
     n1->setState (10.0, 10.0, 0.0);
     // node 2
     XYTh_GraphNode *n2 = new XYTh_GraphNode();
     n2->setId(2);
     n2->setState(12.0, 10.0, 45.0 );
     // node 3
     XYTh_GraphNode *n3 = new XYTh_GraphNode();
     n3->setId(3);
     n3->setState(12.0, 20.0, 45.0 );
     // node 4
     XYTh_GraphNode *n4 = new XYTh_GraphNode();
     n4->setId(4);
     n4->setState(20.0, 10.0, 45.0 );
     // node 5
     XYTh_GraphNode *n5 = new XYTh_GraphNode();
     n5->setId(5);
     n5->setState(20.0, 20.0, 0.0 );

     // the edges
     size_t e_id = 1;
     OutGraphEdge e;

     // succ of n1; n2, n3
     e.id = e_id++;
     e.nextNode = n2;
     e.cost = 10.0;
     n1->addOutEdge(e);
	  
     e.id = e_id++;
     e.nextNode = n3;
     e.cost = 30.0;
     n1->addOutEdge(e);
          
     // succ of n2: n3,n4,n5
     e.id = e_id++;
     e.nextNode = n3;
     e.cost = 10.0;
     n2->addOutEdge(e);
     e.id = e_id++;
     e.nextNode = n4;
     e.cost = 20.0;
     n2->addOutEdge(e);
     e.id = e_id++;
     e.nextNode = n5;
     e.cost = 60.0;
     n2->addOutEdge(e);

     // succ of n3: n2, n5
     e.id = e_id++;
     e.nextNode = n2;
     e.cost = 10.0;
     n3->addOutEdge(e);
     e.id = e_id++;
     e.nextNode = n5;
     e.cost = 60.0;
     n3->addOutEdge(e);

     // succ of n4: n5
     e.id = e_id++;
     e.nextNode = n5;
     e.cost = 20.0;
     n4->addOutEdge(e);

     graph.addNode(n1);
     graph.addNode(n2);
     graph.addNode(n3);
     graph.addNode(n4);
     graph.addNode(n5);

};


////////////////////////////////////////////////////////////////////////
int main()
{

     SearchContext ctxt;
     ctxt.spt = {10.0, 10.0, 0.0}; 
     ctxt.ept = {20.0, 20.0, 0.0};

     std::cout << "Test of A* in a network of nodes " << std::endl;
          
     buildTestGraph(ctxt.graph);
     ctxt.graph.printNodes();

     // testing A*
     vector <int> edge_ids;
     if(ctxt.findPath(edge_ids))
     {
	  cout << "here is the A* solution:\n";
	  for(unsigned int i = 0; i < edge_ids.size(); i++)
	  {
	       cout << "Step: " << i << ": Edge id  " <<edge_ids[i] << endl;
	  }

     }

     // testing reading graph file

     string fn("../src/gph-edges.txt");
     ctxt.graph.clear();
     ctxt.loadGraph(fn);
     ctxt.graph.printNodes();
     edge_ids.clear();
     // CALLING THE PLANNER TO FIND A PLAN
     if(ctxt.findPath(edge_ids))
     {
	  cout << "here is the A* solution:\n";
	  for(unsigned int i = 0; i < edge_ids.size(); i++)
	  {
	       cout << "Step: " << i << ": Edge id  " <<edge_ids[i] << endl;
	  }

     }
};
