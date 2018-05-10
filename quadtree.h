// quadtree header
#ifndef __QUADREE_H_INCLUDED__
#define __QUADREE_H_INCLUDED__

#include <vector>

#include <memory>   // std::shared_ptr
#include <utility>  // std::unique_ptr

struct BoundaryBox
{
	float cx;	// center of the node (x-coordinate)
	float cy;	// center of the node (y-coordinate)
	float dim;	// width of the node

	// constructor
	BoundaryBox(float _cx, float _cy, float _dim)
	{
		cx  = _cx;
		cy  = _cy;
		dim = _dim;
	}
};

class Quadtree
{
	private:
		// Children nodes
		Quadtree *northWest;
		Quadtree *northEast;
		Quadtree *southWest;
		Quadtree *southEast;

		// dimensions of the node
		std::shared_ptr<BoundaryBox> boundary2;

		// pointer to the std::vectors containing all the points
		std::vector<float> *ptrToX;
		std::vector<float> *ptrToY;

		// starting point in the vectors above (as well as the amount of points the 'shape' consists of)
		// TODO: merge element_start and element_amount into a single std::vector<std::pair<int, int> > vec. Especially useful in concatenate_nodes -> removes duplicates from 
		std::vector<int> element_start;
		std::vector<int> element_amount;

		// shared space (elements which do not fit into a single node completely)
		std::vector<int> shared_element_start;
		std::vector<int> shared_element_amount;

		// minimum amount of pts to split the node
		unsigned int maxAmtElements = 2;

		// maximum depth of the children nodes
		int maxDepth = 5;

		// depth of the node (0...root node)
		int nodeDepth;

		// pointer to the parent node
		Quadtree* parent;

		// delete the children (leaf)nodes (NW, NE, SW, SE) of a specific node.
		void clearNode();

		// clear the tree
		void clear(Quadtree *t);

		// recursively insert a element into all leaf nodes (deepest nodes possible) of a given node *t
		void recursive_entry(Quadtree *t, int iStart, int iAmount);

		// recursively remove a element from the shared space of all leafnodes containing a given node *t
		void recursive_remove(Quadtree *t, int iStart, int iAmount);

		// drawing routine (used by traverse_and_draw)
		void colorPick(float elevate, Quadtree* t, float *depthColor, int depthColorLen);

		// Used to collapse nodes and redistribute elements after collapsing.
		void concatenate_nodes(Quadtree *concat_this_node_maybe);

		// fetch the (deepest) node in which the given element resides
		Quadtree* fetch_deepest_node(int iStart, int iAmount);

		// auxiliary function used by fetch_deepest_node().
		Quadtree* fetch_deepest_node_internal(Quadtree* t, int iStart, int iAmount);

		// auxiliary function used by fetch_elements().
		void fetch_elements_internal(std::set< std::pair<int,int> > &vec, Quadtree *t, int iStart, int iAmount);

		void fetch_elements_internal2(std::set< std::pair<int,int> > &vec, Quadtree *t, float xmin, float xmax, float ymin, float ymax);

 		void recursive_removeAABB(Quadtree *t, float xmin, float xmax, float ymin, float ymax, int iStart, int iAmount);





	public:
		// constructor
		Quadtree(std::shared_ptr<BoundaryBox> BB_init, Quadtree *parent, int _nodeDepth, std::vector<float> *iVecX, std::vector<float> *iVecY);

		// destructor
		~Quadtree();

		// insert a point into the tree
		bool insert(int iStart, int iAmount);

		// split the current node into four new (children)nodes (increment depth by one)
		bool subdivide();

		// draw the tree using OpenGL
		void traverse_and_draw(Quadtree* t, float widthRootNode);

		// count the nodes of the tree
		int count_nodes(Quadtree *t);

		// count the elements residing in the tree
		int count_elements(Quadtree *t);

		// returns all possible colliding elements corresponding to the node in which this element (iStart, iAmount) resides
		std::set< std::pair<int,int> > fetch_elements(int iStart, int iAmount);



		std::set< std::pair<int,int> > fetch_elements_AABB_external(float xmin, float xmax, float ymin, float ymax);



		// remove a single element of the tree
		bool delete_element(int iStart, int iAmount);

		void test(float xmin, float xmax, float ymin, float ymax);

		void test2(Quadtree* t, float xmin, float xmax, float ymin, float ymax, int iStart, int iAmount);

		// relocate a single element of the tree
// 		bool relocate_element(int iStart1, int iAmount1, int iStart2, int iAmount2);

		// debuggingfunctions
		// visualizes the nodes, which can be concatenated (colored) and the nodes which only inherits elements in the shared space (shared_element_start, shared_element_amount).  The latter are colored grey.
		void find_concatenable_shared_nodes(Quadtree *t);

		// prints the tree (amount of elements in the vectors and the pointers to the nodes)
		void print_tree();
};
#endif
