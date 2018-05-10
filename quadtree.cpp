// quadtree class & functions
#include <math.h>
#include <iostream>
#include <vector>
#include <algorithm>	// std::min, std::max
#include <utility>		// std::unique_ptr
#include <set>

#include <GL/glut.h>
#include <GL/gl.h>

#include "quadtree.h"


// constructor
Quadtree::Quadtree(std::shared_ptr<BoundaryBox> BB_init, Quadtree *parent, int _nodeDepth, std::vector<float> *iVecX = nullptr, std::vector<float> *iVecY = nullptr)
{
	// Provide pointer to the std::vector containing all the points.
	if ((iVecX != nullptr) and (iVecY != nullptr))
	{
		ptrToX = iVecX;
		ptrToY = iVecY;
	}
	else
	{
		ptrToX = parent->ptrToX;
		ptrToY = parent->ptrToY;
	}

	northWest = nullptr;
	northEast = nullptr;
	southWest = nullptr;
	southEast = nullptr;

	this->boundary2 = std::move(BB_init); 

	if (parent == nullptr)
	{
		this->parent = this;
	}
	else
	{
		this->parent = parent;
	}

	this->nodeDepth = _nodeDepth;
}

// clear the tree
void Quadtree::clear(Quadtree* t)
{
    if(t != nullptr)
	{
		clear(t->northEast);
		clear(t->northWest);
		clear(t->southEast);
		clear(t->southWest);

		delete t;	// calls the destructor
	}
}

// destructor
Quadtree::~Quadtree()
{
    delete northWest;
    delete northEast;
    delete southWest;
    delete southEast;

    northWest = nullptr;
    northEast = nullptr;
    southWest = nullptr;
    southEast = nullptr;
}

// delete the children (leaf)nodes (NW, NE, SW, SE) of a specific node.
void Quadtree::clearNode()
{
    delete northWest;
    delete northEast;
    delete southWest;
    delete southEast;

    northWest = nullptr;
    northEast = nullptr;
    southWest = nullptr;
    southEast = nullptr;
}


// recursively insert a element into all leaf nodes (deepest nodes possible) of a given node (*t)
void Quadtree::recursive_entry(Quadtree *t, int iStart, int iAmount)
{
	if (t->northEast == nullptr)
	{
		t->shared_element_start.push_back(iStart);
		t->shared_element_amount.push_back(iAmount);
	}
	else
	{
		recursive_entry(t->northEast, iStart, iAmount);
		recursive_entry(t->northWest, iStart, iAmount);
		recursive_entry(t->southEast, iStart, iAmount);
		recursive_entry(t->southWest, iStart, iAmount);
	}
}

// recursively remove a element from the shared space of all leafnodes containing a given node *t
void Quadtree::recursive_remove(Quadtree *t, int iStart, int iAmount)
{
	if (t->northWest == nullptr)
	{
		int i = -1;
		bool found_i = false;

		for (i = 0; i < (int)t->shared_element_start.size(); i++)
		{
			if ((t->shared_element_start[i] == iStart) and (t->shared_element_amount[i] == iAmount))
			{
				found_i = true;
				break;
			}
		}

		if (found_i == true)
		{
			t->shared_element_start.erase(t->shared_element_start.begin()+i);
			t->shared_element_amount.erase(t->shared_element_amount.begin()+i);
		}
		else
		{
			std::cout << " rec rem - element not found" << std::endl;
			exit(1);
		}
	}
	else
	{
		recursive_remove(t->northEast, iStart, iAmount);
		recursive_remove(t->northWest, iStart, iAmount);
		recursive_remove(t->southEast, iStart, iAmount);
		recursive_remove(t->southWest, iStart, iAmount);
	}
}







// recursively remove a element from the shared space of all leafnodes containing a given node *t
void Quadtree::recursive_removeAABB(Quadtree *t, float xmin, float xmax, float ymin, float ymax, int iStart, int iAmount)
{
	// collision if:
	if ((xmax > t->boundary2->cx-t->boundary2->dim) and (xmin < t->boundary2->cx+t->boundary2->dim) and (ymin < t->boundary2->cy+t->boundary2->dim) and (ymax > t->boundary2->cy-t->boundary2->dim))
	{
		if (t->northWest != nullptr)	// this node has been split yet -> != nullptr
		{
			recursive_removeAABB(t->northWest, xmin, xmax, ymin, ymax, iStart, iAmount);
			recursive_removeAABB(t->northEast, xmin, xmax, ymin, ymax, iStart, iAmount);
			recursive_removeAABB(t->southWest, xmin, xmax, ymin, ymax, iStart, iAmount);
			recursive_removeAABB(t->southEast, xmin, xmax, ymin, ymax, iStart, iAmount);
		}
		else	// deepest node possible
		{
			// delete the element from the shared space
			int i = -1;
			bool found_i = false;

			for (i = 0; i < (int)t->shared_element_start.size(); i++)
			{
				if ((t->shared_element_start[i] == iStart) and (t->shared_element_amount[i] == iAmount))
				{
					found_i = true;
					break;
				}
			}

			if (found_i == true)
			{
				t->shared_element_start.erase(t->shared_element_start.begin()+i);
				t->shared_element_amount.erase(t->shared_element_amount.begin()+i);
			}
			else
			{
				std::cout << " rec rem - element not found" << std::endl;
				exit(1);
			}
		}
	}
	else
	{
//  		std::cout << "NO COLL" << std::endl;
	}
}














// drawing & colorpicking routine (used by traverse_and_draw). Used by traverse_and_draw()
void Quadtree::colorPick(float elevate, Quadtree *t, float *depthColor, int depthColorLen)
{
	if(t->boundary2)
	{
		if (t->nodeDepth*3+2 > depthColorLen)	// default color when the depth exceeds the available colors from the array
		{
			glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
		}
		else	// pick a color according to the array
		{
			glColor4f(depthColor[t->nodeDepth*3], depthColor[t->nodeDepth*3+1], depthColor[t->nodeDepth*3+2], 1.0f);
		}

		float centerx = t->boundary2->cx;
		float centery = t->boundary2->cy;
		float dim = t->boundary2->dim;

		glBegin(GL_LINES);
			glVertex3f(centerx-dim, centery, elevate);
			glVertex3f(centerx+dim, centery, elevate);

			glVertex3f(centerx, centery-dim, elevate);
			glVertex3f(centerx, centery+dim, elevate);
		glEnd();

	}
}



// fetch the (deepest) node in which the given element resides
Quadtree *Quadtree::fetch_deepest_node(int iStart, int iAmount)
{
	Quadtree *ReturnNode = this;

	ReturnNode = fetch_deepest_node_internal(ReturnNode, iStart, iAmount);

	return ReturnNode;
}

// auxiliary function used by fetch_deepest_node().
Quadtree *Quadtree::fetch_deepest_node_internal(Quadtree *t, int iStart, int iAmount)
{
	int count_inside = iAmount;

	// prevent a "Conditional jump or move depends on uninitialised value(s)" detected by valgrind
	if ((t->parent == t) && (t->northEast == nullptr) && (t->northWest == nullptr) && (t->southEast == nullptr) && (t->southWest == nullptr))
	{
		return t;
	}
	else
	{
		// check if the (end)points of the element reside completely in this node
		for (int i = iStart; i < (iStart+iAmount); i++)
		{
			if ((*ptrToX)[i] > boundary2->cx+boundary2->dim or (*ptrToX)[i] <= boundary2->cx-boundary2->dim or (*ptrToY)[i] > boundary2->cy+boundary2->dim or (*ptrToY)[i] <= boundary2->cy-boundary2->dim)
			{
				count_inside--;
			}
		}

		// object resides completely inside the given node
		if (count_inside == iAmount)
		{
			t = this;

			// deepest node corresponding to this point reached. Return the pointer to this node
			if (northWest == nullptr)
			{
				return t;
			}
			else
			{
				t = northEast->fetch_deepest_node_internal(t, iStart, iAmount);
				t = northWest->fetch_deepest_node_internal(t, iStart, iAmount);
				t = southWest->fetch_deepest_node_internal(t, iStart, iAmount);
				t = southEast->fetch_deepest_node_internal(t, iStart, iAmount);
			}
		}
	}

	return t;
}

// OLD ONE
// auxiliary function used by fetch_elements().
void Quadtree::fetch_elements_internal(std::set< std::pair<int, int> > &vec, Quadtree *t, int iStart, int iAmount)
{
	if ((int)t->element_start.size() != (int)t->element_amount.size())
	{
		std::cout << "internal build err1" << std::endl;
		exit(1);
	}

	if ((int)t->shared_element_start.size() != (int)t->shared_element_start.size())
	{
		std::cout << "internal build err2" << std::endl;
		exit(1);
	}

	for (int i = 0; i < (int)t->element_start.size(); i++)
	{
		vec.insert(std::make_pair(t->element_start[i], t->element_amount[i]));
	}

	for (int i = 0; i < (int)t->shared_element_start.size(); i++)
	{
		vec.insert(std::make_pair(t->shared_element_start[i], t->shared_element_amount[i]));
	}

	if (t->northEast != nullptr)
	{
		fetch_elements_internal(vec, t->northEast, iStart, iAmount);
	}

	if (t->northWest != nullptr)
	{
		fetch_elements_internal(vec, t->northWest,  iStart, iAmount);
	}

	if (t->southEast != nullptr)
	{
		fetch_elements_internal(vec, t->southEast,  iStart, iAmount);
	}

	if (t->southWest != nullptr)
	{
		fetch_elements_internal(vec, t->southWest,  iStart, iAmount);
	}
}


// auxiliary function used by fetch_elements().
void Quadtree::fetch_elements_internal2(std::set< std::pair<int, int> > &vec, Quadtree *t, float xmin, float xmax, float ymin, float ymax)
{
// void Quadtree::test2(Quadtree* t, float xmin, float xmax, float ymin, float ymax, int iStart, int iAmount)
// {
	// collision if:
	if ((xmax > t->boundary2->cx-t->boundary2->dim) and (xmin < t->boundary2->cx+t->boundary2->dim) and (ymin < t->boundary2->cy+t->boundary2->dim) and (ymax > t->boundary2->cy-t->boundary2->dim))
	{
		if (t->northWest != nullptr)	// this node has been split yet -> != nullptr
		{
			fetch_elements_internal2(vec, t->northEast, xmin, xmax, ymin, ymax);
			fetch_elements_internal2(vec, t->northWest, xmin, xmax, ymin, ymax);
			fetch_elements_internal2(vec, t->southEast, xmin, xmax, ymin, ymax);
			fetch_elements_internal2(vec, t->southWest, xmin, xmax, ymin, ymax);
		}
		else	// deepest node possible
		{
// 			t->shared_element_start.push_back(iStart);
// 			t->shared_element_amount.push_back(iAmount);
			// push elements into the vec
			for (int i = 0; i < (int)t->element_start.size(); i++)
			{
				vec.insert(std::make_pair(t->element_start[i], t->element_amount[i]));
			}

			for (int i = 0; i < (int)t->shared_element_start.size(); i++)
			{
				vec.insert(std::make_pair(t->shared_element_start[i], t->shared_element_amount[i]));
			}
		}
// 		std::cout << "COLLISION" << std::endl;
	}
	else
	{
//  		std::cout << "NO COLL" << std::endl;
	}
// }
}


std::set< std::pair<int,int> > Quadtree::fetch_elements_AABB_external(float xmin, float xmax, float ymin, float ymax)
{
	std::set< std::pair<int, int> > vec;
	fetch_elements_internal2(vec, this, xmin, xmax, ymin, ymax);
	return vec;
}

// returns all possible colliding elements corresponding to the node in which this element (iStart, iAmount) resides
std::set< std::pair<int,int> > Quadtree::fetch_elements(int iStart, int iAmount)
{
	// find the deepest node possible in which this shape fits completely
// 	Quadtree *fetch_node = fetch_deepest_node(iStart, iAmount);

	// create the vec for returning the elements
	std::set< std::pair<int, int> > vec;


	// FASTER THAN WITH FETCH_NODE?
		// generate the AABB boundary box
		float xmin = 100.0;
		float xmax = -100.0;
		float ymin = 100.0;
		float ymax = -100.0;

		for (int i = iStart; i < (iStart+iAmount); i++)
		{
			if ((*ptrToX)[i] > xmax)
				xmax = (*ptrToX)[i];

			if ((*ptrToX)[i] < xmin)
				xmin = (*ptrToX)[i];

			if ((*ptrToY)[i] > ymax)
				ymax = (*ptrToY)[i];

			if ((*ptrToY)[i] < ymin)
				ymin = (*ptrToY)[i];
		}

		fetch_elements_internal2(vec, this, xmin, xmax, ymin, ymax);
	// FASTER THAN WITH FETCH_NODE?

/*
	// deepest node possible -> retrieve only elements from this node
	if (fetch_node->northWest == nullptr)
	{
 		std::cout << "deepest node possible" << std::endl;

		for (int i = 0; i < (int)fetch_node->element_start.size(); i++)
		{
			vec.insert(std::make_pair(fetch_node->element_start[i], fetch_node->element_amount[i]));
		}

		for (int i = 0; i < (int)fetch_node->shared_element_start.size(); i++)
		{
			vec.insert(std::make_pair(fetch_node->shared_element_start[i], fetch_node->shared_element_amount[i]));
		}
	}
	else
	{
		// generate the AABB boundary box
		float xmin = 100.0;
		float xmax = -100.0;
		float ymin = 100.0;
		float ymax = -100.0;

		for (int i = iStart; i < (iStart+iAmount); i++)
		{
			if ((*ptrToX)[i] > xmax)
				xmax = (*ptrToX)[i];

			if ((*ptrToX)[i] < xmin)
				xmin = (*ptrToX)[i];

			if ((*ptrToY)[i] > ymax)
				ymax = (*ptrToY)[i];

			if ((*ptrToY)[i] < ymin)
				ymin = (*ptrToY)[i];
		}

		fetch_elements_internal2(vec, fetch_node, xmin, xmax, ymin, ymax);
	}

// 	fetch_elements_internal(vec, fetch_node, iStart, iAmount);
*/

	return vec;
}


void Quadtree::test(float xmin, float xmax, float ymin, float ymax)
{
//  	std::cout << " test || xmin: " << xmin << " / xmax: " << xmax << " /// ymin: " << ymin << " / ymax: " << ymax << std::endl;

	// collision if:
	if ((xmax > boundary2->cx-boundary2->dim) and (xmin < boundary2->cx+boundary2->dim) and (ymin < boundary2->cy+boundary2->dim) and (ymax > boundary2->cy-boundary2->dim))
	{
		if (northWest != nullptr)	// this node has been split yet -> != nullptr
		{
			northWest->test(xmin, xmax, ymin, ymax);
			northEast->test(xmin, xmax, ymin, ymax);
			southWest->test(xmin, xmax, ymin, ymax);
			southEast->test(xmin, xmax, ymin, ymax);
		}
		else	// deepest node possible
		{
			float centerx = boundary2->cx;
			float centery = boundary2->cy;
			float dim = boundary2->dim;

			float elevate = -10.0;

			glColor4f(0.0f, 0.0f, 0.0f, 0.25f);

			glBegin(GL_TRIANGLE_STRIP);
				glVertex3f(centerx+dim, centery+dim, elevate);
				glVertex3f(centerx+dim, centery-dim, elevate);
				glVertex3f(centerx-dim, centery+dim, elevate);
				glVertex3f(centerx-dim, centery-dim, elevate);
			glEnd();
		}
// 		std::cout << "COLLISION" << std::endl;
	}
	else
	{
//  		std::cout << "NO COLL" << std::endl;
	}
}


void Quadtree::test2(Quadtree* t, float xmin, float xmax, float ymin, float ymax, int iStart, int iAmount)
{
	// collision if:
	if ((xmax > t->boundary2->cx-t->boundary2->dim) and (xmin < t->boundary2->cx+t->boundary2->dim) and (ymin < t->boundary2->cy+t->boundary2->dim) and (ymax > t->boundary2->cy-t->boundary2->dim))
	{
		if (t->northWest != nullptr)	// this node has been split yet -> != nullptr
		{
			test2(t->northEast, xmin, xmax, ymin, ymax, iStart, iAmount);
			test2(t->northWest, xmin, xmax, ymin, ymax, iStart, iAmount);
			test2(t->southEast, xmin, xmax, ymin, ymax, iStart, iAmount);
			test2(t->southWest, xmin, xmax, ymin, ymax, iStart, iAmount);
		}
		else	// deepest node possible
		{
			t->shared_element_start.push_back(iStart);
			t->shared_element_amount.push_back(iAmount);
		}
// 		std::cout << "COLLISION" << std::endl;
	}
	else
	{
//  		std::cout << "NO COLL" << std::endl;
	}
}



// insert one point into the tree. Split the tree and relocate the points ot the node if necessary
bool Quadtree::insert(int iStart, int iAmount)
{
	int count_inside = iAmount;

	// TODO: catch cases, where all points of the polygon lies outside of the node but part of the area of the polygon still is inside the QT)
	// check if all the element can be fit completely into the node
 	for (int i = iStart; i < (iStart+iAmount); i++)
	{
 		if ((*ptrToX)[i] > boundary2->cx+boundary2->dim or (*ptrToX)[i] <= boundary2->cx-boundary2->dim or (*ptrToY)[i] > boundary2->cy+boundary2->dim or (*ptrToY)[i] <= boundary2->cy-boundary2->dim)
		{
			count_inside--;
		}
	}

	// object lies completely outside of the root node
	if ((count_inside == 0) and (this == this->parent))
	{
		std::cout << "\033[1;31m" << "Object completely outside of rootnode!!!" << "\033[0m\n";

		return false;
	}

	if (count_inside < iAmount)
	{
		if (count_inside > 0)
		{
			// generate the AABB boundary box
			float xmin = 100.0;
			float xmax = -100.0;
			float ymin = 100.0;
			float ymax = -100.0;

			for (int i = iStart; i < (iStart+iAmount); i++)
			{
				if ((*ptrToX)[i] > xmax)
					xmax = (*ptrToX)[i];

				if ((*ptrToX)[i] < xmin)
					xmin = (*ptrToX)[i];

				if ((*ptrToY)[i] > ymax)
					ymax = (*ptrToY)[i];

				if ((*ptrToY)[i] < ymin)
					ymin = (*ptrToY)[i];
			}


			// insert recursively
			if (this == this->parent)
			{
// 				recursive_entry(this, iStart, iAmount);
				test2(this, xmin, xmax, ymin, ymax, iStart, iAmount);

			}
			else	// go one up, because the insert would only insert into the deepest node it searches (e.g. southEast but the element does NOT fit into southEast completely, so insert it into all sibling nodes)
			{
// 				recursive_entry(this->parent, iStart, iAmount);
				test2(this->parent, xmin, xmax, ymin, ymax, iStart, iAmount);
			}

			return true;
		}

		return false;
	}

	if ((element_start.size() < maxAmtElements and northWest == nullptr) or this->nodeDepth == maxDepth)	// there is room in the node for this pt. Insert the point only if there is no children node available to sort into or if the maximum depth allowed has been reached
	{
        //std::cout << "Miau" << std::endl;
		element_start.push_back(iStart);
		element_amount.push_back(iAmount);
		return true;
	}

	// maximum amount of pts in this node reached -> split into 4 new nodes
	if (northWest == nullptr)	// this node has not been split yet -> nullptr
	{
		bool sub_ret = subdivide();
		if (sub_ret == false)
		{
			std::cout << "SUB DIV RETURN FALSE" << std::endl;
			exit(1);
		}

		// shuffle all elements which fit into a node completely
		// remove all points from the parent node, and sort this points into the child nodes
		// TODO: dont insert into root node -> insert into this ?!
		for (int i = 0; i < (int)element_start.size(); i++)
		{
			insert(element_start[i], element_amount[i]);
		}

		element_start.clear();
		element_amount.clear();

		// shuffle all shared elements
		for (int i = 0; i < (int)shared_element_start.size(); i++)
		{
			// generate the AABB boundary box
			float xmin = 100.0;
			float xmax = -100.0;
			float ymin = 100.0;
			float ymax = -100.0;

			for (int ii = shared_element_start[i]; ii < (shared_element_start[i]+shared_element_amount[i]); ii++)
			{
				if ((*ptrToX)[ii] > xmax)
					xmax = (*ptrToX)[ii];

				if ((*ptrToX)[ii] < xmin)
					xmin = (*ptrToX)[ii];

				if ((*ptrToY)[ii] > ymax)
					ymax = (*ptrToY)[ii];

				if ((*ptrToY)[ii] < ymin)
					ymin = (*ptrToY)[ii];
			}


// 			recursive_entry(this, shared_element_start[i], shared_element_amount[i]);
			test2(this, xmin, xmax, ymin, ymax, shared_element_start[i], shared_element_amount[i]);
/*
			// check the insertion
			int i0 = this->northEast->shared_element_start.size();
			int i1 = this->northWest->shared_element_start.size();
			int i2 = this->southEast->shared_element_start.size();
			int i3 = this->southWest->shared_element_start.size();

			int minimum1 = std::min( { i0, i1, i2, i3 } );
			int maximum1a = std::max( { i0, i1, i2, i3 } );

			if (minimum1 != maximum1a)
			{
				std::cout << "discrepance between min/max1" << std::endl;
				exit(1);
			}

			int ii0 = this->northEast->shared_element_amount.size();
			int ii1 = this->northWest->shared_element_amount.size();
			int ii2 = this->southEast->shared_element_amount.size();
			int ii3 = this->southWest->shared_element_amount.size();

			int minimum11 = std::min( { ii0, ii1, ii2, ii3 } );
			int maximum11a = std::max( { ii0, ii1, ii2, ii3 } );

			if (minimum11 != maximum11a)
			{
				std::cout << "discrepance between min/max2" << std::endl;
				exit(1);
			}
*/
		}

		shared_element_start.clear();
		shared_element_amount.clear();
	}

	if (northEast->insert(iStart, iAmount)) 
	{
		return true;
	}
	
	if (northWest->insert(iStart, iAmount))
	{
		return true;
	}
	
	if (southWest->insert(iStart, iAmount))
	{
		return true;
	}

	if (southEast->insert(iStart, iAmount))
	{
		return true;
	}

	return false;
}

// split the current node into four new (children)nodes (increment depth by one)
bool Quadtree::subdivide()
{
	if (this->nodeDepth < maxDepth)	// split the node only if the maximum depth has not been reached yet
	{
		// subdivide NW
		std::shared_ptr<BoundaryBox> BB_init_NW(new BoundaryBox(boundary2->cx-boundary2->dim*0.5, boundary2->cy+boundary2->dim*0.5, boundary2->dim*0.5));
		northWest = new Quadtree(std::move(BB_init_NW), this, this->nodeDepth+1);

		// subdivide NE
		std::shared_ptr<BoundaryBox> BB_init_NE(new BoundaryBox(boundary2->cx+boundary2->dim*0.5, boundary2->cy+boundary2->dim*0.5, boundary2->dim*0.5));
		northEast = new Quadtree(std::move(BB_init_NE), this, this->nodeDepth+1);

		// subdivide SE
		std::shared_ptr<BoundaryBox> BB_init_SE(new BoundaryBox(boundary2->cx+boundary2->dim*0.5, boundary2->cy-boundary2->dim*0.5, boundary2->dim*0.5));
		southEast = new Quadtree(std::move(BB_init_SE), this, this->nodeDepth+1);

		// subdivide SW
		std::shared_ptr<BoundaryBox> BB_init_SW(new BoundaryBox(boundary2->cx-boundary2->dim*0.5, boundary2->cy-boundary2->dim*0.5, boundary2->dim*0.5));
		southWest = new Quadtree(std::move(BB_init_SW), this, this->nodeDepth+1);

		return true;
	}

	return false;
}

// draw the tree using OpenGL
void Quadtree::traverse_and_draw(Quadtree *t, float widthRootNode)
{
	// adjust the height (z-coordinate) of the quadtree
	float elevate = -10.0;

	// pick the colors according to the depth
	float depthColor [] = 
	{
		1.0f, 0.0f, 0.0f,		// depth 1 = red
		0.0f, 0.392f, 0.0f,		// depth 2 = darkgreen
		0.0f, 0.0f, 1.0f,		// depth 3 = blue
		1.0f, 0.0f, 1.0f,		// depth 4 = purple
		0.0f, 1.0f, 1.0f, 		// depth 5 = cyan
		0.294f, 0.0f, 0.51f,	// depth 6 = indigo
		0.863f, 0.078f, 0.235f,	// depth 6 = Crimson
	};

	glLineWidth(1.0f);
 
	if (t->northEast != nullptr)
	{
		colorPick(elevate, t, depthColor, sizeof(depthColor)/sizeof(*depthColor));
		t->northEast->traverse_and_draw(northEast, widthRootNode);
	}

	if (t->northWest != nullptr)
	{
		colorPick(elevate, t, depthColor, sizeof(depthColor)/sizeof(*depthColor));
		t->northWest->traverse_and_draw(northWest, widthRootNode);
	}

	if (t->southEast != nullptr)
	{
		colorPick(elevate, t, depthColor, sizeof(depthColor)/sizeof(*depthColor));
		t->southEast->traverse_and_draw(southEast, widthRootNode);
	}

	if (t->southWest != nullptr)
	{
		colorPick(elevate, t, depthColor, sizeof(depthColor)/sizeof(*depthColor));
		t->southWest->traverse_and_draw(southWest, widthRootNode);
	}
}

// count the nodes of the tree
int Quadtree::count_nodes(Quadtree *t)
{
	// node has been split
	if (t->northEast != nullptr)
	{
		int nodes_NE = northEast->count_nodes(northEast);
		int nodes_SE = southEast->count_nodes(southEast);
		int nodes_SW = southWest->count_nodes(southWest);
		int nodes_NW = northWest->count_nodes(northWest);
		return nodes_NE + nodes_SE + nodes_SW + nodes_NW;
	}

	return 1;
}

// count the elements residing in the tree
int Quadtree::count_elements(Quadtree *t)
{
	// TODO: count shared elements too
	int fetch_elements = 0;

	// node has been split - continue with the recursion
	if (t->northEast != nullptr)
	{
		fetch_elements += northEast->count_elements(northEast);
		fetch_elements += southEast->count_elements(southEast);
		fetch_elements += southWest->count_elements(southWest);
		fetch_elements += northWest->count_elements(northWest);
	}
	// deepest (child)node possible
	else
	{
		if (t->element_start.size() > 0)	// there are elements in this node
		{
			fetch_elements += t->element_start.size();
		}
	}

	return fetch_elements;
}







// auxiliary function used by delete_element(). Used to collapse nodes and redistribute elements after collapsing.
void Quadtree::concatenate_nodes(Quadtree *concat_this_node_maybe)
{
	if (concat_this_node_maybe->parent == concat_this_node_maybe)   // point resides in parent -> do nothing
	{
	}
	else
	{
		// Concatenate because all four nodes (3 sibling nodes and the one where the point lies) are leaf nodes (deepest nodes possible)
        if ((concat_this_node_maybe->parent->northEast->northEast == nullptr) && (concat_this_node_maybe->parent->northWest->northEast == nullptr) && (concat_this_node_maybe->parent->southEast->northEast == nullptr) && (concat_this_node_maybe->parent->southWest->northEast == nullptr))
		{
			int amtElemntsNE = concat_this_node_maybe->parent->northEast->element_start.size();
			int amtElemntsNW = concat_this_node_maybe->parent->northWest->element_start.size();
			int amtElemntsSE = concat_this_node_maybe->parent->southEast->element_start.size();
			int amtElemntsSW = concat_this_node_maybe->parent->southWest->element_start.size();

			int amtElemntsNE2 = concat_this_node_maybe->parent->northEast->element_amount.size();
			int amtElemntsNW2 = concat_this_node_maybe->parent->northWest->element_amount.size();
			int amtElemntsSE2 = concat_this_node_maybe->parent->southEast->element_amount.size();
			int amtElemntsSW2 = concat_this_node_maybe->parent->southWest->element_amount.size();

			if ((amtElemntsNE != amtElemntsNE2) or (amtElemntsNW != amtElemntsNW2) or (amtElemntsSE != amtElemntsSE2) or (amtElemntsSW != amtElemntsSW2))
			{
				std::cout << "concat -> not eq: " << std::endl;
				std::cout << amtElemntsNE << "/" << amtElemntsNE2 << "|" << amtElemntsNW << "/" << amtElemntsNW2 << "|" << amtElemntsSE << "/" << amtElemntsSE2 << "|" << amtElemntsSW << "/" << amtElemntsSW2 << std::endl;

				exit(1);
			}

			unsigned int sumElements = amtElemntsNE + amtElemntsNW + amtElemntsSE + amtElemntsSW;

			// move all elements from the leaf nodes into their parents node and delete the leaf nodes
			if (sumElements < maxAmtElements)
			{
				// move element_start and element_amount
				// move elements from the northEast node to the parent node
				for (int i = 0; i < amtElemntsNE; i++)
				{
 					int reshuf_element_start  = concat_this_node_maybe->parent->northEast->element_start[i];
 					int reshuf_element_amount = concat_this_node_maybe->parent->northEast->element_amount[i];

					concat_this_node_maybe->parent->element_start.push_back(reshuf_element_start);
					concat_this_node_maybe->parent->element_amount.push_back(reshuf_element_amount);
				}

				// move elements from the northWest node to the parent node
				for (int i = 0; i < amtElemntsNW; i++)
				{
 					int reshuf_element_start  = concat_this_node_maybe->parent->northWest->element_start[i];
 					int reshuf_element_amount = concat_this_node_maybe->parent->northWest->element_amount[i];

					concat_this_node_maybe->parent->element_start.push_back(reshuf_element_start);
					concat_this_node_maybe->parent->element_amount.push_back(reshuf_element_amount);
				}

				// move elements from the southEast node to the parent node
				for (int i = 0; i < amtElemntsSE; i++)
				{
 					int reshuf_element_start = concat_this_node_maybe->parent->southEast->element_start[i];
 					int reshuf_element_amount = concat_this_node_maybe->parent->southEast->element_amount[i];

					concat_this_node_maybe->parent->element_start.push_back(reshuf_element_start);
					concat_this_node_maybe->parent->element_amount.push_back(reshuf_element_amount);
				}

				// move elements from the southWest node to the parent node
				for (int i = 0; i < amtElemntsSW; i++)
				{
 					int reshuf_element_start  = concat_this_node_maybe->parent->southWest->element_start[i];
 					int reshuf_element_amount = concat_this_node_maybe->parent->southWest->element_amount[i];

					concat_this_node_maybe->parent->element_start.push_back(reshuf_element_start);
					concat_this_node_maybe->parent->element_amount.push_back(reshuf_element_amount);
				}

				// move shared_element_start and shared_element_amount
//  				int amtElemntsNE = concat_this_node_maybe->parent->northEast->shared_element_start.size();
//  				int amtElemntsNW = concat_this_node_maybe->parent->northWest->shared_element_start.size();
//  				int amtElemntsSE = concat_this_node_maybe->parent->southEast->shared_element_start.size();
// 				int amtElemntsSW = concat_this_node_maybe->parent->southWest->shared_element_start.size();


				std::set< std::pair<int, int> > insert_shared_elements;
				std::set< std::pair<int, int> > insert_full_elements;


				// shared space -> NE
				for (int i = 0; i < (int)concat_this_node_maybe->parent->northEast->shared_element_start.size(); i++)
				{
 					int reshuf_element_start1  = concat_this_node_maybe->parent->northEast->shared_element_start[i];
 					int reshuf_element_amount1 = concat_this_node_maybe->parent->northEast->shared_element_amount[i];

					// determine, if the element now resides completely in the node -> insert into element_start and element_amount
					int count_inside = reshuf_element_amount1;

					// check if all the element can be fit completely into the node -> if an previous shared element fits completely -> insert it into the "full-fit" vec, i.e., the element_start.- and element_amount.- vecs
					for (int i = reshuf_element_start1; i < reshuf_element_start1+reshuf_element_amount1; i++)
					{
						float bdim = concat_this_node_maybe->parent->boundary2->dim;
						float bcx  = concat_this_node_maybe->parent->boundary2->cx;
						float bcy  = concat_this_node_maybe->parent->boundary2->cy;

						if (((*ptrToX)[i] > bcx+bdim) or ((*ptrToX)[i] <= bcx-bdim) or ((*ptrToY)[i] > bcy+bdim) or ((*ptrToY)[i] <= bcy-bdim))
						{
							count_inside--;
						}
					}

					int concatenate_retrieve_element = 0;

					if (count_inside == reshuf_element_amount1)
					{
						concatenate_retrieve_element = 1;
						insert_full_elements.insert(std::make_pair(reshuf_element_start1, reshuf_element_amount1));
// 						concat_this_node_maybe->parent->element_start.push_back(reshuf_element_start1);
// 						concat_this_node_maybe->parent->element_amount.push_back(reshuf_element_amount1);
					}
					else
					{
						concatenate_retrieve_element = 2;
						insert_shared_elements.insert(std::make_pair(reshuf_element_start1, reshuf_element_amount1));
// 						concat_this_node_maybe->parent->shared_element_start.push_back(reshuf_element_start1);
// 						concat_this_node_maybe->parent->shared_element_amount.push_back(reshuf_element_amount1);
					}

					if (concatenate_retrieve_element == 0)
					{
						std::cout << "concatenate_nodes -> error retrievint element NE" << std::endl;
						exit(1);
					}
				}


				// shared space -> NW
				for (int i = 0; i < (int)concat_this_node_maybe->parent->northWest->shared_element_start.size(); i++)
				{
 					int reshuf_element_start1  = concat_this_node_maybe->parent->northWest->shared_element_start[i];
 					int reshuf_element_amount1 = concat_this_node_maybe->parent->northWest->shared_element_amount[i];

					// determine, if the element now resides completely in the node -> insert into element_start and element_amount
					int count_inside = reshuf_element_amount1;

					// check if all the element can be fit completely into the node -> if an previous shared element fits completely -> insert it into the "full-fit" vec, i.e., the element_start.- and element_amount.- vecs
					for (int i = reshuf_element_start1; i < reshuf_element_start1+reshuf_element_amount1; i++)
					{
						float bdim = concat_this_node_maybe->parent->boundary2->dim;
						float bcx  = concat_this_node_maybe->parent->boundary2->cx;
						float bcy  = concat_this_node_maybe->parent->boundary2->cy;

						if (((*ptrToX)[i] > bcx+bdim) or ((*ptrToX)[i] <= bcx-bdim) or ((*ptrToY)[i] > bcy+bdim) or ((*ptrToY)[i] <= bcy-bdim))
						{
							count_inside--;
						}
					}

					int concatenate_retrieve_element = 0;

					if (count_inside == reshuf_element_amount1)
					{
						concatenate_retrieve_element = 1;
						insert_full_elements.insert(std::make_pair(reshuf_element_start1, reshuf_element_amount1));
// 						concat_this_node_maybe->parent->element_start.push_back(reshuf_element_start1);
// 						concat_this_node_maybe->parent->element_amount.push_back(reshuf_element_amount1);
					}
					else
					{
						concatenate_retrieve_element = 2;
						insert_shared_elements.insert(std::make_pair(reshuf_element_start1, reshuf_element_amount1));
// 						concat_this_node_maybe->parent->shared_element_start.push_back(reshuf_element_start1);
// 						concat_this_node_maybe->parent->shared_element_amount.push_back(reshuf_element_amount1);
					}

					if (concatenate_retrieve_element == 0)
					{
						std::cout << "concatenate_nodes -> error retrievint element NE" << std::endl;
						exit(1);
					}
				}



				// shared space -> SE
				for (int i = 0; i < (int)concat_this_node_maybe->parent->southEast->shared_element_start.size(); i++)
				{
 					int reshuf_element_start1  = concat_this_node_maybe->parent->southEast->shared_element_start[i];
 					int reshuf_element_amount1 = concat_this_node_maybe->parent->southEast->shared_element_amount[i];

					// determine, if the element now resides completely in the node -> insert into element_start and element_amount
					int count_inside = reshuf_element_amount1;

					// check if all the element can be fit completely into the node -> if an previous shared element fits completely -> insert it into the "full-fit" vec, i.e., the element_start.- and element_amount.- vecs
					for (int i = reshuf_element_start1; i < reshuf_element_start1+reshuf_element_amount1; i++)
					{
						float bdim = concat_this_node_maybe->parent->boundary2->dim;
						float bcx  = concat_this_node_maybe->parent->boundary2->cx;
						float bcy  = concat_this_node_maybe->parent->boundary2->cy;

						if (((*ptrToX)[i] > bcx+bdim) or ((*ptrToX)[i] <= bcx-bdim) or ((*ptrToY)[i] > bcy+bdim) or ((*ptrToY)[i] <= bcy-bdim))
						{
							count_inside--;
						}
					}

					int concatenate_retrieve_element = 0;

					if (count_inside == reshuf_element_amount1)
					{
						concatenate_retrieve_element = 1;
						insert_full_elements.insert(std::make_pair(reshuf_element_start1, reshuf_element_amount1));
// 						concat_this_node_maybe->parent->element_start.push_back(reshuf_element_start1);
// 						concat_this_node_maybe->parent->element_amount.push_back(reshuf_element_amount1);
					}
					else
					{
						concatenate_retrieve_element = 2;
						insert_shared_elements.insert(std::make_pair(reshuf_element_start1, reshuf_element_amount1));
// 						concat_this_node_maybe->parent->shared_element_start.push_back(reshuf_element_start1);
// 						concat_this_node_maybe->parent->shared_element_amount.push_back(reshuf_element_amount1);
					}

					if (concatenate_retrieve_element == 0)
					{
						std::cout << "concatenate_nodes -> error retrievint element NE" << std::endl;
						exit(1);
					}
				}




				// shared space -> SW
				for (int i = 0; i < (int)concat_this_node_maybe->parent->southWest->shared_element_start.size(); i++)
				{
 					int reshuf_element_start1  = concat_this_node_maybe->parent->southWest->shared_element_start[i];
 					int reshuf_element_amount1 = concat_this_node_maybe->parent->southWest->shared_element_amount[i];

					// determine, if the element now resides completely in the node -> insert into element_start and element_amount
					int count_inside = reshuf_element_amount1;

					// check if all the element can be fit completely into the node -> if an previous shared element fits completely -> insert it into the "full-fit" vec, i.e., the element_start.- and element_amount.- vecs
					for (int i = reshuf_element_start1; i < reshuf_element_start1+reshuf_element_amount1; i++)
					{
						float bdim = concat_this_node_maybe->parent->boundary2->dim;
						float bcx  = concat_this_node_maybe->parent->boundary2->cx;
						float bcy  = concat_this_node_maybe->parent->boundary2->cy;

						if (((*ptrToX)[i] > bcx+bdim) or ((*ptrToX)[i] <= bcx-bdim) or ((*ptrToY)[i] > bcy+bdim) or ((*ptrToY)[i] <= bcy-bdim))
						{
							count_inside--;
						}
					}

					int concatenate_retrieve_element = 0;

					if (count_inside == reshuf_element_amount1)
					{
						concatenate_retrieve_element = 1;
						insert_full_elements.insert(std::make_pair(reshuf_element_start1, reshuf_element_amount1));
// 						concat_this_node_maybe->parent->element_start.push_back(reshuf_element_start1);
// 						concat_this_node_maybe->parent->element_amount.push_back(reshuf_element_amount1);
					}
					else
					{
						concatenate_retrieve_element = 2;
						insert_shared_elements.insert(std::make_pair(reshuf_element_start1, reshuf_element_amount1));
// 						concat_this_node_maybe->parent->shared_element_start.push_back(reshuf_element_start1);
// 						concat_this_node_maybe->parent->shared_element_amount.push_back(reshuf_element_amount1);
					}

					if (concatenate_retrieve_element == 0)
					{
						std::cout << "concatenate_nodes -> error retrievint element NE" << std::endl;
						exit(1);
					}
				}

				// push the retrieved elements into the full space of the parent node
				// create an iterator for the std::pair
				std::set< std::pair<int, int> >::iterator it1;

				for(it1 = insert_full_elements.begin(); it1 != insert_full_elements.end(); ++it1)
				{
					concat_this_node_maybe->parent->element_start.push_back(it1->first);
					concat_this_node_maybe->parent->element_amount.push_back(it1->second);
				}

				// push the retrieved elements into the shared space of the parent node
				// create an iterator for the std::pair
				std::set< std::pair<int, int> >::iterator it2;

				for(it2 = insert_shared_elements.begin(); it2 != insert_shared_elements.end(); ++it2)
				{
					concat_this_node_maybe->parent->shared_element_start.push_back(it2->first);
					concat_this_node_maybe->parent->shared_element_amount.push_back(it2->second);
				}


				// generate a pointer to the next node to concatenate (prevents an invalid read)
				Quadtree *concat_next = concat_this_node_maybe->parent;

				// delete the sibling nodes (of the removed point)
				concat_this_node_maybe->parent->clearNode();

				// proceed with the recursion
				concatenate_nodes(concat_next);
			}
		}
	}
}






// remove a single element from the tree
bool Quadtree::delete_element(int iStart, int iAmount)
{
	// try to locate the node where the point lies
	Quadtree *fetch_node = fetch_deepest_node(iStart, iAmount);


	if (fetch_node == nullptr)   // this element is not in the QT
	{
		std::cout << "delete_element -> cant find node corresponding to the element" << std::endl;
		exit(1);
	}
	else
	{
		// element resides in a leafnode, i.e., a deepest node possible. This means the element fits completely into a single (leaf)node. Remove the element from element_start and element_amount and then check, whether this was the only element (if this is the case, this node may be concatenated)

		// element fits completely into a single node or one element, which does not fit into a single node or it  resides in the shared space of the root node
		if (fetch_node->northEast == nullptr)
		{
			// try to locate the element in the element_start vector (which means the element fits completely into the retrieved node)
			int i = 0;
 			int k = 0;

			bool found_i = false;
   			bool found_k = false;

			for (i = 0; i < (int)fetch_node->element_start.size(); i++)
			{
				if (fetch_node->element_start[i] == iStart and fetch_node->element_amount[i] == iAmount)
				{
					found_i = true;
					break;
				}
			}

			// last element in the QT may reside in the shared space
			for (k = 0; k < (int)fetch_node->shared_element_start.size(); k++)
			{
				if (fetch_node->shared_element_start[k] == iStart and fetch_node->shared_element_amount[k] == iAmount)
				{
  					found_k = true;
					break;
				}
			}

			// element is in the "element_start" vector -> delete from element_start and element_amount
			if (found_i == true)
			{
				fetch_node->element_start.erase(fetch_node->element_start.begin()+i);
				fetch_node->element_amount.erase(fetch_node->element_amount.begin()+i);

				if ((int)fetch_node->element_start.size() != (int)fetch_node->element_amount.size())
				{
					std::cout << "mismatch" << std::endl;
					exit(1);
				}

				// this was the only element in the node -> concatenate
				if ((int)fetch_node->element_start.size() == 0)
				{
// 					std::cout << "CONTAT" << std::endl;
 					concatenate_nodes(fetch_node);
				}
			}
			// e.g. we remove the last element and this element is in the shared space of the root node (and the root node is the only node there is)

			else if (found_k == true)
			{
				fetch_node->shared_element_start.erase(fetch_node->shared_element_start.begin()+k);
				fetch_node->shared_element_amount.erase(fetch_node->shared_element_amount.begin()+k);
			}
			// element was neither in the shared nor in the regular vector
			else
			{
				std::cout << "delete_element() -> element was found neither in the complete nor in the shared space" << std::endl;
 				exit(1);
			}

			return true;
			// TODO -> check if everything worked properly ?!
		}
		// element resides in multiple nodes because it does not fit completely into a single node. Erase the given element from shared_element_start and shared_element_amount recursively
		else
		{
			// generate the AABB boundary box
			float xmin = 100.0;
			float xmax = -100.0;
			float ymin = 100.0;
			float ymax = -100.0;

			for (int i = iStart; i < (iStart+iAmount); i++)
			{
				if ((*ptrToX)[i] > xmax)
					xmax = (*ptrToX)[i];

				if ((*ptrToX)[i] < xmin)
					xmin = (*ptrToX)[i];

				if ((*ptrToY)[i] > ymax)
					ymax = (*ptrToY)[i];

				if ((*ptrToY)[i] < ymin)
					ymin = (*ptrToY)[i];
			}

			// remove the element from all subnodes it resides
			recursive_removeAABB(fetch_node, xmin, xmax, ymin, ymax, iStart, iAmount);

			// catch cases, where a node has been split, but all elements reside in the shared space of the four subnodes

			// Check whether the node, from which the element was removed, has only four subnodes. If there are only elements in the shared-vectors this may result in the node not being concatenated.
			if ((fetch_node->northEast->northEast == nullptr) && (fetch_node->northWest->northEast == nullptr) && (fetch_node->southEast->northEast == nullptr) && (fetch_node->southWest->northEast == nullptr))
			{
				int amt_el_start_NE = fetch_node->northEast->element_start.size();
				int amt_el_start_NW = fetch_node->northWest->element_start.size();
				int amt_el_start_SE = fetch_node->southEast->element_start.size();
				int amt_el_start_SW = fetch_node->southWest->element_start.size();

				// concatenate 
				if ((amt_el_start_NE == 0) and (amt_el_start_NW == 0) and (amt_el_start_SE == 0) and (amt_el_start_SW == 0))
				{
					concatenate_nodes(fetch_node->northEast);
				}
			}

			return true;
		}
	}
	return false;

}

// visualizes the nodes, which can be concatenated (colored) and the nodes which only inherits elements in the shared space (shared_element_start, shared_element_amount). The latter are colored grey.
void Quadtree::find_concatenable_shared_nodes(Quadtree *t)
{
	// deepest node of the QT reached
	if ((t->northEast == nullptr) && (t->parent != t))
	{
 		if ((t->parent->northWest->northWest == nullptr) and (t->parent->southEast->northWest == nullptr) and (t->parent->southWest->northWest == nullptr) and (t->parent->northEast->northWest == nullptr))
		{
			// draw
			int s1 = t->parent->northWest->element_start.size();
			int s2 = t->parent->northEast->element_start.size();
			int s3 = t->parent->southWest->element_start.size();
			int s4 = t->parent->southEast->element_start.size();

			bool color_overwrite = false;

			if ((s1 == 0) and (s2 == 0) and (s3 == 0) and (s4 == 0))
			{
				color_overwrite = true;
			}

			// DRAW
			float elevate = -10.0;

			float centerx = t->parent->northWest->boundary2->cx;
			float centery = t->parent->northWest->boundary2->cy;
			float dim = t->parent->northWest->boundary2->dim;

			if (color_overwrite == false)
				glColor4f(1.0f, 0.0f, 0.0f, 0.15f);
			else
			{
				glColor4f(0.0f, 0.0f, 0.0f, 0.25f);
			}

			glBegin(GL_TRIANGLE_STRIP);
				glVertex3f(centerx+dim, centery+dim, elevate);
				glVertex3f(centerx+dim, centery-dim, elevate);
				glVertex3f(centerx-dim, centery+dim, elevate);
				glVertex3f(centerx-dim, centery-dim, elevate);
			glEnd();

			centerx = t->parent->northEast->boundary2->cx;
			centery = t->parent->northEast->boundary2->cy;
			dim = t->parent->northEast->boundary2->dim;

			if (color_overwrite == false)
				glColor4f(0.0f, 1.0f, 0.0f, 0.15f);
			else
			{
				glColor4f(0.0f, 0.0f, 0.0f, 0.25f);
			}

			glBegin(GL_TRIANGLE_STRIP);
				glVertex3f(centerx+dim, centery+dim, elevate);
				glVertex3f(centerx+dim, centery-dim, elevate);
				glVertex3f(centerx-dim, centery+dim, elevate);
				glVertex3f(centerx-dim, centery-dim, elevate);
			glEnd();

			centerx = t->parent->southWest->boundary2->cx;
			centery = t->parent->southWest->boundary2->cy;
			dim = t->parent->southWest->boundary2->dim;

			if (color_overwrite == false)
				glColor4f(0.0f, 0.0f, 1.0f, 0.15f);
			else
			{
				glColor4f(0.0f, 0.0f, 0.0f, 0.25f);
			}

			glBegin(GL_TRIANGLE_STRIP);
				glVertex3f(centerx+dim, centery+dim, elevate);
				glVertex3f(centerx+dim, centery-dim, elevate);
				glVertex3f(centerx-dim, centery+dim, elevate);
				glVertex3f(centerx-dim, centery-dim, elevate);
			glEnd();

			centerx = t->parent->southEast->boundary2->cx;
			centery = t->parent->southEast->boundary2->cy;
			dim = t->parent->southEast->boundary2->dim;

			if (color_overwrite == false)
				glColor4f(1.0f, 0.0f, 1.0f, 0.15f);
			else
			{
				glColor4f(0.0f, 0.0f, 0.0f, 0.25f);
			}

			glBegin(GL_TRIANGLE_STRIP);
				glVertex3f(centerx+dim, centery+dim, elevate);
				glVertex3f(centerx+dim, centery-dim, elevate);
				glVertex3f(centerx-dim, centery+dim, elevate);
				glVertex3f(centerx-dim, centery-dim, elevate);
			glEnd();
		}

		return;
	}
	else
	{
		// anything NOT in the deepest node should evoke an error
		if (t->parent != t)
		{
			if (t->shared_element_start.size() > 0 or t->shared_element_amount.size() > 0 or t->element_start.size() > 0 or t->element_amount.size() > 0)
			{
				std::cout << "elements not in deepest node" << std::endl;
	 			exit(1);
			}
		}

		if (t->northEast != nullptr)
			find_concatenable_shared_nodes(t->northEast);

		if (t->northWest != nullptr)
			find_concatenable_shared_nodes(t->northWest);

		if (t->southEast != nullptr)
			find_concatenable_shared_nodes(t->southEast);

		if (t->southWest != nullptr)
			find_concatenable_shared_nodes(t->southWest);
	}
}

// prints the tree (amount of elements in the vectors and the pointers to the nodes)
void Quadtree::print_tree()
{
	std::cout << "print tree(ROOT): " << this << " | FULL: " << element_start.size()  << " / " << element_amount.size() << " ||| SHARED: " << shared_element_start.size() << " / " << shared_element_amount.size() << std::endl;

	if (this->northWest != nullptr)
	{
		std::cout << "NW: ";
		northWest->print_tree();
	}

	if (this->northEast != nullptr)
	{
		std::cout << "NE: ";
		northEast->print_tree();
	}

	if (this->southWest != nullptr)
	{
		std::cout << "SW: ";
		southWest->print_tree();
	}

	if (this->southEast != nullptr)
	{
		std::cout << "SE: ";
		southEast->print_tree();
	}
}
