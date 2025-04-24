using UnityEngine;
using System.Collections;
using System;

//One node in the A star algorithm
public class Node : IHeapItem<Node>
{
    //The cost to this node
    public float gCost;
    //The estimated cost to the goal from this node = the heuristics
    //Needed so we can calculate the f cost easier
    public float hCost;

    public Vector2 position;
    //The direction in radians
    public float direction;
    //Is the car reversing when traveling to this node?
    public bool isReversing;

    //The node we took to get here so we can get the final path
    public Node previousNode;

    //The index this node has in the heap, to make sorting nodes faster
    private int heapIndex;


    public Node()
    {

    }


    public Node(Node previousNode, Vector2 position, float direction, bool isReversing, float gCost, float hCost)
    {
        this.previousNode = previousNode;
        this.position = position;
        this.direction = direction;
        this.isReversing = isReversing;
        this.gCost = gCost;
        this.hCost = hCost;
    }


    //The total cost including heuristic (f = g + h)
    public float fCost
    {
        get { return gCost + hCost; }
    }

    //Take the data from this node and add it to another node
    public void StealDataFromThisNode(Node other)
    {
        other.gCost = gCost;
        other.hCost = hCost;
        other.direction = direction;
        other.isReversing = isReversing;
        other.previousNode = previousNode;
    }


    //The heap requires that we implement this
    public int HeapIndex
    {
        get
        {
            return heapIndex;
        }
        set
        {
            heapIndex = value;
        }
    }



    //To compare nodes when sorting the heap
    public int CompareTo(Node nodeToCompare)
    {
        int compare = fCost.CompareTo(nodeToCompare.fCost);

        //If they are equal, use the one that is the closest
        //Will return 1, 0 or -1, so 0 means the f costs are the same
        if (compare == 0)
        {
            compare = hCost.CompareTo(nodeToCompare.hCost);
        }

        return -compare;
    }
}