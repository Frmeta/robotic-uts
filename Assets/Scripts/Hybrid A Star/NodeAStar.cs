using UnityEngine;
using System.Collections;
using System;

//One node in the A star algorithm
public class NodeAStar : IHeapItem<NodeAStar>
{
    //The cost to this node
    public float cost;

    public Vector2Int position;
    //The direction in radians

    //The index this node has in the heap, to make sorting nodes faster
    private int heapIndex;


    public NodeAStar()
    {

    }


    public NodeAStar(Vector2Int position, float cost)
    {
        this.position = position;
        this.cost = cost;
    }


    //The total cost including heuristic (f = g + h)
    public float fCost
    {
        get { return cost; }
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
    public int CompareTo(NodeAStar nodeToCompare)
    {
        int compare = fCost.CompareTo(nodeToCompare.fCost);

        return -compare;
    }
}