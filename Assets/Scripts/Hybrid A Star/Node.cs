using UnityEngine;
using System.Collections;
using System;

//One node in the A star algorithm, placed in heap
public class Node : IHeapItem<Node>
{

    public Vector3 worldPosition;
    //The direction in radians
    public float direction;
    //Is the car reversing when traveling to this node?
    //The cost to this node
    public float costStartToNode;
    public float costHeuristicNodeToEnd;
    public float fCost {
        get{
            return costStartToNode + costHeuristicNodeToEnd;
        }
    }
    public bool isReversing;

    //The node we took to get here so we can get the final path
    public Node previousNode;

    //The index this node has in the heap, to make sorting nodes faster
    private int heapIndex;
    public int countdownBeforeCanTurn;


    public Node(Node previousNode, Vector3 worldPosition, float direction, bool isReversing, float costStartToNode, float costHeuristicNodeToEnd, int countdownBeforeCanTurn)
    {
        this.previousNode = previousNode;
        this.worldPosition = worldPosition;
        this.direction = direction;
        this.isReversing = isReversing;
        this.costStartToNode = costStartToNode;
        this.costHeuristicNodeToEnd = costHeuristicNodeToEnd;
        this.countdownBeforeCanTurn = countdownBeforeCanTurn;
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
            compare = costHeuristicNodeToEnd.CompareTo(nodeToCompare.costHeuristicNodeToEnd);
        }

        return -compare;
    }
}