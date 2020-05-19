package org.firstinspires.ftc.teamcode.Mapping;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.Stack;

import org.firstinspires.ftc.teamcode.AngleUtils;

public class NBMap implements Serializable {
	// A node-branch Map
	private ArrayList<Node> nodes;

	// currently unreliable & unused
	private Set<Branch> branches;

	public NBMap() {
		nodes = new ArrayList<>();
		branches = new HashSet<>();
	}

	// directly inserts a new node w/o checking for duplicate; should only be used
	// by internal methods
	private void addNode(Node node) {
		nodes.add(node);
		branches.addAll(node.getAttachedLines());
	}

	// directly inserts a new branch w/o checking for duplicate; should only be used
	// by internal methods
	private void addBranch(Branch branch) {
		branches.add(branch);
	}

	public Set<Branch> getBranches() {
		Set<Branch> result = new HashSet<>();
		for (Node n : nodes) {
			result.addAll(n.getAttachedLines());
		}
		return result;
	}

	public ArrayList<Node> getNodes() {
		return nodes;
	}

    public Node getRandomExcludedNode(Node currentNode) {
		ArrayList<Node> randNodes = (ArrayList<Node>)nodes.clone();
		randNodes.remove(currentNode);
		return randNodes.get((int)(Math.random()*randNodes.size()));
    }

    public ArrayList<Branch> getStubs(){
		ArrayList<Branch> result = new ArrayList<>();
		for (Branch b : getBranches()){
			if (b.getIsStub()){
				result.add(b);
			}
		}
		return result;
	}

    private class scoreComparer implements Comparator<Node> {
		private Map<Node, Double> score;

		public scoreComparer(Map<Node, Double> gScore) {
			this.score = gScore;
		}

		@Override
		public int compare(Node o1, Node o2) {
			double g1Score = (score.containsKey(o1) ? score.get(o1) : Double.MAX_VALUE);
			double g2Score = (score.containsKey(o2) ? score.get(o2) : Double.MAX_VALUE);
			return Double.compare(g1Score, g2Score);
		}
	}

	public Stack<Branch> getRoute(Node start, Node end) {
		Map<Node, Double> gScore = new HashMap<>();
		gScore.put(start, 0.0);

		Map<Node, Double> fScore = new HashMap<>();
		gScore.put(start, AngleUtils.distance(start.getApproxPos(), end.getApproxPos()));

		PriorityQueue<Node> openSet = new PriorityQueue<>(1, new scoreComparer(fScore));
		openSet.add(start);
		ArrayList<Node> closedSet = new ArrayList<>();

		Map<Node, Node> cameFrom = new HashMap<>();

		Node current;
		while (openSet.size() != 0) {
			current = openSet.poll();
			if (current.equals(end)) {
				Stack<Branch> result = new Stack<>();
				Node previous;
				while (cameFrom.containsKey(current)) {
					previous = cameFrom.get(current);
					result.add(previous.getBranchTo(current));
					current = previous;
				}
				return result;
			}
			closedSet.add(current);
			for (Node n : current.getAdjacentNodes()) {
				double tentative_gScore = gScore.get(current) + 1;
				if (tentative_gScore < (gScore.containsKey(n) ? gScore.get(n) : Double.MAX_VALUE)) {
					cameFrom.put(n, current);
					gScore.put(n, tentative_gScore);
					fScore.put(n, tentative_gScore + AngleUtils.distance(n.getApproxPos(), end.getApproxPos()));
					if (!closedSet.contains(n)) {
						openSet.add(n);
					}
				}
			}
		}
		return null;
	}

	public Stack<Branch> pathToNearestStub(Node start) {

		Map<Node, Double> gScore = new HashMap<>();
		gScore.put(start, 0.0);
		PriorityQueue<Node> openSet = new PriorityQueue<>(1, new scoreComparer(gScore));
		openSet.add(start);
		ArrayList<Node> closedSet = new ArrayList<>();

		Map<Node, Node> cameFrom = new HashMap<>();

		Node current;
		while (openSet.size() != 0) {
			current = openSet.poll();
			if (current.hasStub()) {
				Stack<Branch> result = new Stack<>();

				Node previous;
				if (cameFrom.containsKey(current)) {
					double throughAngle = cameFrom.get(current).getBranchTo(current).getEndAngle();
					result.add(current.getStubs()
							.get(AngleUtils.nearestAngle(throughAngle, current.getOutwardStubAngles())));
				} else
					result.add(current.getStubs().get(0));
				while (cameFrom.containsKey(current)) {
					previous = cameFrom.get(current);
					result.add(previous.getBranchTo(current));
					current = previous;
				}
				return result;
			}

			closedSet.add(current);
			for (Node n : current.getAdjacentNodes()) {
				double tentative_gScore = gScore.get(current) + 1;
				if (tentative_gScore < (gScore.containsKey(n) ? gScore.get(n) : Double.MAX_VALUE)) {
					cameFrom.put(n, current);
					gScore.put(n, tentative_gScore);
					if (!closedSet.contains(n)) {
						openSet.add(n);
					}
				}
			}
		}

		return null;
	}

	/*
	 * public Node locateNode(Node target){ return locateNode(target,false,0);
	 * 
	 * }
	 */

	public Node locateNode(Node target) {
		return locateNode(target, false);
	}

	/*
	 * public Node locateNode(Node target, boolean insertIfNotFound){ return
	 * locateNode(target,insertIfNotFound,0); }
	 */

	public Node locateNode(Node target, boolean insertIfNotFound) {
		ArrayList<Node> overlappedNodes = new ArrayList<>();
		for (Node node : nodes) {
			if (node.equals(target)) {
				return target;
			}
			if (node.isMergeable(target)) {
				overlappedNodes.add(node);
			}
		}
		if (overlappedNodes.size() > 0) {
			if (overlappedNodes.size() == 1) {
				return overlappedNodes.get(0);
			} else {
				return getBestMerge(overlappedNodes, target);
			}
		}
		if (insertIfNotFound) {
			insertNode(target);
		}
		return target;
	}

	// attempts to create a new node from scratch based off of a source stub and the
	// final position of the node,
	// and inserts it into the network
	public Node createNode(Branch sourceStub, double endAngle, double[] approxPos, double[] otherAngles,
			double currentPositionError) {
		assert sourceStub.getIsStub();
		Node tempNode = new Node(approxPos, otherAngles, currentPositionError);
		sourceStub.growStub(tempNode, endAngle);
		tempNode.attachLine(sourceStub);
		return insertNode(tempNode);
	}

	// attempts to create a new node from scratch based off of a source stub and the
	// final position of the node,
	// and inserts it into the network
	public Node createNode(Branch sourceStub, double endAngle, double totalAngle, double[] approxPos,
			double[] otherAngles, double currentPositionError) {
		Node tempNode = new Node(approxPos, otherAngles, currentPositionError);
		sourceStub.growStub(tempNode, endAngle, totalAngle);
		tempNode.attachLine(sourceStub);
		return insertNode(tempNode);

	}

	public boolean containsNode(Node node) {
		return nodes.contains(node);
	}

	private boolean removeNode(Node n){
		return nodes.remove(n);
	}

	// attempts to insert the node into the network; if another node is found that
	// is considered similar enough to be
	// the same, will merge the incoming node with that node.
	// returns the either the input node, if unique, or the node it was merged into
	public Node insertNode(Node insertNode) {
		ArrayList<Node> overlappedNodes = new ArrayList<>();
		for (Node node : nodes) {
			/*
			 * only look at potential NEW merges, so that the recursive algorithm can merge
			 * nodes that are already in the network still
			 */
			if (node.isMergeable(insertNode) && !node.equals(insertNode)) {
				overlappedNodes.add(node);
			}
		}
		Node mergeInto;
		if (overlappedNodes.size() > 0) {
			if (overlappedNodes.size() == 1) {
				mergeInto = overlappedNodes.get(0);
			} else {
				mergeInto = getBestMerge(overlappedNodes, insertNode);
			}
			if (containsNode(insertNode)){
				removeNode(insertNode);
			}
			mergeInto.merge(insertNode);

			// recursively attempt to merge the new node into the network since it has moved
			return insertNode(mergeInto);
		} else {
			if (!containsNode(insertNode)) {
				addNode(insertNode);
			}
		}
		return insertNode;
	}

	public Node getBestMerge(ArrayList<Node> mergeNodes, Node insertNode) {
		double bestScore = Double.MAX_VALUE;
		Node bestNode = mergeNodes.get(0);
		for (Node n : mergeNodes) {
			double score = Node.getBestAngleAssociationScore(n, insertNode);
			if (score < bestScore) {
				bestNode = n;
				bestScore = score;
			}
		}
		return bestNode;
	}

	@Override
	public String toString() {
		return "Nodes: " + Arrays.toString(nodes.toArray()) + ", Branches: " + Arrays.toString(getBranches().toArray());
	}
}
